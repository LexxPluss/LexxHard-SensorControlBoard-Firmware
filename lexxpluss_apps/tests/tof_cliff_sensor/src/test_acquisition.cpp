/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Tests for the acquisition skeleton. Every one of these pins a boundary that is easy to
 * lose later: the neutrality of the facts, the publication gate, the independence of the
 * heartbeat from the acquisition path, the lock discipline, and the absence of an enable
 * operation.
 *
 * The chain lock is faked here rather than linking the real chain controller, which
 * would drag in the bus and the shell. The fake is a real k_mutex, so the assertions
 * about the lock being held are assertions about the production code path.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "tof_acquisition.hpp"

namespace lexxhard::tof_chain_controller {

k_mutex &chain_lock()
{
    static k_mutex m;
    static bool ready{false};

    if (!ready) {
        ready = true;
        k_mutex_init(&m);
    }
    return m;
}

}  // namespace lexxhard::tof_chain_controller

namespace {

namespace acq = lexxhard::tof_acq;

constexpr uint32_t kCyclePeriodMs{50};
constexpr uint32_t kHealthPeriodMs{20};

// Scripted behaviour and recorded traffic for the faked device ops.
struct fake_dev {
    int open_rc{0};
    int configure_rc{0};
    int start_rc{0};
    int read_rc{0};
    bool fresh{false};
    int16_t mm{0};
    uint8_t status_code{0};
    bool rearm_failed{false};
    uint32_t open_delay_ms{0};

    uint32_t read_delay_ms{0};

    int open_calls{0};
    int start_calls{0};
    int read_calls{0};
    int stop_calls{0};
    bool lock_held_in_open{false};
    bool lock_held_in_read{false};
};

/* Every device op records who called it. The ownership rule -- while a thread owns the ULD, only that
 * thread may drive it -- is not observable any other way: the chain lock serialises callers but says
 * nothing about how many there were, and the ULD's port keeps ONE transport record, so two callers
 * inside it produce a transport error reported as a good sample. */
k_tid_t uld_callers[16];
int uld_call_count;

/* Given from inside a cycle, so a test can tell where in the cadence the thread is. Without it,
 * "between cycles" can only be approximated by sleeping, and a test that requests a stop at an
 * unknown point cannot distinguish "no cycle began after the request" from "the cycle in flight
 * finished" -- which is exactly the distinction the acceptance boundary is about. */
K_SEM_DEFINE(cycle_read_seen, 0, 1);

void record_caller()
{
    if (uld_call_count < static_cast<int>(sizeof uld_callers / sizeof uld_callers[0]))
        uld_callers[uld_call_count++] = k_current_get();
}

bool every_uld_call_came_from(k_tid_t who)
{
    for (int i{0}; i < uld_call_count; ++i) {
        if (uld_callers[i] != who)
            return false;
    }
    return uld_call_count > 0;
}

fake_dev devs[acq::kMaxSources];
/* Real stream states: the acquisition layer must hand each source its own. */
struct tof_cliff_stream_state streams[acq::kMaxSources];

bool lock_is_held()
{
    return lexxhard::tof_chain_controller::chain_lock().owner == k_current_get();
}

int fake_open(void *dev, uint8_t, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.open_calls;
    record_caller();
    d.lock_held_in_open = lock_is_held();
    memset(st, 0, sizeof(*st));
    if (d.open_delay_ms != 0) {
        // Stands in for a device that answers slowly or not at all. The heartbeat has to
        // keep going through this.
        k_msleep(d.open_delay_ms);
    }
    if (d.open_rc != 0)
        st->stage = TOF_CLIFF_STAGE_BOOT;
    return d.open_rc;
}

int fake_configure(void *dev, acq::op_status *st)
{
    memset(st, 0, sizeof(*st));
    return static_cast<fake_dev *>(dev)->configure_rc;
}

int fake_start(void *dev, void *, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.start_calls;
    memset(st, 0, sizeof(*st));
    return d.start_rc;
}

int fake_read(void *dev, void *, void *, struct tof_cliff_sample *out, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.read_calls;
    record_caller();
    k_sem_give(&cycle_read_seen);
    d.lock_held_in_read = lock_is_held();
    if (d.read_delay_ms != 0)
        k_msleep(d.read_delay_ms);   // a sensor that blocks: what a join timeout has to cover
    memset(st, 0, sizeof(*st));
    memset(out, 0, sizeof(*out));
    if (d.fresh) {
        out->fresh = true;
        out->target_count = 1;
        out->entry_count = 1;
        out->entries[0].range_mm = d.mm;
        out->entries[0].range_status = d.status_code;
        st->sample_present = true;
    }
    st->rearm_failed = d.rearm_failed;
    if (d.read_rc != 0)
        st->stage = TOF_CLIFF_STAGE_FETCH;
    return d.read_rc;
}

int fake_stop(void *dev, acq::op_status *st)
{
    memset(st, 0, sizeof(*st));
    record_caller();
    ++static_cast<fake_dev *>(dev)->stop_calls;
    return 0;
}

const acq::source_ops kFakeOps{fake_open, fake_configure, fake_start, fake_read, fake_stop};

// Recorded sink activity.
struct {
    int cycles{0};
    acq::cycle_facts last{};
    int cliff_samples{0};
    int last_sample_index{-1};
    int16_t last_sample_mm{0};
    int health_beats{0};
    int cycle_begins{0};
    uint32_t last_begin_cycle{0};
    /* The FIRST cycle number seen since this was armed. "The first cycle of a new epoch carries 0" is
     * about the first one, and the last one of a multi-cycle run is a different number -- which is
     * how the first version of that assertion managed to fail against correct behaviour. */
    uint32_t first_begin_cycle{0};
    bool saw_begin{false};
    uint32_t last_health_snapshot{0};
    acq::mapping_state last_health_state{acq::mapping_state::not_ready};
} rec;

acq::mapping_state provider_state{acq::mapping_state::not_ready};
uint32_t fake_clock_ms{0};

void on_cycle(const acq::cycle_facts &facts)
{
    ++rec.cycles;
    rec.last = facts;
}

uint32_t sample_cycles[8];
int sample_cycle_count;

void on_cliff_sample(int index, uint32_t cycle_seq, const acq::source_facts &,
                     const struct tof_cliff_sample &s)
{
    ++rec.cliff_samples;
    rec.last_sample_index = index;
    rec.last_sample_mm = s.entries[0].range_mm;
    if (sample_cycle_count < static_cast<int>(sizeof sample_cycles / sizeof sample_cycles[0]))
        sample_cycles[sample_cycle_count++] = cycle_seq;
}

void on_cycle_begin(uint32_t cycle_seq)
{
    ++rec.cycle_begins;
    rec.last_begin_cycle = cycle_seq;
    if (!rec.saw_begin) {
        rec.saw_begin = true;
        rec.first_begin_cycle = cycle_seq;
    }
}

void on_cliff_health(uint32_t snapshot, acq::mapping_state state)
{
    ++rec.health_beats;
    rec.last_health_snapshot = snapshot;
    rec.last_health_state = state;
}

acq::mapping_state mapping_provider()
{
    return provider_state;
}

uint32_t clock_ms()
{
    return fake_clock_ms;
}

acq::source_desc four_cliff_two_grid[acq::kMaxSources];

acq::config make_config(int count)
{
    acq::config c{};

    for (int i{0}; i < acq::kMaxSources; ++i) {
        auto &d{four_cliff_two_grid[i]};

        d.kind = (i < 4) ? acq::model::l4_cliff : acq::model::l7_grid;
        d.addr_7bit = static_cast<uint8_t>(0x30 + i);
        d.role_id = static_cast<uint8_t>(i);
        d.dev = &devs[i];
        d.scratch = &devs[i]; /* the fake ops ignore it; only non-null matters */
        d.stream = &streams[i]; /* per source, never shared - see source_desc */
        d.ops = &kFakeOps;
    }
    c.sources = four_cliff_two_grid;
    c.source_count = count;
    c.periods.cycle_period_ms = kCyclePeriodMs;
    c.periods.health_period_ms = kHealthPeriodMs;
    c.hooks.on_cycle = on_cycle;
    c.hooks.on_cliff_sample = on_cliff_sample;
    c.hooks.on_cliff_health = on_cliff_health;
    c.hooks.on_cycle_begin = on_cycle_begin;
    c.mapping_state_provider = mapping_provider;
    c.now_ms = clock_ms;
    return c;
}

void before(void *)
{
    /* Retire whatever the previous case left running. init() now refuses -EALREADY while the
     * subsystem is live -- because the health work item reads cfg_ from another context -- and
     * several cases call before() themselves inside a loop to re-configure per iteration. */
    acq::teardown();

    acq::stop();
    memset(devs, 0, sizeof(devs));
    k_sem_reset(&cycle_read_seen);
    uld_call_count = 0;
    rec = {};
    sample_cycle_count = 0;
    provider_state = acq::mapping_state::not_ready;
    fake_clock_ms = 1000;
}

}  // namespace

/* Every test re-configures, and a live subsystem now refuses that with -EALREADY -- because the
 * health work item reads cfg_ from another context and overwriting it there is a data race with a
 * function pointer in it. So the suite retires the subsystem after each test, which is what a
 * caller has to do anyway. The old suite got away with having no teardown only because init()
 * silently replaced a live configuration. */
void retire(void *)
{
    acq::teardown();
}

ZTEST_SUITE(tof_acquisition, NULL, NULL, before, retire, NULL);

/* --------------------------------------------------------------- configuration ----- */

ZTEST(tof_acquisition, test_timing_has_no_defaults_and_zero_is_refused)
{
    // A placeholder period frozen in the scheduler would quietly become the
    // specification, and both of these are unresolved symbols in the wire contract.
    acq::config c{make_config(4)};

    c.periods.cycle_period_ms = 0;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(4);
    c.periods.health_period_ms = 0;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(4);
    zassert_equal(acq::init(c), 0);
}

ZTEST(tof_acquisition, test_missing_hooks_and_bad_source_tables_are_refused)
{
    acq::config c{make_config(4)};

    c.hooks.on_cliff_health = nullptr;
    zassert_equal(acq::init(c), -EINVAL, "a missing health sink must not be tolerated");

    c = make_config(4);
    c.hooks.on_cycle = nullptr;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(acq::kMaxSources + 1);
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(0);
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(4);
    four_cliff_two_grid[2].ops = nullptr;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(4);
    four_cliff_two_grid[1].scratch = nullptr;
    zassert_equal(acq::init(c), -EINVAL, "a cliff source needs the shared scratch");
    four_cliff_two_grid[1].scratch = &devs[1];

    four_cliff_two_grid[1].stream = nullptr;
    zassert_equal(acq::init(c), -EINVAL,
                  "a cliff source needs its OWN stream state, or the replay guard never arms");
    four_cliff_two_grid[1].stream = &streams[1];
}

ZTEST(tof_acquisition, test_the_ops_table_has_exactly_five_entries)
{
    // Structural stand-in for "there is no enable operation". An interface that cannot
    // express dropping an enable line is stronger than a test asserting nobody did:
    // dropping an L4's enable returns it to 0x29 and destroys the chain's addressing.
    zassert_equal(sizeof(acq::source_ops), 5 * sizeof(void *),
                  "an operation was added to the device interface - if it is enable, "
                  "the chain's addressing is now reachable from the scheduler");
}

/* ------------------------------------------------------------ publication gate ----- */

ZTEST(tof_acquisition, test_proven_is_unreachable_and_has_no_bypass_flag)
{
    zassert_equal(acq::init(make_config(4)), 0);

    provider_state = acq::mapping_state::proven;

    // A chain that cannot be enumerated across both boards cannot have a proven
    // mapping, so the firmware is not entitled to claim one.
    zassert_true(acq::effective_mapping_state() == acq::mapping_state::not_ready);
    zassert_false(acq::publication_allowed(),
                  "no role measurement may be published on an unproven mapping");

    provider_state = acq::mapping_state::fault;
    zassert_true(acq::effective_mapping_state() == acq::mapping_state::fault);
    zassert_false(acq::publication_allowed());

    // There is no build flag that lifts the clamp: a conditional safety bypass is one
    // careless -D away from shipping, and it would not appear in a diff of the code it
    // disables. Lifting it is an edit in its own commit against fixed hardware.
    provider_state = acq::mapping_state::proven;
    zassert_false(acq::publication_allowed(),
                  "publication became possible - was a bypass reintroduced?");
}

ZTEST(tof_acquisition, test_health_is_sent_from_startup_before_any_sensor_is_open)
{
    zassert_equal(acq::init(make_config(4)), 0);

    // Nothing has been opened yet. A consumer that hears nothing at all cannot tell a
    // booting board from a dead one, so NOT_READY has to be on the wire already.
    k_msleep(kHealthPeriodMs * 3);
    zassert_true(rec.health_beats >= 2, "beats seen: %d", rec.health_beats);
    zassert_true(rec.last_health_state == acq::mapping_state::not_ready);
    zassert_equal(rec.cliff_samples, 0);
}

ZTEST(tof_acquisition, test_health_keeps_beating_while_bring_up_is_stuck)
{
    // This is the case the separation exists for: four sequential inits are blocking
    // device operations, and a part that never answers can stall them. If the heartbeat
    // shared that path, the most important safety signal would vanish exactly when
    // something went wrong.
    zassert_equal(acq::init(make_config(4)), 0);
    devs[0].open_delay_ms = kHealthPeriodMs * 4;

    const int before_beats{rec.health_beats};

    zassert_equal(acq::bring_up(), 0);
    zassert_true(rec.health_beats >= before_beats + 2,
                 "the heartbeat stopped while bring-up was blocked: %d -> %d",
                 before_beats, rec.health_beats);
}

/* ----------------------------------------------------------- neutral facts --------- */

ZTEST(tof_acquisition, test_the_same_failure_produces_the_same_facts_for_both_models)
{
    // Hanging and cliff fail in opposite directions, so this layer must not encode
    // either. Identical outcomes must therefore look identical here.
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (auto &d : devs) {
        d.start_rc = 0;
    }
    zassert_equal(acq::bring_up(), 0);
    for (auto &d : devs) {
        d.read_rc = -EIO;
    }
    acq::run_cycle();

    const acq::source_facts &cliff{rec.last.sources[0]};
    const acq::source_facts &grid{rec.last.sources[4]};

    zassert_true(cliff.kind == acq::model::l4_cliff);
    zassert_true(grid.kind == acq::model::l7_grid);
    zassert_equal(cliff.transport_error, grid.transport_error);
    zassert_equal(cliff.protocol_error, grid.protocol_error);
    zassert_equal(cliff.sample_produced, grid.sample_produced);
    zassert_true(cliff.transport_error);
    zassert_false(cliff.protocol_error);
}

ZTEST(tof_acquisition, test_the_four_failure_shapes_stay_distinguishable)
{
    // Folding these together would decide health semantics by accident: the L7 stub's
    // -ENOSYS would reach the cliff health frame as a broken sensor, and a bad call of
    // our own would arrive as a bus fault.
    struct {
        int rc;
        bool transport;
        bool protocol;
        bool unsupported;
        bool usage;
    } cases[] = {
        {-EIO, true, false, false, false},
        {-ETIMEDOUT, true, false, false, false},
        {-EPROTO, false, true, false, false},
        {-ENOSYS, false, false, true, false},
        {-EINVAL, false, false, false, true},
    };

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        before(nullptr);
        zassert_equal(acq::init(make_config(1)), 0);
        zassert_equal(acq::bring_up(), 0);
        devs[0].read_rc = cases[i].rc;
        acq::run_cycle();

        const acq::source_facts &f{rec.last.sources[0]};

        zassert_equal(f.transport_error, cases[i].transport, "rc %d", cases[i].rc);
        zassert_equal(f.protocol_error, cases[i].protocol, "rc %d", cases[i].rc);
        zassert_equal(f.unsupported, cases[i].unsupported, "rc %d", cases[i].rc);
        zassert_equal(f.usage_error, cases[i].usage, "rc %d", cases[i].rc);
    }
}

ZTEST(tof_acquisition, test_a_stubbed_model_is_not_reported_as_a_sensor_fault)
{
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    zassert_equal(acq::bring_up(), 0);
    for (auto &d : devs) {
        d.read_rc = -ENOSYS;
    }
    acq::run_cycle();

    const int error_shift{2 + acq::kMaxSources};

    // The fault bit feeds the cliff health frame. A model with no implementation is a
    // static property of this build, not something that happened to a sensor.
    for (int i{0}; i < acq::kMaxSources; ++i) {
        zassert_true(rec.last.sources[i].unsupported);
        zassert_false(rec.last.sources[i].transport_error);
        zassert_equal(acq::snapshot() & (1U << (error_shift + i)), 0U,
                      "source %d looks faulty because its driver is a stub", i);
    }
}

ZTEST(tof_acquisition, test_every_outcome_clears_when_the_next_cycle_succeeds)
{
    // The bug this pins: the clearing used to enumerate the fields by hand, so the two
    // outcomes added later were cleared nowhere and a single bad cycle left usage_error -
    // and its fault bit - set for the life of the board.
    static const int transient[] = {-EINVAL, -ENOSYS, -EPROTO, -EIO};
    const int error_shift{2 + acq::kMaxSources};

    for (size_t i = 0; i < ARRAY_SIZE(transient); i++) {
        before(nullptr);
        zassert_equal(acq::init(make_config(1)), 0);
        zassert_equal(acq::bring_up(), 0);

        devs[0].read_rc = transient[i];
        acq::run_cycle();
        const acq::source_facts &bad{rec.last.sources[0]};

        zassert_true(bad.transport_error || bad.protocol_error || bad.unsupported ||
                         bad.usage_error,
                     "rc %d recorded nothing", transient[i]);

        devs[0].read_rc = 0;
        devs[0].fresh = true;
        acq::run_cycle();
        const acq::source_facts &good{rec.last.sources[0]};

        zassert_false(good.transport_error, "rc %d left transport set", transient[i]);
        zassert_false(good.protocol_error, "rc %d left protocol set", transient[i]);
        zassert_false(good.unsupported, "rc %d left unsupported set", transient[i]);
        zassert_false(good.usage_error, "rc %d left usage set", transient[i]);
        zassert_false(good.rearm_failed);
        zassert_true(good.sample_produced);
        zassert_equal(good.status.stage, TOF_CLIFF_STAGE_NONE,
                      "rc %d left a stale stage behind", transient[i]);
        zassert_equal(acq::snapshot() & (1U << error_shift), 0U,
                      "rc %d left the fault bit set after a good cycle", transient[i]);
    }
}

ZTEST(tof_acquisition, test_a_bring_up_failure_survives_the_cycles_that_follow)
{
    // The reason a source never started is durable, not per-cycle, and it is the only
    // record of why. Clearing it at the top of every cycle - which the first version did -
    // left nothing but started == false to go on.
    zassert_equal(acq::init(make_config(2)), 0);
    devs[0].open_rc = -EIO;
    zassert_equal(acq::bring_up(), 0);

    devs[1].fresh = true;
    acq::run_cycle();
    acq::run_cycle();

    acq::cycle_facts f{};

    acq::copy_facts(f);
    zassert_false(f.sources[0].started);
    zassert_true(f.sources[0].transport_error, "the bring-up reason was erased");
    zassert_equal(f.sources[0].status.stage, TOF_CLIFF_STAGE_BOOT);
    zassert_not_equal(acq::snapshot() & (1U << (2 + acq::kMaxSources)), 0U,
                      "a source that failed to come up must stay faulted");
    zassert_true(f.sources[1].sample_produced);
}

/* ---------------------------------------------------------------- C3: the re-arm sticky ----
 *
 * A re-arm failure means "this sample arrived but the next one will not". The danger is what the
 * NEXT cycle looks like if that fact is forgotten: the device is stopped, so nothing is ready,
 * read_once returns 0, record() sets nothing, and the published word is configured=1 started=1
 * fault=0 produced=0 -- bit-for-bit identical to a healthy sensor that simply had nothing this
 * cycle. A sensor that will never range again would be indistinguishable from a quiet one on the
 * only signal that is guaranteed to be sent. So the flag is sticky, one-way, and cleared by
 * exactly one thing: a complete successful re-bring-up.
 *
 * The error bit for source i is at (kErrorShift + i), and kErrorShift is kStateBits + kMaxSources.
 */
constexpr uint32_t error_bit(int i)
{
    return 1U << (2 + acq::kMaxSources + i);
}

ZTEST(tof_acquisition, test_a_rearm_failure_keeps_the_sample_and_faults_the_source)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].fresh = true;
    devs[0].rearm_failed = true;
    devs[0].read_rc = -EIO; /* read_once reports the re-arm's error, sample_present intact */
    acq::run_cycle();

    acq::cycle_facts f{};
    acq::copy_facts(f);
    zassert_true(f.sources[0].sample_produced, "this cycle's reading is real and must survive");
    zassert_true(f.sources[0].rearm_failed);
    zassert_not_equal(acq::snapshot() & error_bit(0), 0U,
                      "a sensor that will not range again is a faulted sensor");
}

ZTEST(tof_acquisition, test_an_ordinary_later_cycle_does_not_clear_the_rearm_sticky)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);
    devs[0].fresh = true;
    devs[0].rearm_failed = true;
    devs[0].read_rc = -EIO;
    acq::run_cycle();

    /* A perfectly ordinary cycle: nothing ready, no error. This is the exact shape that used to
     * erase the sticky and make the wedged sensor look quiet. */
    devs[0].fresh = false;
    devs[0].rearm_failed = false;
    devs[0].read_rc = 0;
    acq::run_cycle();
    acq::run_cycle();

    acq::cycle_facts f{};
    acq::copy_facts(f);
    zassert_true(f.sources[0].rearm_failed, "a quiet cycle is not a recovery");
    zassert_not_equal(acq::snapshot() & error_bit(0), 0U);
}

ZTEST(tof_acquisition, test_an_ordinary_later_error_does_not_clear_the_rearm_sticky)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);
    devs[0].fresh = true;
    devs[0].rearm_failed = true;
    devs[0].read_rc = -EIO;
    acq::run_cycle();

    /* record() used to assign st.rearm_failed rather than OR it, so the next ordinary error
     * cleared the sticky it had just set -- the same squashing, one call later. */
    devs[0].fresh = false;
    devs[0].rearm_failed = false;
    devs[0].read_rc = -EIO;
    acq::run_cycle();

    acq::cycle_facts f{};
    acq::copy_facts(f);
    zassert_true(f.sources[0].transport_error);
    zassert_true(f.sources[0].rearm_failed, "one error must not erase a different fact");
    zassert_not_equal(acq::snapshot() & error_bit(0), 0U);
}

ZTEST(tof_acquisition, test_a_recovery_that_fails_at_any_stage_keeps_the_rearm_sticky)
{
    const int stage_rc[3] = {0, 1, 2}; /* which of open / configure / start fails */

    for (int which : stage_rc) {
        zassert_equal(acq::init(make_config(1)), 0);
        devs[0] = fake_dev{};
        zassert_equal(acq::bring_up(), 0);
        devs[0].fresh = true;
        devs[0].rearm_failed = true;
        devs[0].read_rc = -EIO;
        acq::run_cycle();

        devs[0].fresh = false;
        devs[0].rearm_failed = false;
        devs[0].read_rc = 0;
        devs[0].open_rc = (which == 0) ? -EIO : 0;
        devs[0].configure_rc = (which == 1) ? -EIO : 0;
        devs[0].start_rc = (which == 2) ? -EIO : 0;
        zassert_equal(acq::bring_up(), 0);

        acq::cycle_facts f{};
        acq::copy_facts(f);
        zassert_true(f.sources[0].rearm_failed,
                     "a recovery that failed at stage %d is not a recovery", which);
        zassert_false(f.sources[0].started);
        zassert_not_equal(acq::snapshot() & error_bit(0), 0U);

        /* init() is refused while the subsystem is live, and teardown() is the documented
         * way back -- see test_a_second_init_is_refused_while_the_subsystem_is_live. */
        acq::teardown();
    }
}

ZTEST(tof_acquisition, test_only_a_complete_re_bring_up_clears_the_rearm_sticky)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);
    devs[0].fresh = true;
    devs[0].rearm_failed = true;
    devs[0].read_rc = -EIO;
    acq::run_cycle();

    devs[0] = fake_dev{}; /* a healthy device again: open, configure and start all succeed */
    zassert_equal(acq::bring_up(), 0);

    acq::cycle_facts f{};
    acq::copy_facts(f);
    zassert_true(f.sources[0].started);
    zassert_false(f.sources[0].rearm_failed, "a full successful bring-up is the recovery");
    zassert_equal(acq::snapshot() & error_bit(0), 0U);
}

ZTEST(tof_acquisition, test_the_rearm_sticky_faults_only_its_own_source)
{
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::bring_up(), 0);

    for (int i = 0; i < 4; ++i)
        devs[i].fresh = true;
    devs[2].rearm_failed = true;
    devs[2].read_rc = -EIO;
    acq::run_cycle();

    acq::cycle_facts f{};
    acq::copy_facts(f);
    for (int i = 0; i < 4; ++i) {
        if (i == 2) {
            zassert_true(f.sources[i].rearm_failed);
            zassert_not_equal(acq::snapshot() & error_bit(i), 0U);
        } else {
            zassert_false(f.sources[i].rearm_failed, "source %d was not the one that failed", i);
            zassert_equal(acq::snapshot() & error_bit(i), 0U, "source %d must stay clean", i);
        }
    }
}

ZTEST(tof_acquisition, test_a_usage_error_does_set_the_fault_bit)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);
    devs[0].read_rc = -EINVAL;
    acq::run_cycle();

    // Our own defect must be loud, unlike a stub.
    zassert_true(rec.last.sources[0].usage_error);
    zassert_not_equal(acq::snapshot() & (1U << (2 + acq::kMaxSources)), 0U);
}

ZTEST(tof_acquisition, test_a_missing_sample_is_recorded_and_not_interpreted)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].fresh = false;  // ready check said nothing, which is not an error
    devs[0].read_rc = 0;
    acq::run_cycle();

    const acq::source_facts &f{rec.last.sources[0]};

    zassert_false(f.sample_produced);
    zassert_false(f.transport_error, "silence is not a transport failure");
    zassert_false(f.protocol_error);
    zassert_equal(rec.cliff_samples, 0, "no payload may be invented for a silent source");
}

ZTEST(tof_acquisition, test_rearm_failure_is_carried_as_its_own_fact)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].fresh = true;
    devs[0].mm = 321;
    devs[0].rearm_failed = true;
    devs[0].read_rc = -EIO;
    acq::run_cycle();

    // The sample happened and the next one will not: two facts, not one verdict.
    zassert_true(rec.last.sources[0].rearm_failed);
    zassert_true(rec.last.sources[0].sample_produced);
    zassert_equal(rec.cliff_samples, 1);
    zassert_equal(rec.last_sample_mm, 321);
}

ZTEST(tof_acquisition, test_cycle_seq_advances_once_per_cycle)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);

    // The contract correlates a measurement frame with a health frame through this
    // number, so exactly one increment per cycle is the property that matters.
    const uint32_t first{[]() {
        acq::run_cycle();
        return rec.last.cycle_seq;
    }()};

    acq::run_cycle();
    zassert_equal(rec.last.cycle_seq, first + 1);
    acq::run_cycle();
    zassert_equal(rec.last.cycle_seq, first + 2);
    zassert_equal(rec.cycles, 3);
}

ZTEST(tof_acquisition, test_the_cycle_records_the_injected_clock)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    fake_clock_ms = 4242;
    acq::run_cycle();
    zassert_equal(rec.last.began_ms, 4242);
}

/* ------------------------------------------------------- isolation and locking ----- */

ZTEST(tof_acquisition, test_one_dead_source_does_not_stop_the_others)
{
    zassert_equal(acq::init(make_config(4)), 0);
    devs[0].open_rc = -EIO;
    devs[2].start_rc = -EIO;

    zassert_equal(acq::bring_up(), 0, "bring-up must not abort on one bad sensor");

    // Bring-up runs no cycle, so its outcome is read from the facts rather than from a
    // cycle callback - which is exactly why copy_facts exists.
    acq::cycle_facts after_bring_up{};

    acq::copy_facts(after_bring_up);
    zassert_false(after_bring_up.sources[0].started);
    zassert_true(after_bring_up.sources[1].started);
    zassert_false(after_bring_up.sources[2].started);
    zassert_true(after_bring_up.sources[3].started);
    zassert_true(after_bring_up.sources[0].transport_error);
    zassert_equal(after_bring_up.sources[0].status.stage, TOF_CLIFF_STAGE_BOOT,
                  "the failing stage must survive into the facts");

    // A source that never started is not read, and the started ones are.
    for (int i{0}; i < 4; ++i) {
        devs[i].fresh = true;
        devs[i].mm = static_cast<int16_t>(100 + i);
    }
    acq::run_cycle();
    zassert_equal(devs[0].read_calls, 0);
    zassert_equal(devs[1].read_calls, 1);
    zassert_equal(devs[2].read_calls, 0);
    zassert_equal(devs[3].read_calls, 1);
    zassert_equal(rec.cliff_samples, 2);
}

ZTEST(tof_acquisition, test_bring_up_outcomes_are_readable_without_a_cycle)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);

    acq::cycle_facts f{};

    acq::copy_facts(f);
    zassert_equal(f.source_count, 2);
    zassert_true(f.sources[0].configured);
    zassert_true(f.sources[0].started);
    zassert_equal(f.cycle_seq, 0, "bring-up is not a cycle and must not advance the seq");
    zassert_equal(rec.cycles, 0);
}

ZTEST(tof_acquisition, test_every_device_operation_runs_under_the_chain_lock)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);
    devs[0].fresh = true;
    acq::run_cycle();

    // The chain control lines and the bus are one shared resource, and commissioning
    // takes the same lock for its whole session.
    zassert_true(devs[0].lock_held_in_open);
    zassert_true(devs[1].lock_held_in_open);
    zassert_true(devs[0].lock_held_in_read);
    zassert_true(devs[1].lock_held_in_read);
    zassert_false(lock_is_held(), "the lock must be released between cycles");
}

ZTEST(tof_acquisition, test_the_gap_between_cycles_is_not_an_idle_chain)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();

    // The scheduler will take the lock again within one period. Commissioning cannot
    // enumerate in that gap without re-addressing parts underneath the next read, so a
    // short window must not read as a safe one.
    zassert_false(acq::is_idle(), "a running scheduler between cycles is not idle");

    acq::stop();
    zassert_true(acq::is_idle());
}

ZTEST(tof_acquisition, test_stop_quiesces_and_leaves_the_chain_free_for_commissioning)
{
    zassert_equal(acq::init(make_config(3)), 0);
    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();

    acq::stop();

    // Enumeration drops enable lines, which re-addresses parts underneath a reader, so
    // it may only run once acquisition is idle.
    zassert_true(acq::is_idle());
    zassert_false(lock_is_held());
    zassert_equal(devs[0].stop_calls, 1);
    zassert_equal(devs[2].stop_calls, 1);

    const int cycles{rec.cycles};

    acq::run_cycle();
    zassert_equal(rec.cycles, cycles, "a stopped scheduler must not run a cycle");
}

/* ------------------------------------------------------------------ L7 stub -------- */

ZTEST(tof_acquisition, test_the_grid_ops_are_an_explicit_stub)
{
    const acq::source_ops &ops{acq::l7_stub_ops()};
    acq::op_status st{};
    struct tof_cliff_sample sample{};

    // A named stub rather than a null pointer or a copy of the cliff ops, so wiring L7
    // to the wrong driver has to be deliberate. Note what the refusal of
    // read_cliff_sample actually says: the shared interface is still the shape of a point
    // sensor, and an 8x8 zone frame cannot travel through it. That is prototype debt, and
    // this assertion is where it is visible.
    zassert_equal(ops.open(nullptr, 0x30, &st), -ENOSYS);
    zassert_equal(ops.configure(nullptr, &st), -ENOSYS);
    zassert_equal(ops.start(nullptr, nullptr, &st), -ENOSYS);
    zassert_equal(ops.read_cliff_sample(nullptr, nullptr, nullptr, &sample, &st), -ENOSYS);
    zassert_equal(ops.stop(nullptr, &st), -ENOSYS);
    zassert_false(sample.fresh);

    zassert_not_equal(&acq::l7_stub_ops(), &acq::l4_cliff_ops());
}

ZTEST(tof_acquisition, test_a_grid_source_never_produces_a_cliff_payload)
{
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    zassert_equal(acq::bring_up(), 0);

    // Even if the grid path did return a sample, it must not travel down the cliff
    // sink: the two packers must never have to step over each other's data.
    for (auto &d : devs) {
        d.fresh = true;
        d.mm = 500;
    }
    acq::run_cycle();

    zassert_equal(rec.cliff_samples, 4, "only the four cliff sources may reach that sink");
    zassert_true(rec.last_sample_index < 4);
}

/* ----------------------------------------------------------------- snapshot -------- */

ZTEST(tof_acquisition, test_the_snapshot_carries_what_the_heartbeat_needs)
{
    zassert_equal(acq::init(make_config(4)), 0);
    devs[1].open_rc = -EIO;
    zassert_equal(acq::bring_up(), 0);

    devs[0].fresh = true;
    devs[2].read_rc = -EIO;
    acq::run_cycle();

    const uint32_t snap{acq::snapshot()};
    const int produced_shift{2};
    const int error_shift{produced_shift + acq::kMaxSources};
    const int configured_shift{error_shift + acq::kMaxSources};
    const int started_shift{configured_shift + acq::kMaxSources};

    zassert_equal(snap & 0x3U, static_cast<uint32_t>(acq::mapping_state::not_ready));
    zassert_not_equal(snap & (1U << (produced_shift + 0)), 0U);
    zassert_equal(snap & (1U << (produced_shift + 2)), 0U);
    zassert_not_equal(snap & (1U << (error_shift + 2)), 0U);
    zassert_not_equal(snap & (1U << (configured_shift + 1)), 0U);
    zassert_equal(snap & (1U << (started_shift + 1)), 0U, "source 1 never started");
    zassert_not_equal(snap & (1U << (started_shift + 0)), 0U);

    // The heartbeat reports the same word, and it does so without the lock.
    k_msleep(kHealthPeriodMs * 2);
    zassert_equal(rec.last_health_snapshot, snap);
}

/* ------------------------------------------------------ cycle_seq, per contract --- */

ZTEST(tof_acquisition, test_the_first_cycle_of_an_epoch_is_zero)
{
    /* The contract is specific: cycle_seq "starts at 0 for the first cycle of a new
     * mapping_epoch, increments by one per COMPLETED cycle and wraps 255 -> 0". A
     * pre-increment at the top of the cycle gives 1 for the first one, and that violation
     * once made it as far as being asserted as expected behaviour in another test. */
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    sample_cycle_count = 0;
    acq::run_cycle();
    zassert_true(sample_cycle_count > 0, "no sample to read a cycle number from");
    zassert_equal(sample_cycles[0], 0, "the first cycle of an epoch must be 0");

    acq::cycle_facts f{};
    acq::copy_facts(f);
    zassert_equal(f.cycle_seq, 0);

    sample_cycle_count = 0;
    acq::run_cycle();
    zassert_equal(sample_cycles[0], 1, "the second cycle must be 1");
    acq::copy_facts(f);
    zassert_equal(f.cycle_seq, 1);

    acq::stop();
}

ZTEST(tof_acquisition, test_the_same_epoch_never_reuses_a_cycle_number)
{
    /* The contract allows at most one measurement frame per
     * (source_id, mapping_epoch, cycle_seq), and calls a second one a conflict rather than
     * a retransmission to tolerate. This layer does not own the epoch -- it is injected on
     * the publishing side and nothing here advances it -- so restarting the cycle count on
     * a second bring-up would reissue triples already used. An earlier version did exactly
     * that, and a test asserted it as correct.
     *
     * Advancing the epoch and restarting the count are two halves of one operation that
     * belongs to whatever owns the mapping. begin_epoch() is now that half, and a bring-up on
     * its own still must not renumber -- which is what this test holds. */
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    sample_cycle_count = 0;
    acq::run_cycle();
    acq::run_cycle();
    zassert_true(sample_cycle_count >= 2);
    const uint32_t before_second_bring_up{sample_cycles[sample_cycle_count - 1]};

    zassert_equal(acq::bring_up(), 0);
    sample_cycle_count = 0;
    acq::run_cycle();

    zassert_true(sample_cycles[0] > before_second_bring_up,
                 "a second bring-up reused a cycle number under the same epoch");

    acq::stop();
}

ZTEST(tof_acquisition, test_the_wire_byte_wraps_255_to_zero)
{
    /* The wire field is a uint8. The counter is kept wider so a pending frame can be
     * matched to its own cycle without aliasing every 256th one, and the wrap the contract
     * requires happens on truncation. */
    zassert_equal(acq::init(make_config(1)), 0);
    devs[0].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    for (int i = 0; i < 256; ++i)
        acq::run_cycle();

    sample_cycle_count = 0;
    acq::run_cycle();
    zassert_equal(sample_cycles[0], 256, "the internal counter does not wrap");
    zassert_equal(static_cast<uint8_t>(sample_cycles[0] & 0xFF), 0,
                  "the wire byte must wrap 255 -> 0");

    acq::stop();
}

ZTEST(tof_acquisition, test_begin_epoch_restarts_the_numbering_when_acquisition_is_idle)
{
    /* The other half of the epoch transaction, and the real one -- the authority's own tests
     * inject a fake begin_epoch so they can make it fail, which proves nothing about this. */
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    sample_cycle_count = 0;
    acq::run_cycle();
    acq::run_cycle();
    zassert_true(sample_cycle_count >= 2);
    zassert_true(sample_cycles[sample_cycle_count - 1] > 0);

    acq::stop();
    zassert_equal(acq::begin_epoch(), 0, "idle is exactly when this is legal");

    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);
    sample_cycle_count = 0;
    acq::run_cycle();
    zassert_equal(sample_cycles[0], 0, "the first cycle of the new epoch must be 0 again");

    acq::stop();
}

ZTEST(tof_acquisition, test_begin_epoch_refuses_while_cycles_are_being_produced)
{
    /* A reset mid-flight renumbers a sequence the consumer is half-way through assembling,
     * and the uniqueness the contract guarantees is over the triple rather than the cycle
     * alone. Refusing is the whole reason this returns an errno instead of void. */
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    sample_cycle_count = 0;
    acq::run_cycle();
    const uint32_t before{sample_cycles[0]};

    zassert_equal(acq::begin_epoch(), -EBUSY);

    sample_cycle_count = 0;
    acq::run_cycle();
    zassert_true(sample_cycles[0] > before, "a refused begin_epoch must not have reset anything");

    acq::stop();
}

ZTEST(tof_acquisition, test_stopping_acquisition_does_not_stop_the_heartbeat)
{
    /* stop() means "stop reading sensors". The contract requires health to keep flowing while no
     * acquisition runs, which is exactly when a consumer needs to know the subsystem is alive
     * and why it is idle -- a commissioning pause must not look like a crashed producer, or the
     * consumer's own timeout raises a fault for a machine behaving as asked. */
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::bring_up(), 0);
    acq::stop();

    const int before{rec.health_beats};
    k_msleep(kHealthPeriodMs * 3);
    zassert_true(rec.health_beats >= before + 2,
                 "the heartbeat stopped with acquisition: %d -> %d", before, rec.health_beats);
}

ZTEST(tof_acquisition, test_teardown_is_what_stops_the_heartbeat)
{
    /* The other half. Retiring the subsystem is a separate, deliberate act from pausing it, and
     * having only one call for both is how the heartbeat got stopped by a pause in the first
     * place. */
    zassert_equal(acq::init(make_config(4)), 0);
    k_msleep(kHealthPeriodMs * 2);
    acq::teardown();

    const int before{rec.health_beats};
    k_msleep(kHealthPeriodMs * 3);
    zassert_equal(rec.health_beats, before, "the heartbeat survived teardown");
}

ZTEST(tof_acquisition, test_every_cycle_is_announced_before_the_first_sensor_is_read)
{
    /* The hook exists so a cycle that produces nothing can still be reported, so it has to fire
     * before any read -- if it fired with the first sample it would never fire for such a cycle,
     * which is the whole hole it closes. */
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    rec.cycle_begins = 0;
    acq::run_cycle();
    zassert_equal(rec.cycle_begins, 1, "exactly one announcement per cycle");
    zassert_equal(rec.last_begin_cycle, 0, "and it carries the cycle about to run");

    acq::run_cycle();
    zassert_equal(rec.cycle_begins, 2);
    zassert_equal(rec.last_begin_cycle, 1);

    acq::stop();
}

ZTEST(tof_acquisition, test_a_cycle_where_no_sensor_produces_is_still_announced)
{
    /* The case the hook was added for. Nothing to sample, so nothing but the announcement and the
     * completion reach the sink -- and that pair is what tells a consumer the cycle happened. */
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    for (int i = 0; i < acq::kMaxSources; ++i)
        devs[i].fresh = false;
    zassert_equal(acq::bring_up(), 0);

    rec.cycle_begins = 0;
    rec.cliff_samples = 0;
    acq::run_cycle();

    zassert_equal(rec.cycle_begins, 1);
    zassert_equal(rec.cliff_samples, 0, "no sensor had anything");
    zassert_equal(rec.cycles, 1, "and the cycle still completed");

    acq::stop();
}

ZTEST(tof_acquisition, test_a_missing_cycle_begin_hook_is_refused)
{
    /* Not optional. A sink that never hears the start of a cycle cannot report an empty one, and
     * a silently missing hook would turn every empty cycle into a gap that reads as a stopped
     * producer. */
    acq::config c{make_config(4)};
    c.hooks.on_cycle_begin = nullptr;
    zassert_equal(acq::init(c), -EINVAL);
}

ZTEST(tof_acquisition, test_a_second_init_is_refused_while_the_subsystem_is_live)
{
    /* cfg_ is read by the health work item from another context, so replacing it under a live
     * subsystem is a data race on a struct full of function pointers. Retiring is teardown()'s
     * job and has to be asked for. */
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::init(make_config(4)), -EALREADY);

    acq::teardown();
    zassert_equal(acq::init(make_config(4)), 0, "after teardown it is configurable again");
}

ZTEST(tof_acquisition, test_no_health_frame_escapes_after_teardown_returns)
{
    /* Stopping the timer is not enough: the last tick may already have submitted a work item that
     * is queued or running, so a frame could go out after teardown claimed the subsystem was
     * down. teardown() cancels it synchronously, which is what makes "no more frames" true at the
     * moment it returns rather than shortly after. */
    zassert_equal(acq::init(make_config(4)), 0);
    k_msleep(kHealthPeriodMs * 2);
    zassert_true(rec.health_beats > 0, "the heartbeat has to be running for this to prove anything");

    acq::teardown();
    const int after_teardown{rec.health_beats};
    k_msleep(kHealthPeriodMs * 4);
    zassert_equal(rec.health_beats, after_teardown, "a health frame escaped after teardown");
}

ZTEST(tof_acquisition, test_teardown_retires_the_subsystem_rather_than_pausing_it)
{
    /* The path commissioning walks on every attempt. teardown() used to leave the configuration
     * in place, so bring_up() and begin_epoch() still accepted it -- and restarted acquisition
     * with the heartbeat already stopped. A producer emitting measurements with no liveness
     * channel is the one combination the consumer cannot reason about. */
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::bring_up(), 0);
    acq::teardown();

    zassert_equal(acq::bring_up(), -EINVAL, "a retired subsystem restarted acquisition");
    zassert_equal(acq::begin_epoch(), -EINVAL, "a retired subsystem accepted an epoch");

    /* And a fresh configuration is the only way back -- which also restarts the heartbeat. */
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::bring_up(), 0);
    const int before{rec.health_beats};
    k_msleep(kHealthPeriodMs * 3);
    zassert_true(rec.health_beats >= before + 2);
}

/* A commissioning run holds the chain for two full enumerations plus an isolation -- seconds, on real
 * hardware. The heartbeat has to keep going for all of it, because it is the only channel telling a
 * consumer that the subsystem is alive and that its mapping is being re-proven. The health work item
 * takes no lock and reads one atomic word, so this holds by construction; "by construction" is what
 * gets verified, not asserted. */
K_THREAD_STACK_DEFINE(cycle_runner_stack, 2048);
static k_thread cycle_runner;
K_SEM_DEFINE(cycle_thread_started, 0, 1);

static void cycle_runner_entry(void *, void *, void *)
{
    /* Signals BEFORE the call, and the PRIORITY is what makes that sound.
     *
     * The interleaving being staged is a cycle that is already waiting for the chain when the
     * quiesce runs -- so this thread has to get as far as the blocking acquire before the test
     * thread continues. Created one band ABOVE the test thread for exactly that: giving the
     * semaphore does not yield, so this thread runs on until it blocks on the chain.
     *
     * The first version ran one band BELOW the ztest thread (cooperative at -1), so k_sem_give()
     * handed control straight back to the test before this thread had reached run_cycle() at all.
     * back then run_cycle() still had an unlocked pre-check, so the cycle returned there instead,
     * and the case passed with the locked check deleted -- it was staging nothing. The pre-check is
     * gone now (an unsynchronised read of running_ was a data race, not an optimisation), but the
     * priority is still what decides whether anything is staged. */
    k_sem_give(&cycle_thread_started);
    acq::run_cycle();
}

K_THREAD_STACK_DEFINE(chain_holder_stack, 1024);
static k_thread chain_holder;
K_SEM_DEFINE(holder_took_it, 0, 1);
K_SEM_DEFINE(holder_release, 0, 1);

static void chain_holder_entry(void *, void *, void *)
{
    k_mutex_lock(&lexxhard::tof_chain_controller::chain_lock(), K_FOREVER);
    k_sem_give(&holder_took_it);
    (void)k_sem_take(&holder_release, K_FOREVER);
    k_mutex_unlock(&lexxhard::tof_chain_controller::chain_lock());
}

ZTEST(tof_acquisition, test_the_heartbeat_survives_a_long_chain_session)
{
    zassert_equal(acq::init(make_config(4)), 0);
    acq::stop();   // what commissioning does: quiesce, and keep the heartbeat

    k_sem_reset(&holder_took_it);
    k_sem_reset(&holder_release);
    k_thread_create(&chain_holder, chain_holder_stack, K_THREAD_STACK_SIZEOF(chain_holder_stack),
                    chain_holder_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
    zassert_equal(k_sem_take(&holder_took_it, K_MSEC(500)), 0, "the holder never took the chain");

    const int before{rec.health_beats};
    k_msleep(kHealthPeriodMs * 4);
    const int during{rec.health_beats};

    k_sem_give(&holder_release);
    (void)k_thread_join(&chain_holder, K_MSEC(500));

    zassert_true(during >= before + 3,
                 "the heartbeat stalled while the chain was held: %d -> %d", before, during);
}

/* --------------------------------------------------- the commissioning quiesce ------ */

/* try_stop() is the quiesce commissioning actually calls, and the only thing that distinguishes it
 * from stop() is that it will not wait for the chain. That difference is the whole point: the
 * session that follows takes the chain with K_NO_WAIT, so a quiesce that blocks moves the hang one
 * step earlier instead of removing it. Tested with a holder thread, because a K_NO_WAIT acquire
 * from the thread that already owns a Zephyr mutex SUCCEEDS -- the mutex is recursive for its
 * owner, so "busy" cannot be staged from the test thread itself. */

ZTEST(tof_acquisition, test_try_stop_refuses_a_busy_chain_and_changes_nothing)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);

    k_sem_reset(&holder_took_it);
    k_sem_reset(&holder_release);
    k_thread_create(&chain_holder, chain_holder_stack, K_THREAD_STACK_SIZEOF(chain_holder_stack),
                    chain_holder_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
    zassert_equal(k_sem_take(&holder_took_it, K_MSEC(500)), 0, "the holder never took the chain");

    const int stops_before{devs[0].stop_calls};
    zassert_equal(acq::try_stop(), -EBUSY, "a held chain has to be a refusal, not a wait");

    /* Refused means untouched, which is what makes refusing safe: the caller gets to give up
     * without having half-quiesced a subsystem it no longer intends to commission. */
    zassert_equal(devs[0].stop_calls, stops_before, "the refusal still stopped a device");

    k_sem_give(&holder_release);
    zassert_equal(k_thread_join(&chain_holder, K_MSEC(500)), 0);

    /* Still running, so the refusal cost nothing: a cycle runs on demand exactly as before. */
    const int begins_before{rec.cycle_begins};
    acq::run_cycle();
    zassert_equal(rec.cycle_begins, begins_before + 1, "the refusal left acquisition stopped");

    zassert_equal(acq::try_stop(), 0, "a free chain has to be quiesced");
    zassert_true(devs[0].stop_calls > stops_before);
    zassert_true(acq::is_idle());
}

ZTEST(tof_acquisition, test_try_stop_leaves_the_heartbeat_running)
{
    /* The same rule stop() obeys, for the same reason: commissioning takes seconds, and the
     * heartbeat is the only channel that says the subsystem is alive while it does. A quiesce that
     * silenced it would make a controlled pause indistinguishable from a dead producer. */
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);
    zassert_equal(acq::try_stop(), 0);

    const int before{rec.health_beats};
    k_msleep(kHealthPeriodMs * 4);
    zassert_true(rec.health_beats >= before + 3,
                 "try_stop() stopped the heartbeat: %d -> %d", before, rec.health_beats);
}

ZTEST(tof_acquisition, test_a_cycle_that_lost_the_race_to_a_quiesce_does_not_run)
{
    /* The race that arrives with the acquisition thread, staged deterministically.
     *
     * A cycle blocks on the chain this test thread is holding, and while it waits, try_stop()
     * (recursive on the same thread, so it succeeds) sets running_ false. When the cycle finally
     * gets the lock, the quiesce it is about to ignore has already returned success to a
     * commissioner who has been told the chain is safe to enumerate in. Enumeration drops enable
     * lines, so the cycle would be reading parts mid-re-address.
     *
     * The cycle must therefore not begin: no on_cycle_begin, no reads, and no cycle number spent
     * on something that did not happen. */
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);

    const int begins_before{rec.cycle_begins};
    const int reads_before{devs[0].read_calls};

    k_mutex_lock(&lexxhard::tof_chain_controller::chain_lock(), K_FOREVER);

    k_sem_reset(&cycle_thread_started);
    /* One band above whatever the test thread is running at, read rather than hardcoded: the
     * staging depends on this thread NOT yielding when it signals, and a hardcoded number would
     * silently stop staging anything if CONFIG_ZTEST_THREAD_PRIORITY changed. */
    const int racing_prio{k_thread_priority_get(k_current_get()) - 1};
    k_thread_create(&cycle_runner, cycle_runner_stack, K_THREAD_STACK_SIZEOF(cycle_runner_stack),
                    cycle_runner_entry, nullptr, nullptr, nullptr, racing_prio, 0, K_NO_WAIT);
    zassert_equal(k_sem_take(&cycle_thread_started, K_MSEC(500)), 0,
                  "the cycle thread never got as far as the lock");

    zassert_equal(acq::try_stop(), 0, "recursive acquire by the lock owner has to succeed");
    k_mutex_unlock(&lexxhard::tof_chain_controller::chain_lock());

    zassert_equal(k_thread_join(&cycle_runner, K_MSEC(500)), 0);

    zassert_equal(rec.cycle_begins, begins_before,
                  "a cycle began after the quiesce had already reported success");
    zassert_equal(devs[0].read_calls, reads_before, "a device was read after the quiesce");

    /* And the cycle number was not spent. No cycle has completed in this test, so the next real one
     * still owes cycle_seq 0; had the cancelled cycle counted, this would be 1. Checked through the
     * facts rather than the counter, because the counter is private and the wire is what a consumer
     * correlates a measurement with its health frame on. */
    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();
    zassert_equal(rec.last.cycle_seq, 0u, "the cancelled cycle consumed a cycle_seq");
}


/* ----------------------------------------------- the acquisition thread ------------- */

/* The stack is the SUITE's, because there is no devicetree here to size one from and a fallback
 * compiled into the module would be a size nobody chose -- in the product image as well as this one. */
K_THREAD_STACK_DEFINE(acq_thread_stack, 2048);

acq::thread_config thread_cfg(uint32_t join_timeout_ms)
{
    acq::thread_config t{};

    t.stack = acq_thread_stack;
    t.stack_size = K_THREAD_STACK_SIZEOF(acq_thread_stack);
    t.priority = K_PRIO_PREEMPT(5);
    t.join_timeout_ms = join_timeout_ms;
    return t;
}

ZTEST(tof_acquisition, test_the_thread_brings_up_and_then_cycles_at_the_cadence)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0);
    zassert_true(acq::thread_running());

    k_msleep(kCyclePeriodMs * 4);
    const int cycles{rec.cycles};

    zassert_true(cycles >= 2, "cycles in four periods: %d", cycles);
    zassert_equal(devs[0].start_calls, 1, "bring-up did not happen exactly once");
    zassert_equal(acq::try_stop(), 0);
    zassert_false(acq::thread_running());
}

ZTEST(tof_acquisition, test_a_stop_request_ends_the_cycles_and_the_thread_stops_the_devices)
{
    /* The first acceptance boundary: after a stop request nothing enters a new cycle, and the STOP
     * came from the thread rather than from whoever asked. */
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0);

    /* Wait until the thread is provably BETWEEN cycles: a read has happened, and the count has then
     * stopped moving for a fraction of the cadence. Requesting the stop from an unknown point in the
     * cadence is what let an earlier version of this case pass against an implementation that ran one
     * more cycle after the request -- by the time join() returned, cycles had stopped either way. */
    zassert_equal(k_sem_take(&cycle_read_seen, K_MSEC(kCyclePeriodMs * 4)), 0,
                  "the thread never read a sensor");
    k_msleep(5);
    const int begins_before_request{rec.cycle_begins};
    k_msleep(5);
    zassert_equal(rec.cycle_begins, begins_before_request,
                  "the thread was still mid-cadence; the request point is not known");

    acq::request_stop();
    zassert_equal(acq::join(500), 0);

    const int cycles_at_stop{rec.cycles};
    const int begins_at_stop{rec.cycle_begins};

    zassert_equal(begins_at_stop, begins_before_request,
                  "a cycle BEGAN after the stop request (%d -> %d)", begins_before_request,
                  begins_at_stop);

    zassert_equal(devs[0].stop_calls, 1, "the thread did not stop its devices on the way out");
    zassert_equal(devs[1].stop_calls, 1);
    zassert_false(acq::thread_running());

    k_msleep(kCyclePeriodMs * 4);
    zassert_equal(rec.cycles, cycles_at_stop, "a cycle ran after the stop request");
    zassert_equal(rec.cycle_begins, begins_at_stop, "a cycle BEGAN after the stop request");
}

ZTEST(tof_acquisition, test_every_uld_call_comes_from_the_acquisition_thread)
{
    /* The second acceptance boundary. Recorded per call rather than argued from the lock: serialised
     * is not single-owner, and the ULD's one transport record is what cannot survive two callers. */
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0);
    k_msleep(kCyclePeriodMs * 3);

    const k_tid_t owner{acq::thread_id_for_test()};

    zassert_not_null(owner);
    zassert_true(every_uld_call_came_from(owner),
                 "%d ULD calls recorded, not all from the acquisition thread", uld_call_count);

    /* And a foreign attempt is refused rather than merely discouraged. This test thread is not the
     * owner, so all three lifecycle calls must do nothing and be counted. */
    const int cycles_before{rec.cycles};
    const int stops_before{devs[0].stop_calls};

    zassert_equal(acq::bring_up(), -EPERM, "a foreign bring_up() was allowed");
    acq::run_cycle();
    acq::stop();
    zassert_true(acq::foreign_lifecycle_calls() >= 2, "foreign calls went uncounted: %u",
                 acq::foreign_lifecycle_calls());
    zassert_equal(devs[0].stop_calls, stops_before, "a foreign stop() stopped a device");
    zassert_true(acq::thread_running(), "a foreign stop() ended the thread");

    zassert_equal(acq::try_stop(), 0);
    zassert_true(rec.cycles >= cycles_before);
}

ZTEST(tof_acquisition, test_a_join_that_times_out_changes_nothing_and_kills_nothing)
{
    /* The third acceptance boundary: the timeout path must leave the subsystem exactly as it was, so
     * that a caller which cannot get a clean stop can only refuse. Nothing is aborted -- a thread
     * inside a vendor driver holds the chain lock and a half-finished transfer, and killing it would
     * leave both. */
    zassert_equal(acq::init(make_config(2)), 0);
    devs[0].read_delay_ms = kCyclePeriodMs * 8;
    zassert_equal(acq::start(thread_cfg(kCyclePeriodMs)), 0);
    k_msleep(kCyclePeriodMs);   // let it get inside the blocking read

    const int64_t started{k_uptime_get()};
    const int rc{acq::try_stop()};
    const int64_t elapsed{k_uptime_get() - started};

    zassert_equal(rc, -EBUSY, "a thread stuck in a read reported a clean stop");
    zassert_true(elapsed < kCyclePeriodMs * 6, "the bounded join was not bounded (%lld ms)", elapsed);
    zassert_true(acq::thread_running(), "the timeout killed the thread");
    zassert_equal(devs[0].stop_calls, 0, "the timeout path stopped devices anyway");

    /* It does finish eventually -- the stop request is still set, so this is a clean exit, not a
     * rescue. */
    devs[0].read_delay_ms = 0;
    zassert_equal(acq::join(kCyclePeriodMs * 20), 0);
    zassert_equal(devs[0].stop_calls, 1);
}

ZTEST(tof_acquisition, test_starting_the_thread_does_not_renumber_the_cycles)
{
    /* cycle_seq belongs to the mapping epoch, not to the thread. begin_epoch() resets it as one step
     * of the authority's commit; a reset on start would either renumber a sequence a consumer is
     * part-way through or reissue a triple that has already been used. */
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::begin_epoch(), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0);
    k_msleep(kCyclePeriodMs * 3);
    zassert_equal(acq::try_stop(), 0);

    const uint32_t last{rec.last.cycle_seq};

    zassert_true(rec.cycles >= 2);
    zassert_equal(rec.last_begin_cycle, last);

    zassert_equal(acq::start(thread_cfg(500)), 0);
    k_msleep(kCyclePeriodMs * 2);
    zassert_equal(acq::try_stop(), 0);
    zassert_true(rec.last.cycle_seq > last, "restarting the thread renumbered the cycles: %u -> %u",
                 last, rec.last.cycle_seq);

    /* And a new epoch does start at 0, which is the other half of the same rule. Armed first, and
     * checked against the FIRST cycle of the run: the last one of several is a later number. */
    rec.saw_begin = false;
    zassert_equal(acq::begin_epoch(), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0);
    k_msleep(kCyclePeriodMs * 2);
    zassert_equal(acq::try_stop(), 0);
    zassert_true(rec.saw_begin);
    zassert_equal(rec.first_begin_cycle, 0u, "the first cycle of a new epoch was not 0");
}

ZTEST(tof_acquisition, test_the_thread_configuration_has_no_defaults_either)
{
    zassert_equal(acq::init(make_config(1)), 0);

    acq::thread_config no_stack{thread_cfg(500)};
    no_stack.stack = nullptr;
    zassert_equal(acq::start(no_stack), -EINVAL);

    acq::thread_config no_size{thread_cfg(500)};
    no_size.stack_size = 0;
    zassert_equal(acq::start(no_size), -EINVAL);

    /* The join timeout has no default because it decides how long commissioning waits before it
     * refuses -- a deployment decision, and one nobody would find if it were invented here. */
    zassert_equal(acq::start(thread_cfg(0)), -EINVAL);

    zassert_equal(acq::start(thread_cfg(500)), 0);
    zassert_equal(acq::start(thread_cfg(500)), -EALREADY, "a second thread was created");
    zassert_equal(acq::try_stop(), 0);
}

ZTEST(tof_acquisition, test_a_thread_that_exited_but_was_not_joined_cannot_be_restarted)
{
    /* k_thread_create() over a kernel object whose previous thread has not been joined reuses a live
     * structure. -EALREADY is also the honest answer to the caller: it has not finished stopping. */
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0);
    acq::request_stop();
    k_msleep(kCyclePeriodMs * 3);   // it has exited by now, but nobody has joined it

    zassert_false(acq::thread_running());
    zassert_equal(acq::start(thread_cfg(500)), -EALREADY);
    zassert_equal(acq::join(500), 0);
    zassert_equal(acq::start(thread_cfg(500)), 0, "a joined thread must be restartable");
    zassert_equal(acq::try_stop(), 0);
}
