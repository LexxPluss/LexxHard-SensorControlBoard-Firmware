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
/* What VL53LX_DataInit already leaves: MEDIUM at 33,333 us. Stated here so the suite uses the
 * baseline rather than a number invented for a test. */
constexpr uint32_t kBudgetUs{33333};
constexpr uint8_t kDistanceMode{2};

// Scripted behaviour and recorded traffic for the faked device ops.
struct fake_dev {
    int open_rc{0};
    int configure_rc{0};
    int start_rc{0};
    int read_rc{0};
    bool fresh{false};
    int16_t mm{0};
    uint8_t status_code{0};
    uint8_t stream_count{0};
    bool rearm_failed{false};
    uint32_t open_delay_ms{0};
    /* What this read COSTS, stated rather than slept. A real k_msleep would make every timing
     * assertion a race against the host's scheduler; the acquisition layer reads its clock through
     * an injected hook precisely so a test can say how long something took. */
    uint32_t read_cycles{0};

    uint32_t read_delay_ms{0};

    int open_calls{0};
    int start_calls{0};
    int read_calls{0};
    int stop_calls{0};
    uint8_t configured_frequency_hz{0};
    /* What configure() was actually handed. Zero means it was never told anything, which until
     * this commit was the normal case: configure() was a no-op. */
    uint32_t configured_budget_us{0};
    uint8_t configured_distance_mode{0};
    bool lock_held_in_open{false};
    bool lock_held_in_read{false};
};

/* Every device op records who called it. The ownership rule -- while a thread owns the ULD, only that
 * thread may drive it -- is not observable any other way: the chain lock serialises callers but says
 * nothing about how many there were, and the ULD's port keeps ONE transport record, so two callers
 * inside it produce a transport error reported as a good sample. */
/* A free-running cycle counter the test drives by hand. Starting value is deliberately not zero:
 * an implementation that forgot to subtract would still pass from zero. */
uint32_t fake_cycles{0};
/* Advanced by the on_cycle sink, so the publish leg of a cycle has a stated cost of its own. */
uint32_t publish_cost_cycles{0};

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

int fake_configure(void *dev, uint32_t timing_budget_us, uint8_t distance_mode, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    memset(st, 0, sizeof(*st));
    d.configured_budget_us = timing_budget_us;
    d.configured_distance_mode = distance_mode;
    return d.configure_rc;
}

int fake_start(void *dev, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.start_calls;
    memset(st, 0, sizeof(*st));
    return d.start_rc;
}

int fake_read(void *dev, void *, struct tof_cliff_sample *out, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.read_calls;
    record_caller();
    k_sem_give(&cycle_read_seen);
    d.lock_held_in_read = lock_is_held();
    if (d.read_delay_ms != 0)
        k_msleep(d.read_delay_ms); // a sensor that blocks: what a join timeout has
                                   // to cover
    fake_cycles += d.read_cycles;
    memset(st, 0, sizeof(*st));
    memset(out, 0, sizeof(*out));
    if (d.fresh) {
        out->fresh = true;
        out->target_count = 1;
        out->entry_count = 1;
        out->entries[0].range_mm = d.mm;
        out->entries[0].range_status = d.status_code;
        out->stream_count = d.stream_count;
        /* A re-arm failure returns an error but preserves the current sample. Any
         * other error with `fresh` left set models stale output and must not earn
         * sample_present. */
        st->sample_present = d.read_rc == 0 || d.rearm_failed;
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

int fake_grid_open(void *dev, uint8_t, lexxhard::tof_l7::operation_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.open_calls;
    record_caller();
    d.lock_held_in_open = lock_is_held();
    *st = lexxhard::tof_l7::operation_status{};
    if (d.open_delay_ms != 0)
        k_msleep(d.open_delay_ms);
    if (d.open_rc != 0)
        st->failed_stage = lexxhard::tof_l7::stage::initialise;
    return d.open_rc;
}

int fake_grid_configure(void *dev, uint8_t frequency_hz,
                        lexxhard::tof_l7::operation_status *st)
{
    *st = lexxhard::tof_l7::operation_status{};
    auto &d{*static_cast<fake_dev *>(dev)};
    d.configured_frequency_hz = frequency_hz;
    return d.configure_rc;
}

int fake_grid_start(void *dev, lexxhard::tof_l7::operation_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.start_calls;
    *st = lexxhard::tof_l7::operation_status{};
    return d.start_rc;
}

int fake_grid_read(void *dev, void *, lexxhard::tof_l7::sample *out,
                   lexxhard::tof_l7::operation_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.read_calls;
    record_caller();
    k_sem_give(&cycle_read_seen);
    d.lock_held_in_read = lock_is_held();
    if (d.read_delay_ms != 0)
        k_msleep(d.read_delay_ms);
    *st = lexxhard::tof_l7::operation_status{};
    *out = lexxhard::tof_l7::sample{};
    if (d.fresh) {
        out->fresh = true;
        for (size_t zone{0}; zone < lexxhard::tof_l7::kZoneCount; ++zone) {
            out->target_count[zone] = 1;
            out->distance_mm[zone] = static_cast<uint16_t>(d.mm + zone);
            out->target_status[zone] = d.status_code;
        }
        st->sample_present = d.read_rc == 0;
    }
    if (d.read_rc != 0)
        st->failed_stage = lexxhard::tof_l7::stage::fetch;
    return d.read_rc;
}

int fake_grid_stop(void *dev, lexxhard::tof_l7::operation_status *st)
{
    *st = lexxhard::tof_l7::operation_status{};
    record_caller();
    ++static_cast<fake_dev *>(dev)->stop_calls;
    return 0;
}

const acq::grid_source_ops kFakeGridOps{fake_grid_open, fake_grid_configure, fake_grid_start,
                                        fake_grid_read, fake_grid_stop};

// Recorded sink activity.
struct {
    int cycles{0};
    acq::cycle_facts last{};
    int cliff_samples{0};
    int grid_samples{0};
    int last_sample_index{-1};
    int16_t last_sample_mm{0};
    uint16_t last_grid_mm{0};
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
    /* This hook is where the publisher's synchronous CAN sends happen in production, so charging it
     * a cost here is what makes the publish leg measurable at all. */
    fake_cycles += publish_cost_cycles;
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

void on_grid_sample(int index, uint32_t, const acq::source_facts &,
                    const lexxhard::tof_l7::sample &s)
{
    ++rec.grid_samples;
    rec.last_sample_index = index;
    rec.last_grid_mm = s.distance_mm[63];
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

uint32_t clock_cycles()
{
    return fake_cycles;
}

/* One place turns microseconds into this counter's units, because a test that hard-coded cycles
 * would be asserting the host's clock rate rather than the code. */
uint32_t us_to_cycles(uint32_t us)
{
    return k_us_to_cyc_floor32(us);
}

acq::source_desc four_cliff_two_grid[acq::kMaxSources];

acq::config make_config(int count)
{
    acq::config c{};

    for (int i{0}; i < acq::kMaxSources; ++i) {
        auto &d{four_cliff_two_grid[i]};

        d = acq::source_desc{};
        d.kind = (i < 4) ? acq::model::l4_cliff : acq::model::l7_grid;
        d.addr_7bit = static_cast<uint8_t>(0x30 + i);
        d.role_id = static_cast<uint8_t>(i);
        d.dev = &devs[i];
        d.scratch = &devs[i]; /* the fake ops ignore it; only non-null matters */
        if (d.kind == acq::model::l4_cliff) {
            d.ops = &kFakeOps;
            d.cliff_timing_budget_us = kBudgetUs;
            d.cliff_distance_mode = kDistanceMode;
        } else {
            d.grid_ops = &kFakeGridOps;
        }
        if (d.kind == acq::model::l7_grid)
            d.grid_frequency_hz = 15;
    }
    c.sources = four_cliff_two_grid;
    c.source_count = count;
    c.periods.cycle_period_ms = kCyclePeriodMs;
    c.periods.health_period_ms = kHealthPeriodMs;
    c.hooks.on_cycle = on_cycle;
    c.hooks.on_cliff_sample = on_cliff_sample;
    c.hooks.on_grid_sample = on_grid_sample;
    c.hooks.on_cliff_health = on_cliff_health;
    c.hooks.on_cycle_begin = on_cycle_begin;
    c.mapping_state_provider = mapping_provider;
    c.now_ms = clock_ms;
    c.now_cycles = clock_cycles;
    return c;
}

void before(void *)
{
    /* Retire whatever the previous case left running. init() now refuses -EALREADY while the
     * subsystem is live -- because the health work item reads cfg_ from another context -- and
     * several cases call before() themselves inside a loop to re-configure per iteration. */
    acq::teardown();

    acq::stop();
    for (auto &dev : devs)
        dev = fake_dev{};
    k_sem_reset(&cycle_read_seen);
    uld_call_count = 0;
    rec = {};
    sample_cycle_count = 0;
    provider_state = acq::mapping_state::not_ready;
    fake_clock_ms = 1000;
    fake_cycles = 1234567;
    publish_cost_cycles = 0;
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

    c = make_config(acq::kMaxSources);
    c.hooks.on_grid_sample = nullptr;
    zassert_equal(acq::init(c), -EINVAL, "a configured grid needs its typed sink");

    c = make_config(acq::kMaxSources + 1);
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(0);
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(4);
    four_cliff_two_grid[2].ops = nullptr;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(acq::kMaxSources);
    four_cliff_two_grid[4].grid_ops = nullptr;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(acq::kMaxSources);
    four_cliff_two_grid[4].ops = &kFakeOps;
    zassert_equal(acq::init(c), -EINVAL,
                  "a grid descriptor must not carry the point-sensor table too");

    c = make_config(acq::kMaxSources);
    four_cliff_two_grid[4].grid_frequency_hz = 0;
    zassert_equal(acq::init(c), -EINVAL,
                  "a real grid table needs an explicit non-zero frequency");

    c = make_config(4);
    four_cliff_two_grid[1].scratch = nullptr;
    zassert_equal(acq::init(c), -EINVAL, "a cliff source needs its own scratch");
}

ZTEST(tof_acquisition, test_the_l4_ranging_profile_is_injected_and_reaches_the_device)
{
    /* configure() was a no-op until now, so the four cliff sensors ran on whatever
     * VL53LX_DataInit left -- MEDIUM at 33,333 us, a vendor default rather than a decision. The
     * profile now comes from the descriptor and has to arrive at the device unchanged.
     *
     * DELIBERATELY NOT THE BASELINE VALUES. A first version of this case asserted 33,333 and
     * MEDIUM, which a bring-up that hard-coded exactly those would also pass -- the assertion
     * could not tell an injected value from a baked-in one. These two agree with no default
     * anywhere in the tree. */
    constexpr uint32_t kOtherBudgetUs{20000};
    constexpr uint8_t kOtherMode{3};  // LONG, and the suite's default is MEDIUM
    acq::config c{make_config(acq::kMaxSources)};

    for (int i{0}; i < 4; ++i) {
        four_cliff_two_grid[i].cliff_timing_budget_us = kOtherBudgetUs;
        four_cliff_two_grid[i].cliff_distance_mode = kOtherMode;
    }
    zassert_equal(acq::init(c), 0);
    zassert_equal(acq::bring_up(), 0);

    for (int i{0}; i < 4; ++i) {
        zassert_equal(devs[i].configured_budget_us, kOtherBudgetUs, "cliff source %d", i);
        zassert_equal(devs[i].configured_distance_mode, kOtherMode, "cliff source %d", i);
    }
    // And nothing of the sort reached the grid sources, whose table takes a frequency instead.
    zassert_equal(devs[4].configured_budget_us, 0U);
    zassert_equal(devs[4].configured_frequency_hz, 15);
}

ZTEST(tof_acquisition, test_a_missing_or_impossible_ranging_profile_is_refused)
{
    /* No defaults, the same rule the periods follow. A budget invented here would become the
     * specification by being the only number in the build, and a distance mode outside the ULD's
     * three would reach VL53LX_SetDistanceMode as an unhandled case -- the interesting failure
     * being the one where it is quietly accepted. */
    acq::config c{make_config(4)};

    four_cliff_two_grid[2].cliff_timing_budget_us = 0;
    zassert_equal(acq::init(c), -EINVAL, "a cliff source with no timing budget was accepted");

    c = make_config(4);
    four_cliff_two_grid[2].cliff_distance_mode = 0;
    zassert_equal(acq::init(c), -EINVAL);

    c = make_config(4);
    four_cliff_two_grid[2].cliff_distance_mode = 4;  // one past LONG
    zassert_equal(acq::init(c), -EINVAL, "a distance mode the ULD does not define was accepted");

    // All three the ULD defines are legal, and nothing here prefers one of them.
    for (uint8_t mode{1}; mode <= 3; ++mode) {
        c = make_config(4);
        for (int i{0}; i < 4; ++i)
            four_cliff_two_grid[i].cliff_distance_mode = mode;
        zassert_equal(acq::init(c), 0, "mode %u was refused", mode);
        acq::teardown();
    }

    /* And the profile belongs to the cliff model only: a grid descriptor carrying it is the same
     * class of wiring mistake as a grid descriptor carrying the cliff table. */
    c = make_config(acq::kMaxSources);
    four_cliff_two_grid[4].cliff_timing_budget_us = kBudgetUs;
    zassert_equal(acq::init(c), -EINVAL, "a grid descriptor carried an L4 ranging profile");
}

ZTEST(tof_acquisition, test_the_ops_table_has_exactly_five_entries)
{
    // Structural stand-in for "there is no enable operation". An interface that cannot
    // express dropping an enable line is stronger than a test asserting nobody did:
    // dropping an L4's enable returns it to 0x29 and destroys the chain's addressing.
    zassert_equal(sizeof(acq::source_ops), 5 * sizeof(void *),
                  "an operation was added to the device interface - if it is enable, "
                  "the chain's addressing is now reachable from the scheduler");
    zassert_equal(sizeof(acq::grid_source_ops), 5 * sizeof(void *),
                  "an operation was added to the grid interface - if it is enable, "
                  "the chain's addressing is now reachable from the scheduler");
}

/* ------------------------------------------------------------ publication gate ----- */

ZTEST(tof_acquisition, test_the_state_acted_on_is_the_one_the_authority_reports)
{
    /* This case used to be test_proven_is_unreachable_and_has_no_bypass_flag, and it asserted
     * that PROVEN was forced to NOT_READY. That clamp is gone: the proof now ends at the product
     * speed and re-verifies every position there, and the acquisition thread has been measured
     * reading all four sensors at that speed for thousands of consecutive cycles.
     *
     * What the case is for now is the opposite hazard. Nothing between the authority and the
     * publication gate may rewrite the state in EITHER direction -- an inserted clamp would
     * silence a proven chain, and an inserted promotion would publish on an unproven one. Every
     * state goes through unchanged. */
    zassert_equal(acq::init(make_config(4)), 0);

    provider_state = acq::mapping_state::proven;
    zassert_true(acq::effective_mapping_state() == acq::mapping_state::proven,
                 "a proven mapping is no longer rewritten on its way to the gate");
    zassert_true(acq::publication_allowed());

    provider_state = acq::mapping_state::fault;
    zassert_true(acq::effective_mapping_state() == acq::mapping_state::fault);
    zassert_false(acq::publication_allowed(), "a faulted mapping may not publish");

    provider_state = acq::mapping_state::lost;
    zassert_true(acq::effective_mapping_state() == acq::mapping_state::lost);
    zassert_false(acq::publication_allowed(), "a lost mapping may not publish");

    provider_state = acq::mapping_state::not_ready;
    zassert_true(acq::effective_mapping_state() == acq::mapping_state::not_ready);
    zassert_false(acq::publication_allowed(),
                  "publication became possible on an unproven mapping");
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
    zassert_equal(cliff.status.domain, acq::status_domain::l4);
    zassert_equal(grid.status.domain, acq::status_domain::l7);
    zassert_equal(strcmp(acq::operation_stage_name(cliff.status), "fetch"), 0);
    zassert_equal(strcmp(acq::operation_stage_name(grid.status), "fetch"), 0);
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
        {-EIO, true, false, false, false},     {-ETIMEDOUT, true, false, false, false},
        {-EPROTO, false, true, false, false},  {-EBADMSG, false, true, false, false},
        {-ENOSYS, false, false, true, false},  {-EINVAL, false, false, false, true},
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

/* ---------------------------------------------------------- typed L7 grid path
 * ----- */

ZTEST(tof_acquisition, test_the_grid_ops_are_an_explicit_stub)
{
    const acq::grid_source_ops &ops{acq::l7_grid_stub_ops()};
    lexxhard::tof_l7::operation_status st{};
    lexxhard::tof_l7::sample sample{};

    // A named stub rather than a null pointer. Unlike the old prototype stub, its
    // read signature is a real 64-zone grid: an L7 descriptor can no longer be
    // wired to the point-sensor operation table by accident.
    zassert_equal(ops.open(nullptr, 0x30, &st), -ENOSYS);
    zassert_equal(ops.configure(nullptr, 15, &st), -ENOSYS);
    zassert_equal(ops.start(nullptr, &st), -ENOSYS);
    zassert_equal(ops.read_grid_sample(nullptr, nullptr, &sample, &st), -ENOSYS);
    zassert_equal(ops.stop(nullptr, &st), -ENOSYS);
    zassert_false(sample.fresh);
}

ZTEST(tof_acquisition, test_payloads_can_only_reach_their_typed_sink)
{
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    zassert_equal(acq::bring_up(), 0);
    zassert_equal(devs[4].configured_frequency_hz, 15);
    zassert_equal(devs[5].configured_frequency_hz, 15);

    // The two packers must never have to step over or reinterpret each other's
    // data.
    for (auto &d : devs) {
        d.fresh = true;
        d.mm = 500;
    }
    acq::run_cycle();

    zassert_equal(rec.cliff_samples, 4, "only the four cliff sources may reach that sink");
    zassert_equal(rec.grid_samples, 2, "both grids must reach only the grid sink");
    zassert_true(rec.last_sample_index >= 4);
    zassert_equal(rec.last_grid_mm, 563);
}

ZTEST(tof_acquisition, test_an_error_can_never_publish_even_if_an_adapter_leaves_fresh_set)
{
    zassert_equal(acq::init(make_config(acq::kMaxSources)), 0);
    zassert_equal(acq::bring_up(), 0);

    for (auto &d : devs) {
        d.fresh = true;
        d.read_rc = -EIO;
    }
    acq::run_cycle();

    zassert_equal(rec.cliff_samples, 0);
    zassert_equal(rec.grid_samples, 0);
    for (int i{0}; i < acq::kMaxSources; ++i) {
        zassert_true(rec.last.sources[i].transport_error);
        zassert_false(rec.last.sources[i].sample_produced);
    }
}

/* ------------------------------------------------------------------- cadence ------ */

ZTEST(tof_acquisition, test_the_wait_is_the_remainder_of_the_period_not_a_whole_one)
{
    /* The defect this replaces, in one line of arithmetic: a cycle that took 18 ms used to be
     * followed by a full 50 ms wait, giving 68 ms and 14.7 Hz from a schedule configured for 20.
     * The fake clock states the elapsed time; the thread's own wait is a kernel call and could only
     * have been observed by sleeping against it. */
    constexpr uint32_t kPeriod{50};
    const acq::schedule_decision step{acq::next_cycle_due(1050, 1018, kPeriod)};

    zassert_equal(step.wait_ms, 32U, "the work is supposed to happen INSIDE the period");
    zassert_false(step.overran);
    // Advanced from the deadline it met, not from now: otherwise the cadence drifts by however
    // long each cycle happened to take, which is the same defect one step removed.
    zassert_equal(step.next_due_ms, 1100);
}

ZTEST(tof_acquisition, test_a_cycle_that_overran_yields_briefly_and_owes_no_backlog)
{
    constexpr uint32_t kPeriod{50};

    /* Twelve milliseconds late. Not a zero wait -- that is a spin, and this thread holds the chain
     * lock for most of a cycle, so a spinning loop is a chain that never goes idle and a
     * commissioning quiesce that never succeeds. And not a whole period either, which is what this
     * case used to assert: at a 20 ms target a cycle taking 20.1 ms would then run at 40.1 ms, or
     * about 25 Hz, the schedule throwing away half the rate over a 0.1 ms miss. */
    const acq::schedule_decision late{acq::next_cycle_due(1050, 1062, kPeriod)};

    zassert_true(late.overran);
    zassert_true(late.yield_only, "a late cycle was made to wait out a whole period");
    // Re-based on now, not advanced by a period: a late cycle must not be followed by a burst of
    // back-to-back cycles repaying the debt.
    zassert_equal(late.next_due_ms, 1112);

    // Exactly due is the overrun case too, because the alternative is a zero wait.
    const acq::schedule_decision exact{acq::next_cycle_due(1050, 1050, kPeriod)};

    zassert_true(exact.overran);
    zassert_true(exact.yield_only);

    /* THE CRITICAL MISS, which is the whole point of the change: one millisecond late against a
     * 20 ms period. The cycle after it must be allowed to start straight away, so the achieved
     * cadence is the work rather than the work plus a discarded period. */
    const acq::schedule_decision critical{acq::next_cycle_due(1020, 1021, 20)};

    zassert_true(critical.overran, "a miss is still a miss and still gets counted");
    zassert_true(critical.yield_only);
    zassert_equal(critical.next_due_ms, 1041);

    // Very late. Still a brief yield, still no catch-up.
    const acq::schedule_decision very{acq::next_cycle_due(1050, 5000, kPeriod)};

    zassert_true(very.overran);
    zassert_true(very.yield_only);
    zassert_equal(very.next_due_ms, 5050);
}

ZTEST(tof_acquisition, test_the_loop_never_asks_the_kernel_for_a_zero_timeout)
{
    /* The gap a mutation found: every assertion about next_cycle_due() passed while the loop read
     * wait_ms regardless of yield_only, which on a missed deadline is K_MSEC(0) -- K_NO_WAIT, and
     * the spin the whole rule exists to prevent. The decision and the timeout are different
     * statements and both have to be made.
     *
     * K_NO_WAIT compares equal to a zero-tick timeout, so that is what is checked. */
    const acq::schedule_decision missed{acq::next_cycle_due(1050, 1050, 50)};
    const acq::schedule_decision met{acq::next_cycle_due(1050, 1018, 50)};
    const k_timeout_t on_miss{acq::cadence_timeout(missed)};
    const k_timeout_t on_time{acq::cadence_timeout(met)};

    zassert_true(K_TIMEOUT_EQ(on_miss, K_TICKS(1)),
                 "a missed deadline asked for something other than the kernel's shortest wait");
    zassert_false(K_TIMEOUT_EQ(on_miss, K_NO_WAIT), "a missed deadline asked for no wait at all");
    zassert_true(K_TIMEOUT_EQ(on_time, K_MSEC(32)));
    zassert_false(K_TIMEOUT_EQ(on_time, K_NO_WAIT));
}

ZTEST(tof_acquisition, test_a_deadline_further_off_than_a_period_is_repaired_not_obeyed)
{
    /* Found by the property test below rather than by reasoning about it, which is why it is
     * written down here as its own case. The loop cannot produce such a deadline; a clock that
     * stepped backwards can. Obeying it would park acquisition for the difference -- heartbeat
     * still flowing, measurements simply absent, nothing anywhere saying why. */
    constexpr uint32_t kPeriod{20};
    const acq::schedule_decision step{acq::next_cycle_due(1000, 0, kPeriod)};

    zassert_false(step.yield_only, "a repaired deadline is not a miss");
    zassert_equal(step.wait_ms, kPeriod, "a nonsense deadline was waited out");
    zassert_equal(step.next_due_ms, kPeriod, "the deadline was left wrong for the next cycle");
    zassert_false(step.overran, "nothing was late; the deadline was wrong");
}

ZTEST(tof_acquisition, test_the_cadence_never_returns_a_zero_wait)
{
    /* The property, rather than the cases: whatever it is asked, it must not tell the loop to run
     * again with no wait at all. Either it names a wait, and then that wait is non-zero and no
     * longer than the period, or it asks for the kernel's own floor -- never neither. Includes a
     * zero period, which init() refuses -- checked here anyway so the two places cannot disagree
     * about who is responsible. */
    static const int64_t due[] = {0, 1000, 1050, -5};
    static const int64_t now[] = {0, 999, 1050, 1051, 999999};
    static const uint32_t period[] = {0, 1, 20, 50};

    for (size_t d = 0; d < ARRAY_SIZE(due); ++d) {
        for (size_t n = 0; n < ARRAY_SIZE(now); ++n) {
            for (size_t p = 0; p < ARRAY_SIZE(period); ++p) {
                const acq::schedule_decision step{acq::next_cycle_due(due[d], now[n], period[p])};

                if (period[p] == 0U) {
                    zassert_true(step.overran, "a zero period must not read as a met deadline");
                    zassert_true(step.yield_only, "a zero period must still yield");
                    continue;
                }
                if (step.yield_only) {
                    zassert_true(step.overran, "a yield that was not a miss");
                } else {
                    zassert_not_equal(step.wait_ms, 0U, "due %lld now %lld period %u",
                                      static_cast<long long>(due[d]),
                                      static_cast<long long>(now[n]), period[p]);
                    zassert_true(step.wait_ms <= period[p], "a wait longer than the period");
                }
                zassert_true(step.next_due_ms > now[n], "the next deadline is already past");
            }
        }
    }
}

ZTEST(tof_acquisition, test_a_steady_schedule_holds_its_phase_across_uneven_cycles)
{
    /* Cycles of 18, 5 and 31 ms against a 50 ms period. The deadlines must stay on the 50 ms grid
     * regardless, because each one is advanced from the previous DEADLINE and not from the
     * previous finish. A schedule that drifted by the work would land at 1118, 1173, 1254. */
    constexpr uint32_t kPeriod{50};
    int64_t due{1050};
    static const int64_t work[] = {18, 5, 31};
    static const int64_t expected_due[] = {1100, 1150, 1200};
    int64_t now{1000};

    for (size_t i = 0; i < ARRAY_SIZE(work); ++i) {
        now += work[i];
        const acq::schedule_decision step{acq::next_cycle_due(due, now, kPeriod)};

        zassert_false(step.overran, "cycle %zu", i);
        zassert_equal(step.next_due_ms, expected_due[i], "cycle %zu drifted", i);
        // What the loop then does: wait out the remainder, arriving exactly at the deadline.
        now = due;
        due = step.next_due_ms;
    }
}

/* --------------------------------------------------------------- rate [BENCH] ----- */

ZTEST(tof_acquisition, test_the_timing_clock_is_required)
{
    acq::config c{make_config(4)};

    c.now_cycles = nullptr;
    zassert_equal(acq::init(c), -EINVAL,
                  "a missing cycle clock must be refused, not defaulted to a wall clock");
}

ZTEST(tof_acquisition, test_a_read_is_timed_and_the_high_water_mark_holds)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].read_cycles = us_to_cycles(4000);
    acq::run_cycle();
    zassert_within(acq::source_read_us_last(0), 4000, 2);
    zassert_within(acq::source_read_us_max(0), 4000, 2);

    // A shorter read moves `last` and must NOT move `max`: the longest read is the one the
    // cadence has to survive, and a mean would hide it.
    devs[0].read_cycles = us_to_cycles(1000);
    acq::run_cycle();
    zassert_within(acq::source_read_us_last(0), 1000, 2);
    zassert_within(acq::source_read_us_max(0), 4000, 2);

    devs[0].read_cycles = us_to_cycles(9000);
    acq::run_cycle();
    zassert_within(acq::source_read_us_max(0), 9000, 2);
}

ZTEST(tof_acquisition, test_a_counter_wrap_does_not_inflate_a_duration)
{
    /* THE TARGET'S RATE, STATED. This suite runs on native_sim, whose cycle counter is 1 MHz --
     * there the conversion is the identity, both orderings agree, and a test written against the
     * host's rate passes whichever way the code does it. It would pin nothing. The SCB's counter
     * runs at 216 MHz, where converting before subtracting returns about 4.27e9 us for any
     * measurement that spanned the wrap. So the rate is a parameter and this states it. */
    constexpr uint32_t kTargetHz{216000000U};
    constexpr uint32_t kCyclesPerUs{216U};
    /* 1000 us short of the wrap, and a 4000 us interval that crosses it. */
    const uint32_t before{0U - 1000U * kCyclesPerUs};
    const uint32_t after{before + 4000U * kCyclesPerUs};

    zassert_true(after < before, "the interval under test did not actually cross the wrap");
    zassert_equal(acq::elapsed_us_at(before, after, kTargetHz), 4000U);

    // Ordinary intervals, away from the wrap, and the degenerate rate.
    zassert_equal(acq::elapsed_us_at(0U, 4000U * kCyclesPerUs, kTargetHz), 4000U);
    zassert_equal(acq::elapsed_us_at(7U, 7U, kTargetHz), 0U);
    zassert_equal(acq::elapsed_us_at(0U, 1000U, 0U), 0U, "a zero rate must not be divided by");
}

ZTEST(tof_acquisition, test_a_wrap_during_a_real_cycle_is_carried_through)
{
    /* The plumbing half: the same crossing, but through run_cycle() rather than the arithmetic on
     * its own. On this host it cannot tell the two orderings apart -- see the test above for that
     * -- so what it pins is narrower and worth having anyway: nothing in the path stores the raw
     * counter where a duration belongs, and a wrapped read is still recorded. */
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    fake_cycles = 0U - us_to_cycles(1000);
    devs[0].read_cycles = us_to_cycles(4000);
    acq::run_cycle();

    zassert_within(acq::source_read_us_last(0), 4000, 2);
    zassert_within(acq::source_read_us_max(0), 4000, 2);
}

ZTEST(tof_acquisition, test_a_cycle_splits_into_work_publish_and_total)
{
    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::bring_up(), 0);

    for (auto &d : devs)
        d.read_cycles = us_to_cycles(3000);
    publish_cost_cycles = us_to_cycles(5000);
    acq::run_cycle();

    // Four cliff reads under the chain lock. The lock was free, so waiting for it cost nothing --
    // which is a different statement from the work, and is why the two are separate readings.
    zassert_equal(acq::cycle_lock_wait_us_last(), 0U);
    zassert_within(acq::cycle_work_us_last(), 12000, 8);
    // The CAN sends, outside the lock and inside the cadence -- which is why they are timed apart
    // rather than folded into the work.
    zassert_within(acq::cycle_publish_us_last(), 5000, 4);
    zassert_within(acq::cycle_total_us_last(), 17000, 8);
}

ZTEST(tof_acquisition, test_the_cycle_gap_is_start_to_start_and_the_first_one_has_none)
{
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].read_cycles = us_to_cycles(2000);
    acq::run_cycle();
    zassert_equal(acq::cycle_gap_us_max(), 0U,
                  "the first cycle has no predecessor and must not report a gap from zero");

    // Stands in for the thread's wait between cycles. The gap is start to start, so it covers the
    // work, the sends and the wait -- the whole achieved cadence, not the configured period.
    fake_cycles += us_to_cycles(48000);
    acq::run_cycle();
    zassert_within(acq::cycle_gap_us_max(), 50000, 8);

    fake_cycles += us_to_cycles(10000);
    acq::run_cycle();
    zassert_within(acq::cycle_gap_us_max(), 50000, 8,
                   "a shorter gap must not lower the high-water mark");
}

ZTEST(tof_acquisition, test_a_repeated_stream_count_is_not_a_new_measurement)
{
    /* The measurement hazard this whole commit exists to avoid. On this image `fresh` means the
     * device reported data ready, NOT that the data changed, so a rate computed from samples,
     * frames or ROS messages can count one measurement several times. */
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].fresh = true;
    devs[0].stream_count = 7;
    acq::run_cycle();
    zassert_equal(acq::source_new_measurements(0), 1U);
    zassert_equal(acq::source_repeat_measurements(0), 0U);

    acq::run_cycle();
    zassert_equal(acq::source_new_measurements(0), 1U, "the device had not ranged again");
    zassert_equal(acq::source_repeat_measurements(0), 1U);

    devs[0].stream_count = 8;
    acq::run_cycle();
    zassert_equal(acq::source_new_measurements(0), 2U);
    zassert_equal(acq::source_repeat_measurements(0), 1U);

    // 255 -> 0 is an advance, not a repeat: StreamCount is eight bits and wraps, so the test is
    // inequality and never ordering.
    devs[0].stream_count = 255;
    acq::run_cycle();
    devs[0].stream_count = 0;
    acq::run_cycle();
    zassert_equal(acq::source_new_measurements(0), 4U);
    zassert_equal(acq::source_repeat_measurements(0), 1U);

    // And it counts only, it does not gate: every one of those reads still reached the sink.
    zassert_equal(rec.cliff_samples, 5, "a counter must not have become a filter");
}

ZTEST(tof_acquisition, test_a_restart_does_not_call_the_first_measurement_a_repeat)
{
    /* A successful start renumbers the device's stream from zero, so the remembered count is from a
     * session that no longer exists. Comparing across the restart would report the first real
     * measurement of the new session as a repeat -- and a repeat is the shape of a stalled sensor. */
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].fresh = true;
    devs[0].stream_count = 3;
    acq::run_cycle();
    zassert_equal(acq::source_new_measurements(0), 1U);

    acq::stop();
    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();
    zassert_equal(acq::source_new_measurements(0), 2U,
                  "the first measurement after a restart was counted as a repeat");
    zassert_equal(acq::source_repeat_measurements(0), 0U);
}

ZTEST(tof_acquisition, test_a_cycle_that_read_nothing_still_has_a_gap_and_no_read_time)
{
    /* A source that never started is never read, so it has no read time -- but the cycle still
     * happened and still consumed cadence. Reporting nothing at all for such a cycle would make a
     * chain that is failing to start look like a chain that is idle. */
    zassert_equal(acq::init(make_config(1)), 0);
    devs[0].open_rc = -EIO;
    zassert_equal(acq::bring_up(), 0);

    acq::run_cycle();
    fake_cycles += us_to_cycles(50000);
    acq::run_cycle();

    zassert_equal(acq::source_read_us_last(0), 0U, "a source that never started was timed");
    zassert_equal(acq::source_new_measurements(0), 0U);
    zassert_within(acq::cycle_gap_us_max(), 50000, 8);
    zassert_equal(acq::cycles_completed(), 2U);
}

ZTEST(tof_acquisition, test_an_out_of_range_source_reads_zero_rather_than_memory)
{
    zassert_equal(acq::init(make_config(1)), 0);

    zassert_equal(acq::source_read_us_last(-1), 0U);
    zassert_equal(acq::source_read_us_max(acq::kMaxSources), 0U);
    zassert_equal(acq::source_new_measurements(-1), 0U);
    zassert_equal(acq::source_repeat_measurements(acq::kMaxSources), 0U);
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

ZTEST(tof_acquisition, test_waiting_for_a_busy_chain_is_timed_apart_from_the_work)
{
    /* Commissioning holds the chain for seconds at a time. Without this number, a cycle that was
     * late because the chain was busy and one that was late because a sensor was slow are the same
     * reading -- and they are opposite findings.
     *
     * The holder advances the fake clock before releasing, which is how a blocking wait becomes a
     * stated duration rather than a race against the host. */
    zassert_equal(acq::init(make_config(1)), 0);
    zassert_equal(acq::bring_up(), 0);

    k_sem_reset(&holder_took_it);
    k_sem_reset(&holder_release);
    k_sem_reset(&cycle_thread_started);
    k_thread_create(&chain_holder, chain_holder_stack, K_THREAD_STACK_SIZEOF(chain_holder_stack),
                    chain_holder_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    zassert_equal(k_sem_take(&holder_took_it, K_MSEC(500)), 0, "the holder never took the chain");

    k_thread_create(&cycle_runner, cycle_runner_stack, K_THREAD_STACK_SIZEOF(cycle_runner_stack),
                    cycle_runner_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    zassert_equal(k_sem_take(&cycle_thread_started, K_MSEC(500)), 0);
    k_msleep(10);  // let it reach the blocking acquire

    fake_cycles += us_to_cycles(7000);
    k_sem_give(&holder_release);
    zassert_equal(k_thread_join(&cycle_runner, K_MSEC(500)), 0);
    zassert_equal(k_thread_join(&chain_holder, K_MSEC(500)), 0);

    zassert_within(acq::cycle_lock_wait_us_max(), 7000, 4,
                   "the wait for a busy chain was folded into the work instead");
    zassert_within(acq::cycle_lock_wait_us_last(), 7000, 4);
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

ZTEST(tof_acquisition, test_the_work_happens_inside_the_period_and_not_on_top_of_it)
{
    /* The wiring, not the rule. next_cycle_due() is pinned by its own tests; this is the only place
     * that proves the loop actually asks it and waits for what it said. A mutation that put the
     * configured period back into k_sem_take() passes every one of those and fails here.
     *
     * Real time on purpose, because the thread's wait is a kernel call -- which means the numbers
     * have to clear THIS HOST'S sleep granularity. native_sim runs a 100 Hz tick here, so every
     * k_msleep rounds up to 10 ms and adds one tick: a requested 40 ms sleep takes 50. Measured,
     * not assumed. The first version of this test used the suite's 50 ms period and 15 ms reads,
     * and failed against correct code: the rounding ate the entire remainder, every cycle landed on
     * its deadline, and the schedule legitimately degraded to a cycle per period plus work. A
     * demonstration needs a period well above the tick, so this one states its own.
     *
     * 100 ms of work against a 200 ms period. The old loop produced a cycle every 310 ms and this
     * one produces one every 200, which over 1200 ms is 3 cycles against 6. */
    constexpr uint32_t kPeriodMs{200};
    constexpr int kObserveMs{1200};
    acq::config c{make_config(2)};

    c.periods.cycle_period_ms = kPeriodMs;
    zassert_equal(acq::init(c), 0);
    /* 50 ms each once rounded, so 100 ms of work per cycle. read_delay_ms really sleeps;
     * read_cycles is the stated-cost knob the timing tests use and moves no kernel clock. */
    devs[0].read_delay_ms = 40;
    devs[1].read_delay_ms = 40;
    zassert_equal(acq::start(thread_cfg(1000)), 0);

    k_msleep(kObserveMs);
    const int cycles{rec.cycles};
    const uint32_t overruns{acq::cycle_overruns()};

    zassert_equal(acq::try_stop(), 0);

    zassert_true(cycles >= 5, "cycles in %d ms with 100 ms of work and a %u ms period: %d -- the "
                              "work is still being added to the period rather than fitting inside "
                              "it", kObserveMs, kPeriodMs, cycles);
    // And it is a cadence, not a spin: a loop that stopped waiting would be far above this.
    zassert_true(cycles <= 9, "cycles in %d ms: %d -- the loop is not waiting", kObserveMs, cycles);
    zassert_equal(overruns, 0U, "100 ms of work does not overrun a 200 ms period");
}

ZTEST(tof_acquisition, test_work_longer_than_the_period_runs_at_the_work_and_says_so)
{
    /* The rate consequence of the overrun rule, through the loop rather than the arithmetic.
     *
     * 100 ms of work against a 90 ms period. The rule that waited a whole period on a miss produced
     * a cycle every 200 ms here; yielding one tick produces one every 110. Over 1200 ms that is
     * 6 cycles against about 10. When the work is longer than the period the best achievable
     * cadence IS the work, and the schedule must not stand between the loop and it.
     *
     * Every one of them is still counted as an overrun, because "the period is too short for the
     * work" is the finding and running fast is not the same as meeting the cadence. */
    constexpr uint32_t kPeriodMs{90};
    constexpr int kObserveMs{1200};
    acq::config c{make_config(2)};

    c.periods.cycle_period_ms = kPeriodMs;
    zassert_equal(acq::init(c), 0);
    devs[0].read_delay_ms = 40;  // 50 ms each once this host has rounded it
    devs[1].read_delay_ms = 40;
    zassert_equal(acq::start(thread_cfg(1000)), 0);

    k_msleep(kObserveMs);
    const int cycles{rec.cycles};
    const uint32_t overruns{acq::cycle_overruns()};

    zassert_equal(acq::try_stop(), 0);

    zassert_true(cycles >= 8, "cycles in %d ms with 100 ms of work and a %u ms period: %d -- a "
                              "missed deadline is still costing a whole period",
                 kObserveMs, kPeriodMs, cycles);
    zassert_true(overruns >= 8, "overruns seen: %u of %d cycles -- an unmeetable cadence was "
                                "absorbed silently", overruns, cycles);
}

ZTEST(tof_acquisition, test_a_stop_is_still_prompt_while_every_cycle_is_overrunning)
{
    /* The other half of the overrun rule. A loop that yields for one tick between cycles is much
     * closer to a spin than one that waits a period, so the acceptance boundary that mattered
     * before matters more now: after a stop request, no cycle begins and the join returns well
     * inside its bound. */
    acq::config c{make_config(2)};

    c.periods.cycle_period_ms = 20;  // unmeetable on purpose: 100 ms of work
    zassert_equal(acq::init(c), 0);
    devs[0].read_delay_ms = 40;
    devs[1].read_delay_ms = 40;
    zassert_equal(acq::start(thread_cfg(1000)), 0);

    k_msleep(300);
    zassert_true(acq::cycle_overruns() >= 2, "the loop was not actually overrunning");

    const int64_t asked{k_uptime_get()};

    acq::request_stop();
    zassert_equal(acq::join(1000), 0, "the thread did not stop while overrunning");

    const int64_t took{k_uptime_get() - asked};
    const int begins_at_stop{rec.cycle_begins};

    /* One cycle in flight is 100 ms; anything beyond that is the loop failing to notice. */
    zassert_true(took <= 250, "the stop took %lld ms", static_cast<long long>(took));
    zassert_false(acq::thread_running());

    k_msleep(200);
    zassert_equal(rec.cycle_begins, begins_at_stop, "a cycle BEGAN after the stop request");
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
    /* The third acceptance boundary: the timeout path must leave the subsystem
     * exactly as it was, so that a caller which cannot get a clean stop can only
     * refuse. Nothing is aborted -- a thread inside a vendor driver holds the
     * chain lock and a half-finished transfer, and killing it would leave both.
     */
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

/* The start-order window, opened on purpose.
 *
 * With K_NO_WAIT the new thread became runnable inside k_thread_create(), so a priority above the
 * caller's preempted right there -- before the return value ever reached owner_. The thread then
 * asked may_touch_devices(), found thread_active_ already true and owner_ still null, and concluded
 * that IT was the foreign caller. bring_up() refused at the guard, running_ was never set, and every
 * later run_cycle() took the !running_ path. That is the worst shape a fault can have: a thread that
 * is alive, owns the chain, reports itself running, and silently never touches a sensor -- found on
 * hardware, where the only visible symptom was a stack watermark that never moved.
 *
 * Every other thread case in this file runs BELOW the ztest thread and therefore never opens the
 * window, which is exactly how this reached a robot. This one puts the acquisition thread ABOVE its
 * creator, which makes the preemption certain rather than possible.
 */
ZTEST(tof_acquisition, test_a_thread_that_preempts_its_creator_still_brings_the_sources_up)
{
    acq::thread_config t{thread_cfg(500)};
    const int caller_prio{k_thread_priority_get(k_current_get())};

    zassert_equal(acq::init(make_config(4)), 0);
    zassert_equal(acq::begin_epoch(), 0);

    /* A lifetime counter that the legitimate foreign-caller cases also move, so only the DELTA
     * across this start says anything. Non-zero here IS the defect: the acquisition thread
     * refusing itself is recorded as a foreign call. */
    const uint32_t foreign_before{acq::foreign_lifecycle_calls()};

    /* The CALLER has to be preemptible, and that is not a detail. A cooperative thread is never
     * preempted involuntarily in Zephyr, so k_thread_create() would return -- assigning owner_ --
     * before the new thread ran a single instruction, whatever priority it was given. ztest runs
     * its cases cooperatively at priority -1, so the first version of this test, which only raised
     * the NEW thread's priority, passed cleanly against the very defect it was written to catch.
     *
     * Lowering the caller to a preemptible priority and putting the acquisition thread one step
     * above it reproduces the product's arrangement: there the shell thread issuing `tof cliff
     * start` is preemptible at 14 and acquisition runs at 7. */
    k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(9));
    t.priority = k_thread_priority_get(k_current_get()) - 1;

    /* No assertion between these two, so a failure cannot leave the ztest thread at a priority
     * the rest of the suite did not ask for. */
    const int start_rc{acq::start(t)};
    k_thread_priority_set(k_current_get(), caller_prio);

    zassert_equal(start_rc, 0);
    k_msleep(kCyclePeriodMs * 3);
    zassert_equal(acq::try_stop(), 0);

    zassert_equal(acq::foreign_lifecycle_calls() - foreign_before, 0u,
                  "the acquisition thread was refused as a foreign caller");

    for (int i = 0; i < 4; ++i) {
        zassert_equal(devs[i].open_calls, 1, "source %d was never opened", i);
        zassert_equal(devs[i].start_calls, 1, "source %d was never started", i);
        /* stop_locked() skips every source whose started flag is clear, so a stop that happened
         * is proof the source finished bring-up started. Asserting the flag directly would have
         * to read it before try_stop() clears it, and would race the thread to do so. */
        zassert_equal(devs[i].stop_calls, 1, "source %d was never brought up", i);
    }

    /* And cycles really ran: without this the case would still pass against a thread that brought
     * the sources up and then cycled over nothing. */
    zassert_true(rec.cycles >= 2, "no cycle completed: %d", rec.cycles);
    zassert_true(rec.saw_begin);
    zassert_true(devs[0].read_calls >= 2, "the sources were never read: %d", devs[0].read_calls);
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

/* ------------------------------------------------- cumulative observation counters ----- */

/* These exist because every other record in the acquisition layer is per cycle and cleared, so it
 * can report a failure but never sustained success -- and while the PROVEN clamp zeroed the health
 * frame's cycle fields, sustained success was not observable at all. What the counters
 * claim has to be pinned here, because the only other place their meaning is visible is a shell
 * command nobody runs in CI. */

ZTEST(tof_acquisition, test_stats_count_reads_samples_and_cycles)
{
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);
    for (int i{0}; i < 4; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);

    acq::run_cycle();
    acq::run_cycle();
    acq::run_cycle();

    zassert_equal(acq::cycles_completed(), 3U);
    for (int i{0}; i < 4; ++i) {
        zassert_equal(acq::source_reads(i), 3U, "source %d", i);
        zassert_equal(acq::source_samples(i), 3U, "source %d", i);
        zassert_equal(acq::source_read_errors(i), 0U, "source %d", i);
        zassert_equal(acq::source_rearm_failures(i), 0U, "source %d", i);
    }
}

ZTEST(tof_acquisition, test_stats_a_cycle_with_no_sample_still_counts_the_read)
{
    /* The distinction the whole thing is for: a sensor that is being read and has nothing ready is
     * not a sensor that is not being read, and on the wire both are silence. */
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);
    for (int i{0}; i < 4; ++i)
        devs[i].fresh = false;
    zassert_equal(acq::bring_up(), 0);

    acq::run_cycle();
    acq::run_cycle();

    zassert_equal(acq::cycles_completed(), 2U);
    for (int i{0}; i < 4; ++i) {
        zassert_equal(acq::source_reads(i), 2U, "source %d", i);
        zassert_equal(acq::source_samples(i), 0U, "source %d", i);
        zassert_equal(acq::source_read_errors(i), 0U, "source %d: nothing ready is not an error", i);
    }
}

ZTEST(tof_acquisition, test_stats_read_errors_and_rearm_failures_are_counted_separately)
{
    /* They answer different questions. A read that failed produced nothing; a re-arm that failed
     * produced a perfectly good sample and guaranteed there will not be another one. Collapsing
     * them is how a chain silently stops ranging while every read still looks fine. */
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);
    devs[0].read_rc = -EIO;               // a failed read: nothing to show for it
    /* The REAL shape of a re-arm failure, taken from tof_cliff_sensor.c: the sample is fetched and
     * intact, and the call still returns non-zero, because the device will not produce another one.
     * An earlier version of this test used rc == 0 with rearm_failed set, which the sensor cannot
     * produce -- so it asserted that read_errors stays 0 for a re-arm failure, and would have
     * passed just as happily if the two counters had been wired to each other. */
    devs[1].fresh = true;
    devs[1].rearm_failed = true;
    devs[1].read_rc = -EIO;
    devs[2].fresh = true;                 // ordinary success
    zassert_equal(acq::bring_up(), 0);

    acq::run_cycle();
    acq::run_cycle();

    zassert_equal(acq::source_read_errors(0), 2U);
    zassert_equal(acq::source_rearm_failures(0), 0U, "a failed read is not a failed re-arm");
    zassert_equal(acq::source_samples(0), 0U);

    zassert_equal(acq::source_read_errors(1), 2U, "a re-arm failure returns non-zero and counts");
    zassert_equal(acq::source_rearm_failures(1), 2U);
    zassert_equal(acq::source_samples(1), 2U, "the sample it did return still counts");

    zassert_equal(acq::source_read_errors(2), 0U);
    zassert_equal(acq::source_rearm_failures(2), 0U);
    zassert_equal(acq::source_samples(2), 2U);
}

ZTEST(tof_acquisition, test_stats_survive_begin_epoch)
{
    /* begin_epoch() resets the CONTRACT's cycle_seq to 0, which is correct and is why these
     * counters are separate from it: a reader watching a renumbered cycle_seq for liveness would
     * see it drop to zero and conclude the thread had stopped. */
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);
    for (int i{0}; i < 4; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();
    acq::run_cycle();
    acq::stop();

    zassert_equal(acq::begin_epoch(), 0);

    zassert_equal(acq::cycles_completed(), 2U, "a new epoch erased the lifetime cycle count");
    zassert_equal(acq::source_reads(0), 2U, "a new epoch erased the per-source counts");
    zassert_equal(acq::source_samples(0), 2U);
}

ZTEST(tof_acquisition, test_stats_are_cleared_by_init)
{
    /* Counts carried across an init() would describe a source table that no longer exists, and
     * index 2 of the old table is not index 2 of the new one. */
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);
    for (int i{0}; i < 4; ++i)
        devs[i].fresh = true;
    zassert_equal(acq::bring_up(), 0);
    acq::run_cycle();
    zassert_equal(acq::cycles_completed(), 1U);

    acq::teardown();
    acq::config again{make_config(4)};
    zassert_equal(acq::init(again), 0);

    zassert_equal(acq::cycles_completed(), 0U);
    for (int i{0}; i < acq::kMaxSources; ++i) {
        zassert_equal(acq::source_reads(i), 0U, "source %d", i);
        zassert_equal(acq::source_samples(i), 0U, "source %d", i);
        zassert_equal(acq::source_read_errors(i), 0U, "source %d", i);
        zassert_equal(acq::source_rearm_failures(i), 0U, "source %d", i);
        zassert_equal(acq::source_last_status(i), 0U, "source %d", i);
    }
}

ZTEST(tof_acquisition, test_stats_an_out_of_range_index_reads_zero)
{
    /* Every one of these indexes an array, and the caller is a shell command taking an operator's
     * word for the index. Reading past the end is not an acceptable answer to a typo. */
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);

    const int bad_indexes[]{-1, acq::kMaxSources, acq::kMaxSources + 1, 1000};
    for (const int bad : bad_indexes) {
        zassert_equal(acq::source_reads(bad), 0U, "index %d", bad);
        zassert_equal(acq::source_samples(bad), 0U, "index %d", bad);
        zassert_equal(acq::source_read_errors(bad), 0U, "index %d", bad);
        zassert_equal(acq::source_rearm_failures(bad), 0U, "index %d", bad);
        zassert_equal(acq::source_last_status(bad), 0U, "index %d", bad);
        zassert_equal(acq::source_identity(bad), 0U, "index %d", bad);
    }
}

ZTEST(tof_acquisition, test_stats_identity_uses_the_product_source_order)
{
    /* THE PRODUCT ORDER, built here rather than taken from make_config(): on the real chain the two
     * grid sensors occupy descriptor indexes 0 and 1 and the four cliff L4s are 2..5, carrying
     * contract roles 0..3. make_config() has it the other way round, which is fine for the cases
     * that only need four cliff sources -- and useless for this one, because a helper that puts the
     * cliff sources first cannot catch anything about where they actually are. An index/role
     * confusion here names the wrong corner of the robot. */
    /* STATIC, because init() keeps the pointer and the suite's teardown walks the table after
     * this function has returned -- a local array here is a dangling read inside stop_locked(),
     * which is exactly how the first version of this test crashed. make_config() uses a
     * file-scope array for the same reason. */
    static acq::source_desc product[acq::kMaxSources];
    for (int i{0}; i < acq::kMaxSources; ++i) {
        auto &d{product[i]};

        d = acq::source_desc{};
        d.kind = (i < 2) ? acq::model::l7_grid : acq::model::l4_cliff;
        d.addr_7bit = static_cast<uint8_t>(0x2A + i);
        d.role_id = (i < 2) ? 255 : static_cast<uint8_t>(i - 2);
        d.dev = &devs[i];
        d.scratch = &devs[i];
        if (d.kind == acq::model::l4_cliff) {
            d.ops = &kFakeOps;
            d.cliff_timing_budget_us = kBudgetUs;
            d.cliff_distance_mode = kDistanceMode;
        } else {
            d.grid_ops = &kFakeGridOps;
        }
        if (d.kind == acq::model::l7_grid)
            d.grid_frequency_hz = 15;
    }

    acq::config c{make_config(6)};
    c.sources = product;
    c.source_count = acq::kMaxSources;
    zassert_equal(acq::init(c), 0);

    for (int i{0}; i < 2; ++i)
        zassert_false(acq::identity_is_cliff(acq::source_identity(i)),
                      "index %d is a grid sensor on the product chain", i);
    for (int i{2}; i < 6; ++i) {
        const uint32_t id{acq::source_identity(i)};
        zassert_true(acq::identity_is_cliff(id), "index %d is a cliff sensor", i);
        zassert_equal(acq::identity_role(id), i - 2,
                      "acq index %d carries contract role %d, not %d", i, i - 2, i);
    }
}

ZTEST(tof_acquisition, test_bring_up_rereads_roles_keyed_after_init)
{
    /* The real sequence on the board, and the defect it used to hide. init() runs at boot with no
     * proven mapping, so every cliff role is 255. The proof's commit later writes the real roles
     * into the descriptor table -- the same array cfg.sources points at -- and nothing copied them
     * into facts_. The publisher refuses any sample whose descriptor role and facts role disagree,
     * marks the cycle invalid, and facts_'s copy is also the value it would have encoded: a stale
     * 255 suppresses EVERY measurement, silently. While the PROVEN clamp was in place this was
     * invisible, because nothing was being published anyway. */
    /* STATIC, because init() keeps the pointer and the suite's teardown walks the table after
     * this function has returned -- a local array here is a dangling read inside stop_locked(),
     * which is exactly how the first version of this test crashed. make_config() uses a
     * file-scope array for the same reason. */
    static acq::source_desc product[acq::kMaxSources];
    for (int i{0}; i < acq::kMaxSources; ++i) {
        auto &d{product[i]};

        d = acq::source_desc{};
        d.kind = (i < 2) ? acq::model::l7_grid : acq::model::l4_cliff;
        d.addr_7bit = static_cast<uint8_t>(0x2A + i);
        d.role_id = 255; /* unassigned: nothing is proven at init time */
        d.dev = &devs[i];
        d.scratch = &devs[i];
        if (d.kind == acq::model::l4_cliff) {
            d.ops = &kFakeOps;
            d.cliff_timing_budget_us = kBudgetUs;
            d.cliff_distance_mode = kDistanceMode;
        } else {
            d.grid_ops = &kFakeGridOps;
        }
        if (d.kind == acq::model::l7_grid)
            d.grid_frequency_hz = 15;
    }

    acq::config c{make_config(6)};
    c.sources = product;
    c.source_count = acq::kMaxSources;
    zassert_equal(acq::init(c), 0);
    for (int i{2}; i < 6; ++i)
        zassert_equal(acq::identity_role(acq::source_identity(i)), 255,
                      "index %d should be unassigned before any proof", i);

    /* What the proof's commit does: key the descriptors in place. */
    for (int i{2}; i < 6; ++i)
        product[i].role_id = static_cast<uint8_t>(i - 2);

    zassert_equal(acq::bring_up(), 0);

    /* THE FIELD THAT MATTERS IS facts_.sources[].role_id, not the stats mirror. The publisher
     * compares the descriptor's role against THAT one and refuses the sample when they differ, and
     * that one is also what it encodes into the frame. An earlier version of this test asserted
     * only on source_identity(), and deleting the facts refresh left it passing -- a test of the
     * diagnostic instead of the defect. */
    acq::cycle_facts f{};
    acq::copy_facts(f);
    for (int i{2}; i < 6; ++i)
        zassert_equal(f.sources[i].role_id, i - 2,
                      "bring-up did not re-read the keyed role into the facts for index %d", i);
    for (int i{0}; i < 2; ++i)
        zassert_equal(f.sources[i].role_id, 255, "a grid source has no cliff role");

    /* And the diagnostic must agree with it, or the shell would print a role the publisher is not
     * using. */
    for (int i{2}; i < 6; ++i)
        zassert_equal(acq::identity_role(acq::source_identity(i)), i - 2,
                      "the stats identity disagrees with the facts for index %d", i);
}

ZTEST(tof_acquisition, test_stats_last_status_carries_both_halves_in_one_word)
{
    /* One word, because two would not be a snapshot: a reader preempted between them comes back
     * with this cycle's stage beside the last cycle's errno. */
    acq::config c{make_config(4)};
    zassert_equal(acq::init(c), 0);
    devs[0].read_rc = -EIO;  // the fake sets stage FETCH on a failed read
    zassert_equal(acq::bring_up(), 0);

    acq::run_cycle();

    const uint32_t st{acq::source_last_status(0)};
    zassert_equal(acq::last_status_stage(st), TOF_CLIFF_STAGE_FETCH);
    zassert_equal(acq::last_status_errno(st), 0, "the fake records no port errno");

    /* A negative errno must survive the packing as a negative number: it is carried in 16 bits and
     * a truncation that dropped the sign would report a different errno entirely. */
    zassert_equal(acq::last_status_errno(0xFFFFU), -1);
    zassert_equal(acq::last_status_errno((static_cast<uint32_t>(TOF_CLIFF_STAGE_FETCH) << 16) |
                                         (static_cast<uint32_t>(-5) & 0xFFFFU)),
                  -5);

    /* The clamp boundaries. An errno outside 16 bits is clamped rather than truncated, because a
     * truncated errno is a DIFFERENT errno and would be read as one -- -65536 becoming 0 would
     * report success. The extremes must survive as the extremes. */
    zassert_equal(acq::last_status_errno(static_cast<uint32_t>(INT16_MIN) & 0xFFFFU), INT16_MIN);
    zassert_equal(acq::last_status_errno(static_cast<uint32_t>(INT16_MAX) & 0xFFFFU), INT16_MAX);
    zassert_equal(acq::last_status_errno(0U), 0);
}
