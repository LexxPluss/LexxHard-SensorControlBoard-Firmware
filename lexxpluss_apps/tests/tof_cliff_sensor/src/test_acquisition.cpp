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

    int open_calls{0};
    int start_calls{0};
    int read_calls{0};
    int stop_calls{0};
    bool lock_held_in_open{false};
    bool lock_held_in_read{false};
};

fake_dev devs[acq::kMaxSources];

bool lock_is_held()
{
    return lexxhard::tof_chain_controller::chain_lock().owner == k_current_get();
}

int fake_open(void *dev, uint8_t, acq::op_status *st)
{
    auto &d{*static_cast<fake_dev *>(dev)};

    ++d.open_calls;
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
    d.lock_held_in_read = lock_is_held();
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

void on_cliff_sample(int index, const acq::source_facts &, const struct tof_cliff_sample &s)
{
    ++rec.cliff_samples;
    rec.last_sample_index = index;
    rec.last_sample_mm = s.entries[0].range_mm;
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
        d.ops = &kFakeOps;
    }
    c.sources = four_cliff_two_grid;
    c.source_count = count;
    c.periods.cycle_period_ms = kCyclePeriodMs;
    c.periods.health_period_ms = kHealthPeriodMs;
    c.hooks.on_cycle = on_cycle;
    c.hooks.on_cliff_sample = on_cliff_sample;
    c.hooks.on_cliff_health = on_cliff_health;
    c.mapping_state_provider = mapping_provider;
    c.now_ms = clock_ms;
    return c;
}

void before(void *)
{
    acq::stop();
    memset(devs, 0, sizeof(devs));
    rec = {};
    provider_state = acq::mapping_state::not_ready;
    fake_clock_ms = 1000;
}

}  // namespace

ZTEST_SUITE(tof_acquisition, NULL, NULL, before, NULL, NULL);

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
    zassert_equal(acq::init(c), -EINVAL, "a cliff source needs its own scratch");
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

ZTEST(tof_acquisition, test_proven_is_clamped_until_the_enable_chain_is_fixed)
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

ZTEST(tof_acquisition, test_protocol_and_transport_errors_stay_distinguishable)
{
    zassert_equal(acq::init(make_config(2)), 0);
    zassert_equal(acq::bring_up(), 0);

    devs[0].read_rc = -EPROTO;
    devs[1].read_rc = -EIO;
    acq::run_cycle();

    zassert_true(rec.last.sources[0].protocol_error);
    zassert_false(rec.last.sources[0].transport_error);
    zassert_true(rec.last.sources[1].transport_error);
    zassert_false(rec.last.sources[1].protocol_error);
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
    // to the wrong driver has to be deliberate.
    zassert_equal(ops.open(nullptr, 0x30, &st), -ENOSYS);
    zassert_equal(ops.configure(nullptr, &st), -ENOSYS);
    zassert_equal(ops.start(nullptr, &st), -ENOSYS);
    zassert_equal(ops.read_once(nullptr, nullptr, &sample, &st), -ENOSYS);
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
