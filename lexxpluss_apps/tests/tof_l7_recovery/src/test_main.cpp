/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * What the boot-time L7 recovery pass does, and -- more to the point -- what it must not do.
 *
 * The pass exists because an SCB-only reset cannot reset an L7: the enable chain gates that board's
 * comms and leaves it powered, and no XSHUT or power-rail control for it exists on this carrier. So
 * a surviving sensor can only be told to stop over the bus, and the ULD makes that possible because
 * stop_ranging and is_alive both work from an address alone.
 *
 * TWO PROPERTIES CARRY THE DESIGN and both are pinned below. A stop is never issued to an address
 * that did not answer, because with a zeroed configuration stop_ranging polls for up to five
 * seconds and most boots are cold boots with nothing to recover -- an unguarded pass would add ten
 * seconds to every one of them. And a failure is reported rather than acted on: enumeration owns
 * the verdict on chain health, and a recovery pass that could stop a boot would be a second opinion
 * on it.
 */

#include <zephyr/ztest.h>

#include <string.h>

#include "tof_l7_recovery.hpp"

namespace
{

namespace rec = lexxhard::tof_l7_recovery;

constexpr size_t kMaxCalls{8};

struct recorder {
    /* Scripted answers, consumed in order. */
    bool alive_answer[kMaxCalls]{};
    int alive_rc[kMaxCalls]{};
    int stop_rc[kMaxCalls]{};

    uint8_t probed[kMaxCalls]{};
    size_t probes{0};
    uint8_t stopped[kMaxCalls]{};
    size_t stops{0};
};

recorder rec_{};

int fake_is_alive(void *, uint8_t addr, bool *alive)
{
    const size_t n{rec_.probes};
    if (n < kMaxCalls)
        rec_.probed[n] = addr;
    ++rec_.probes;
    if (alive != nullptr)
        *alive = n < kMaxCalls ? rec_.alive_answer[n] : false;
    return n < kMaxCalls ? rec_.alive_rc[n] : 0;
}

int fake_stop(void *, uint8_t addr)
{
    const size_t n{rec_.stops};
    if (n < kMaxCalls)
        rec_.stopped[n] = addr;
    ++rec_.stops;
    return n < kMaxCalls ? rec_.stop_rc[n] : 0;
}

rec::ops wired_ops()
{
    rec::ops o{};
    o.is_alive = fake_is_alive;
    o.stop_ranging = fake_stop;
    o.ctx = nullptr;
    return o;
}

/* The product chain's two grid targets, as enumeration assigns them. */
rec::request two_sensors()
{
    rec::request q{};
    q.addr_7bit[0] = 0x2AU;
    q.addr_7bit[1] = 0x2BU;
    q.count = 2;
    return q;
}

void reset_recorder()
{
    memset(&rec_, 0, sizeof(rec_));
}

}  // namespace

ZTEST_SUITE(tof_l7_recovery, nullptr, nullptr, nullptr, nullptr, nullptr);

/* The ordinary boot. Both sensors are silent because the board came up from power-on, so the pass
 * finds nothing, spends two probes and issues no stop at all. This is the case that must stay
 * cheap, since it is nearly every boot. */
ZTEST(tof_l7_recovery, test_a_cold_chain_costs_two_probes_and_no_stop)
{
    reset_recorder();
    rec_.alive_answer[0] = false;
    rec_.alive_answer[1] = false;

    const rec::report r{rec::run(wired_ops(), two_sensors())};

    zassert_equal(r.count, 2U);
    zassert_equal(r.at[0], rec::result::absent);
    zassert_equal(r.at[1], rec::result::absent);
    zassert_equal(r.stopped, 0U);
    zassert_false(r.any_failure);
    zassert_equal(rec_.probes, 2U);
    zassert_equal(rec_.stops, 0U, "a silent address must never be sent the five-second stop");
}

/* The reset this module was written for: the STM32 restarted, both sensors kept running. */
ZTEST(tof_l7_recovery, test_two_survivors_are_each_stopped_once_at_their_own_address)
{
    reset_recorder();
    rec_.alive_answer[0] = true;
    rec_.alive_answer[1] = true;

    const rec::report r{rec::run(wired_ops(), two_sensors())};

    zassert_equal(r.at[0], rec::result::stopped);
    zassert_equal(r.at[1], rec::result::stopped);
    zassert_equal(r.stopped, 2U);
    zassert_false(r.any_failure);
    zassert_equal(rec_.stops, 2U);
    zassert_equal(rec_.stopped[0], 0x2AU);
    zassert_equal(rec_.stopped[1], 0x2BU);
    zassert_equal(rec_.probed[0], 0x2AU);
    zassert_equal(rec_.probed[1], 0x2BU);
}

/* One sensor survived and the other did not. Nothing about this is exotic -- the chain enables the
 * two boards through the same flip-flops -- but a pass that handled only the uniform cases would
 * either stop a silent address or skip a live one. */
ZTEST(tof_l7_recovery, test_a_mixed_chain_stops_only_the_one_that_answered)
{
    reset_recorder();
    rec_.alive_answer[0] = false;
    rec_.alive_answer[1] = true;

    const rec::report r{rec::run(wired_ops(), two_sensors())};

    zassert_equal(r.at[0], rec::result::absent);
    zassert_equal(r.at[1], rec::result::stopped);
    zassert_equal(r.stopped, 1U);
    zassert_false(r.any_failure);
    zassert_equal(rec_.stops, 1U);
    zassert_equal(rec_.stopped[0], 0x2BU, "the stop went to the address that answered");
}

/* A bus that cannot carry the question has not answered it. Treating a transport error as "absent"
 * would be the comfortable reading and the wrong one; treating it as a survivor would spend five
 * seconds talking to a bus already known to be broken. It is its own outcome, and no stop follows. */
ZTEST(tof_l7_recovery, test_a_probe_transport_error_is_neither_absent_nor_a_reason_to_stop)
{
    reset_recorder();
    rec_.alive_rc[0] = -5;       /* -EIO */
    rec_.alive_answer[1] = true;

    const rec::report r{rec::run(wired_ops(), two_sensors())};

    zassert_equal(r.at[0], rec::result::probe_failed);
    zassert_true(r.any_failure);
    zassert_equal(rec_.stops, 1U, "only the second address, which answered, may be stopped");
    zassert_equal(rec_.stopped[0], 0x2BU);
    zassert_equal(r.at[1], rec::result::stopped, "one failure does not abandon the rest of the chain");
}

/* A stop that fails is recorded and the pass carries on. The boot is not this module's to stop. */
ZTEST(tof_l7_recovery, test_a_failed_stop_is_reported_and_does_not_end_the_pass)
{
    reset_recorder();
    rec_.alive_answer[0] = true;
    rec_.alive_answer[1] = true;
    rec_.stop_rc[0] = -110;      /* -ETIMEDOUT, the five-second poll giving up */

    const rec::report r{rec::run(wired_ops(), two_sensors())};

    zassert_equal(r.at[0], rec::result::stop_failed);
    zassert_equal(r.at[1], rec::result::stopped);
    zassert_equal(r.stopped, 1U, "stopped counts sessions actually ended, not attempts");
    zassert_true(r.any_failure);
    zassert_equal(rec_.stops, 2U);
}

/* Half an ops is worse than none: it would spend the probes and then be unable to act on what they
 * found, reporting every survivor as untouched. */
ZTEST(tof_l7_recovery, test_an_incomplete_ops_makes_no_calls_at_all)
{
    reset_recorder();
    rec::ops o{wired_ops()};
    o.stop_ranging = nullptr;

    const rec::report r{rec::run(o, two_sensors())};

    zassert_true(r.any_failure);
    zassert_equal(rec_.probes, 0U);
    zassert_equal(rec_.stops, 0U);
    zassert_equal(r.at[0], rec::result::not_attempted);
    zassert_equal(r.at[1], rec::result::not_attempted);
}

/* An address this pass cannot use is a caller defect. It must not become a bus operation, and it
 * must not take the other sensor down with it. */
ZTEST(tof_l7_recovery, test_an_unusable_address_is_skipped_without_touching_the_bus)
{
    reset_recorder();
    rec_.alive_answer[0] = true;
    rec::request q{two_sensors()};
    q.addr_7bit[0] = 0x00U;

    const rec::report r{rec::run(wired_ops(), q)};

    zassert_equal(r.at[0], rec::result::not_attempted);
    zassert_true(r.any_failure);
    zassert_equal(rec_.probes, 1U, "only the usable address is probed");
    zassert_equal(rec_.probed[0], 0x2BU);
    zassert_equal(r.at[1], rec::result::stopped);
}

/* More addresses than this chain holds is also a caller defect, and the safe reading of it is to do
 * the work for the sensors that exist. Refusing the whole pass would turn the defect into a boot
 * that silently recovers nothing, which is the failure this module was written to end. */
ZTEST(tof_l7_recovery, test_an_oversized_request_is_clamped_rather_than_refused)
{
    reset_recorder();
    rec_.alive_answer[0] = true;
    rec_.alive_answer[1] = true;
    rec::request q{two_sensors()};
    q.count = 7;

    const rec::report r{rec::run(wired_ops(), q)};

    zassert_equal(r.count, rec::kMaxGridSensors);
    zassert_equal(r.stopped, 2U);
    zassert_equal(rec_.probes, 2U);
}

/* Nothing asked for, nothing done, and not an error: an image built without the L7 ULD has no grid
 * addresses to offer and must not be told its boot went wrong. */
ZTEST(tof_l7_recovery, test_an_empty_request_is_a_clean_no_op)
{
    reset_recorder();

    const rec::report r{rec::run(wired_ops(), rec::request{})};

    zassert_equal(r.count, 0U);
    zassert_equal(r.stopped, 0U);
    zassert_false(r.any_failure);
    zassert_equal(rec_.probes, 0U);
    zassert_equal(rec_.stops, 0U);
}

/* The names reach a log line a person reads at three in the morning, so they are pinned. */
ZTEST(tof_l7_recovery, test_every_result_has_its_own_name)
{
    zassert_true(strcmp(rec::result_name(rec::result::not_attempted), "not_attempted") == 0);
    zassert_true(strcmp(rec::result_name(rec::result::absent), "absent") == 0);
    zassert_true(strcmp(rec::result_name(rec::result::stopped), "stopped") == 0);
    zassert_true(strcmp(rec::result_name(rec::result::stop_failed), "stop_failed") == 0);
    zassert_true(strcmp(rec::result_name(rec::result::probe_failed), "probe_failed") == 0);
}
