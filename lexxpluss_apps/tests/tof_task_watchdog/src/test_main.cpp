/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Feeding the hardware watchdog on task progress instead of on a timer.
 *
 * Most of this suite is about NOT firing, which is the right proportion. A watchdog that resets a
 * healthy board is worse than one that never fires: it takes the machine out of service and hides
 * its own cause, and the two easiest ways to build one are both starting conditions rather than
 * exotic faults. A post-DFU boot blocks every CAN sender until the robot PC returns, so a board
 * that has sent nothing may be working perfectly and waiting. An ULD download holds the chain for
 * two seconds per sensor, which dwarfs any per-cycle bound.
 *
 * The cases that must fire are the ones that made this necessary: an L7 open that never returns,
 * and any single task stopping while the others carry on.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_task_watchdog.hpp"

namespace
{

namespace wd = lexxhard::tof_task_watchdog;

struct harness {
    wd::state st{};
    wd::bounds b{};
    wd::input in{};

    /* Everything healthy and finished, nothing in flight: the shape of a board between cycles. */
    void healthy()
    {
        in.acquisition = {10, 10};
        in.can_send = {20, 20};
        in.health = {5, 5};
        in.l7 = {3, 3};
        in.zcan_loops = 1000;
    }

    /* Advances time and calls the decision. Every activity makes progress unless `freeze` says
     * otherwise, which is how a single stuck task is expressed. */
    bool tick(uint32_t ms, bool advance_acq = true, bool advance_send = true,
              bool advance_health = true, bool advance_zcan = true, bool advance_l7 = true)
    {
        in.now_ms += ms;
        if (advance_acq)   { ++in.acquisition.begun; ++in.acquisition.ended; }
        if (advance_send)  { ++in.can_send.begun; ++in.can_send.ended; }
        if (advance_health){ ++in.health.begun; ++in.health.ended; }
        if (advance_zcan)  { ++in.zcan_loops; }
        if (advance_l7)    { ++in.l7.begun; ++in.l7.ended; }
        return wd::feed_allowed(st, b, in);
    }

    /* Takes the baseline the way the product does: after commissioning, before the first L7 open. */
    void arm()
    {
        healthy();
        in.baseline_point = true;
        (void)tick(1);
        zassert_equal(st.current, wd::phase::armed, "baseline should have been taken");
        /* Past the grace period, with everything still moving. */
        for (int i{0}; i < 5; ++i)
            zassert_true(tick(b.grace_ms / 2));
    }
};

}  // namespace

ZTEST_SUITE(tof_task_watchdog, nullptr, nullptr, nullptr, nullptr, nullptr);

/* THE BOOT THIS MUST NOT RESET. The robot PC is not on the bus yet, so nothing has ever been sent
 * and nothing ever will be until it arrives. That is a working board waiting, and no amount of
 * waiting may turn it into a reset. */
ZTEST(tof_task_watchdog, test_a_board_whose_host_never_arrives_is_never_judged)
{
    harness h{};
    h.healthy();
    h.in.can_send = {0, 0};      /* never ACKed, never completed */
    h.in.baseline_point = true;

    for (int i{0}; i < 200; ++i)
        zassert_true(h.tick(1000, true, false), "a missing host is not a hang");

    zassert_equal(h.st.current, wd::phase::waiting, "no baseline, so nothing was ever judged");
    zassert_equal(h.st.why, 0U);
}

/* The other half of the same rule: every activity must have FINISHED something, not merely be
 * configured. A zero is not a baseline. */
ZTEST(tof_task_watchdog, test_the_baseline_waits_for_every_activity_to_have_finished_once)
{
    harness h{};
    h.healthy();
    h.in.health = {0, 0};
    h.in.baseline_point = true;
    (void)h.tick(1, true, true, false);
    zassert_equal(h.st.current, wd::phase::waiting);

    h.in.health = {1, 1};
    (void)h.tick(1, true, true, false);
    zassert_equal(h.st.current, wd::phase::armed);
}

/* A baseline taken with a cycle in flight would make the first judgement after the grace period
 * about work that began before anybody was watching. */
ZTEST(tof_task_watchdog, test_the_baseline_is_not_taken_while_something_is_in_flight)
{
    harness h{};
    h.healthy();
    h.in.acquisition = {11, 10};     /* one cycle inside */
    h.in.baseline_point = true;

    (void)h.tick(1, false);
    zassert_equal(h.st.current, wd::phase::waiting);

    h.in.acquisition = {11, 11};
    (void)h.tick(1, false);
    zassert_equal(h.st.current, wd::phase::armed);
}

/* Bring-up's long operations are declared, and while one is declared the per-cycle bounds do not
 * apply. Two ULD downloads at about two seconds each would otherwise trip every bound there is. */
ZTEST(tof_task_watchdog, test_a_declared_long_operation_suspends_the_per_cycle_bounds)
{
    harness h{};
    h.arm();

    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;
    h.in.acquisition.begun = h.in.acquisition.ended + 1;   /* stuck inside, legitimately */

    for (int i{0}; i < 8; ++i)
        zassert_true(h.tick(1000, false, false, false, false, false),
                     "a declared long operation is not a hang");

    zassert_equal(h.st.current, wd::phase::armed);
}

/* And the declaration is bounded, because "the firmware said it was busy" is the shape of excuse a
 * hang would offer. This is the first L7 open never returning. */
ZTEST(tof_task_watchdog, test_a_long_operation_that_never_ends_is_itself_a_reason_to_stop)
{
    harness h{};
    h.arm();

    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;
    h.in.l7.begun = h.in.l7.ended + 1;       /* inside the first open, and it never comes back */

    bool fed{true};
    for (int i{0}; i < 60 && fed; ++i)
        fed = h.tick(1000, false, false, false, false, false);

    zassert_false(fed, "an unbounded declaration is not a bound");
    zassert_equal(h.st.current, wd::phase::stopped);
    zassert_true((h.st.why & wd::long_operation_over) != 0U);
}

/* The same hang without the declaration, which is what an L7 read that wedges mid-acquisition looks
 * like: the L7 bound alone fires. */
ZTEST(tof_task_watchdog, test_an_undeclared_l7_that_never_returns_stops_the_feed)
{
    harness h{};
    h.arm();
    h.in.l7_expected = true;
    h.in.l7.begun = h.in.l7.ended + 1;

    bool fed{true};
    for (int i{0}; i < 20 && fed; ++i)
        fed = h.tick(1000, true, true, true, true, false);

    zassert_false(fed);
    zassert_true((h.st.why & wd::stuck_l7) != 0U);
}

/* An image built without the L7 ULD has no grid progress to make, and must not be reset for the
 * absence of a sensor it was never built to talk to. */
ZTEST(tof_task_watchdog, test_an_image_without_the_l7_is_not_asked_for_l7_progress)
{
    harness h{};
    h.healthy();
    h.in.l7 = {0, 0};
    h.in.l7_expected = false;
    h.in.baseline_point = true;

    (void)h.tick(1, true, true, true, true, false);
    zassert_equal(h.st.current, wd::phase::armed, "a missing L7 must not block the baseline");

    h.in.l7.begun = 1;      /* would look stuck, if anybody were looking */
    bool fed{true};
    for (int i{0}; i < 30 && fed; ++i)
        fed = h.tick(1000, true, true, true, true, false);

    zassert_true(fed);
    zassert_equal(h.st.why & wd::stuck_l7, 0U);
}

/* One task stops and the rest carry on. The reason names that task and only that task, because
 * after an unexplained reset the first useful question is which one. */
ZTEST(tof_task_watchdog, test_a_single_stalled_task_is_named_on_its_own)
{
    harness h{};
    h.arm();
    h.in.health.begun = h.in.health.ended + 1;

    bool fed{true};
    for (int i{0}; i < 20 && fed; ++i)
        fed = h.tick(1000, true, true, false, true, true);

    zassert_false(fed);
    zassert_equal(h.st.why, static_cast<uint32_t>(wd::stuck_health),
                  "exactly one reason, and it is the one that stopped");
}

/* The free-running loop is judged on movement alone: there is no "inside" it to be stuck in, so
 * silence is the whole symptom. */
ZTEST(tof_task_watchdog, test_the_zcan_loop_is_judged_by_silence)
{
    harness h{};
    h.arm();

    bool fed{true};
    for (int i{0}; i < 20 && fed; ++i)
        fed = h.tick(1000, true, true, true, false, true);

    zassert_false(fed);
    zassert_true((h.st.why & wd::stuck_zcan) != 0U);
}

/* An activity sitting idle between cycles is idle, not stuck. Judging it would reset a board for
 * being quiet, which on a 5 Hz grid is most of the time. */
ZTEST(tof_task_watchdog, test_an_idle_activity_is_not_a_stuck_one)
{
    harness h{};
    h.arm();
    /* Nothing in flight anywhere, and nothing moving either. */
    bool fed{true};
    for (int i{0}; i < 10 && fed; ++i) {
        h.in.now_ms += 1000;
        ++h.in.zcan_loops;   /* only the free-running loop keeps moving */
        fed = wd::feed_allowed(h.st, h.b, h.in);
    }
    zassert_true(fed, "idle is not stuck");
}

/* Counters wrap. A wrapped counter has not gone backwards -- it has moved, which is the only thing
 * being asked -- and the millisecond clock wraps too. */
ZTEST(tof_task_watchdog, test_a_counter_wrap_is_progress_and_not_an_event)
{
    harness h{};
    h.arm();

    h.in.acquisition = {0xFFFFFFFEU, 0xFFFFFFFEU};
    h.in.can_send = {0xFFFFFFFFU, 0xFFFFFFFFU};
    h.in.zcan_loops = 0xFFFFFFFFU;
    (void)h.tick(100);

    bool fed{true};
    for (int i{0}; i < 20 && fed; ++i)
        fed = h.tick(100);

    zassert_true(fed, "counters that wrapped through zero were still making progress");
    zassert_equal(h.st.current, wd::phase::armed);
}

/* The millisecond clock wrapping must not look like a bound being exceeded. */
ZTEST(tof_task_watchdog, test_the_millisecond_clock_may_wrap_without_firing)
{
    harness h{};
    h.arm();
    h.in.now_ms = 0xFFFFF000U;
    (void)h.tick(100);

    bool fed{true};
    for (int i{0}; i < 100 && fed; ++i)
        fed = h.tick(500);

    zassert_true(fed);
}

/* THE LATCH. A board that hangs intermittently would otherwise get an unlimited number of chances
 * to look healthy between hangs, and the reset it was supposed to cause would never happen. */
ZTEST(tof_task_watchdog, test_a_stopped_feed_never_resumes_however_healthy_things_look)
{
    harness h{};
    h.arm();
    h.in.health.begun = h.in.health.ended + 1;

    bool fed{true};
    for (int i{0}; i < 20 && fed; ++i)
        fed = h.tick(1000, true, true, false, true, true);
    zassert_false(fed);

    /* Everything recovers, perfectly, forever. It changes nothing. */
    h.healthy();
    for (int i{0}; i < 50; ++i)
        zassert_false(h.tick(100), "the reset is the point");

    zassert_equal(h.st.current, wd::phase::stopped);
}

/* Nothing is judged inside the grace period, which covers the first cycle of each activity after
 * the baseline. */
ZTEST(tof_task_watchdog, test_the_grace_period_judges_nothing)
{
    harness h{};
    h.healthy();
    h.in.baseline_point = true;
    (void)h.tick(1);
    zassert_equal(h.st.current, wd::phase::armed);

    h.in.acquisition.begun = h.in.acquisition.ended + 1;
    zassert_true(h.tick(h.b.grace_ms - 10, false, false, false, false, false));
}

ZTEST(tof_task_watchdog, test_every_reason_has_its_own_name)
{
    zassert_true(strcmp(wd::reason_name(wd::stuck_cycle), "stuck_cycle") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_send), "stuck_send") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_health), "stuck_health") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_zcan), "stuck_zcan") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_l7), "stuck_l7") == 0);
    zassert_true(strcmp(wd::reason_name(wd::long_operation_over), "long_operation_over") == 0);
}
