/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Feeding the hardware watchdog on task progress instead of from a timer interrupt.
 *
 * Much of this suite is about NOT firing, which is the right proportion: a watchdog that resets a
 * healthy board takes the machine out of service and hides its own cause, and the two easiest ways
 * to build one -- a boot whose host has not arrived, and bring-up's long operations -- are starting
 * conditions rather than exotic faults.
 *
 * The cases that MUST fire are the four this design was corrected for. A first L7 open that never
 * returns, when no L7 has ever completed anything and the naive baseline could not even be reached.
 * One CAN sender wedging while the other keeps running, which a summed counter hides completely. A
 * periodic task dying cleanly between two cycles, which an in-flight test cannot see. And a task
 * going silent during a declared long operation that had nothing to do with it.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_task_watchdog.hpp"

namespace
{

namespace wd = lexxhard::tof_task_watchdog;

/* Which activities a tick advances. Named rather than positional because the interesting scenarios
 * are all "everything except one". */
struct advance {
    bool acq{true};
    bool send_acq{true};
    bool send_workq{true};
    bool health{true};
    bool zcan{true};
    bool l7{true};
};

struct harness {
    wd::state st{};
    wd::bounds b{};
    wd::input in{};

    /* Everything finished, nothing in flight: a board between cycles. The L7 is deliberately at
     * zero, because the baseline happens before the first open. */
    void healthy_pre_l7()
    {
        in.acquisition = {10, 10};
        in.send_acq = {20, 20};
        in.send_workq = {7, 7};
        in.health = {5, 5};
        in.l7 = {0, 0};
        in.zcan_loops = 1000;
    }

    bool tick(uint32_t ms, advance a = {})
    {
        in.now_ms += ms;
        if (a.acq)        { ++in.acquisition.begun; ++in.acquisition.ended; }
        if (a.send_acq)   { ++in.send_acq.begun; ++in.send_acq.ended; }
        if (a.send_workq) { ++in.send_workq.begun; ++in.send_workq.ended; }
        if (a.health)     { ++in.health.begun; ++in.health.ended; }
        if (a.zcan)       { ++in.zcan_loops; }
        if (a.l7)         { ++in.l7.begun; ++in.l7.ended; }
        return wd::feed_allowed(st, b, in);
    }

    /* The product's arming point: commissioning done, no L7 opened yet. */
    void arm(bool l7_expected = true)
    {
        healthy_pre_l7();
        in.l7_expected = l7_expected;
        in.baseline_point = true;
        (void)tick(1, advance{.l7 = false});
        zassert_equal(st.current, wd::phase::armed, "the documented arming point must be reachable");
        for (int i{0}; i < 4; ++i)
            zassert_true(tick(b.grace_ms / 2, advance{.l7 = false}));
    }

    /* Runs until the feed stops or the budget runs out. */
    bool run_until_stopped(int ticks, uint32_t ms, advance a)
    {
        for (int i{0}; i < ticks; ++i)
            if (!tick(ms, a))
                return true;
        return false;
    }
};

}  // namespace

ZTEST_SUITE(tof_task_watchdog, nullptr, nullptr, nullptr, nullptr, nullptr);

/* ------------------------------------------------------ the four corrected cases ------ */

/* THE ARMING POINT MUST BE REACHABLE, which the first version of this module made impossible: it
 * required an L7 completion for the baseline while documenting that the baseline is taken before
 * the first L7 open. */
ZTEST(tof_task_watchdog, test_the_baseline_is_reached_before_any_l7_has_ever_completed)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.l7_expected = true;
    h.in.baseline_point = true;

    (void)h.tick(1, advance{.l7 = false});

    zassert_equal(h.st.current, wd::phase::armed);
    zassert_false(h.st.l7_watching, "nothing has opened an L7 yet, so nothing is being watched yet");
}

/* THE FIRST L7 OPEN NEVER RETURNS. The monitor starts when the open begins, so this is a fault even
 * though the L7 has completed nothing and never will. It is the single operation this whole
 * investigation started from. */
ZTEST(tof_task_watchdog, test_a_first_l7_open_that_never_returns_stops_the_feed)
{
    harness h{};
    h.arm();

    h.in.l7.begun = 1;          /* the open starts, and that is the last anybody hears of it */
    (void)h.tick(1, advance{.l7 = false});
    zassert_true(h.st.l7_watching, "the monitor starts at the first open, not at the baseline");

    zassert_true(h.run_until_stopped(30, 1000, advance{.l7 = false}));
    zassert_true((h.st.why & wd::stuck_l7) != 0U);
    zassert_equal(h.st.why & wd::silent_l7, 0U,
                  "an operation that never had a chance to complete is stuck, not silent");
}

/* ONE CAN SENDER WEDGES WHILE THE OTHER RUNS. With a single summed counter the live heartbeat
 * refreshes the wedged sender's timestamp forever and this is invisible. The diagnostic images
 * split the two slots for exactly this reason. */
ZTEST(tof_task_watchdog, test_a_wedged_send_slot_is_not_hidden_by_the_other_one_running)
{
    harness h{};
    h.arm();

    /* The acquisition sender goes in and does not come out; the workqueue heartbeat keeps going. */
    h.in.send_acq.begun = h.in.send_acq.ended + 1;

    zassert_true(h.run_until_stopped(30, 1000, advance{.send_acq = false, .l7 = false}));
    zassert_true((h.st.why & (wd::stuck_send_acq | wd::silent_send_acq)) != 0U);
    zassert_equal(h.st.why & (wd::stuck_send_workq | wd::silent_send_workq), 0U,
                  "the healthy slot must not be blamed");
}

/* A PERIODIC TASK DIES BETWEEN CYCLES. begun == ended, so nothing is in flight and an in-flight test
 * sees an activity that looks exactly like one that is idle. It can stay dead forever. */
ZTEST(tof_task_watchdog, test_a_periodic_task_that_stops_being_scheduled_is_caught)
{
    harness h{};
    h.arm();
    /* Nothing in flight anywhere. Acquisition simply never runs again. */
    zassert_true(h.run_until_stopped(30, 1000, advance{.acq = false, .l7 = false}));
    zassert_true((h.st.why & wd::silent_cycle) != 0U);
    zassert_equal(h.st.why & wd::stuck_cycle, 0U, "nothing was inside it; it stopped running");
}

ZTEST(tof_task_watchdog, test_a_health_work_item_that_stops_being_scheduled_is_caught)
{
    harness h{};
    h.arm();
    zassert_true(h.run_until_stopped(30, 1000, advance{.health = false, .l7 = false}));
    zassert_true((h.st.why & wd::silent_health) != 0U);
}

/* A DECLARED LONG OPERATION SUSPENDS A FIXED SET AND NOTHING ELSE. An ULD download holds the chain;
 * it says nothing about the zcan loop, and a declaration that bought thirty seconds of silence for
 * unrelated tasks would be a hole shaped like the hang it is meant to survive. */
ZTEST(tof_task_watchdog, test_a_long_operation_does_not_excuse_the_tasks_it_never_touched)
{
    harness h{};
    h.arm();

    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;

    zassert_true(h.run_until_stopped(20, 1000, advance{.zcan = false, .l7 = false}),
                 "zcan silence is not covered by a chain operation");
    zassert_true((h.st.why & wd::silent_zcan) != 0U);
    zassert_equal(h.st.why & wd::long_operation_over, 0U, "the cap had not been reached");
}

ZTEST(tof_task_watchdog, test_a_long_operation_does_not_excuse_a_dead_heartbeat)
{
    harness h{};
    h.arm();
    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;

    zassert_true(h.run_until_stopped(20, 1000, advance{.send_workq = false, .l7 = false}));
    zassert_true((h.st.why & wd::silent_send_workq) != 0U);
}

/* What it DOES excuse: the chain work it actually holds. */
ZTEST(tof_task_watchdog, test_a_long_operation_excuses_the_chain_work_it_holds)
{
    harness h{};
    h.arm();

    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;
    h.in.l7.begun = 1;                                   /* the ULD download, legitimately slow */
    h.in.acquisition.begun = h.in.acquisition.ended + 1;  /* the cycle waiting on the chain */

    for (int i{0}; i < 8; ++i)
        zassert_true(h.tick(1000, advance{.acq = false, .l7 = false}),
                     "acquisition and the L7 are exactly what a chain operation holds");
    zassert_equal(h.st.current, wd::phase::armed);
}

/* And the declaration is itself bounded. */
ZTEST(tof_task_watchdog, test_a_long_operation_that_never_ends_is_its_own_reason_to_stop)
{
    harness h{};
    h.arm();

    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;
    h.in.l7.begun = 1;

    zassert_true(h.run_until_stopped(60, 1000, advance{.acq = false, .l7 = false}));
    zassert_true((h.st.why & wd::long_operation_over) != 0U,
                 "an unbounded declaration is not a bound");
}

/* THE ONE THING JUDGED BEFORE THE BASELINE. The declared operation that exists today -- verifying
 * the stored L7 blob -- runs in this phase, so a phase that judged nothing at all would feed a
 * wedged blob verification forever. That is the exact hang this subsystem exists to catch, and it
 * was the one place it could not. */
ZTEST(tof_task_watchdog, test_a_long_operation_that_wedges_before_the_baseline_still_stops_the_feed)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.baseline_point = false;          /* commissioning has not finished; nothing periodic yet */
    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;

    zassert_true(h.run_until_stopped(60, 1000, advance{.l7 = false}));
    zassert_equal(h.st.current, wd::phase::stopped);
    zassert_true((h.st.why & wd::long_operation_over) != 0U);
}

/* And the ordinary heartbeats are still not judged there, because nothing periodic is expected
 * until commissioning has installed a mapping. */
ZTEST(tof_task_watchdog, test_ordinary_activity_is_still_unjudged_before_the_baseline)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.baseline_point = false;
    h.in.acquisition.begun = h.in.acquisition.ended + 1;   /* inside a cycle, and staying there */

    for (int i{0}; i < 60; ++i)
        zassert_true(h.tick(1000, advance{.acq = false, .l7 = false}),
                     "a chain that has not been commissioned is not a chain that is stuck");
    zassert_equal(h.st.current, wd::phase::waiting);
}

/* ------------------------------------------------------------- not firing ------------- */

/* THE BOOT THIS MUST NOT RESET. The robot PC is not on the bus, so nothing has ever been sent and
 * nothing will be until it arrives. That is a working board waiting. */
ZTEST(tof_task_watchdog, test_a_board_whose_host_never_arrives_is_never_judged)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.send_acq = {0, 0};
    h.in.send_workq = {0, 0};
    h.in.baseline_point = true;

    for (int i{0}; i < 200; ++i)
        zassert_true(h.tick(1000, advance{.send_acq = false, .send_workq = false, .l7 = false}),
                     "a missing host is not a hang");

    zassert_equal(h.st.current, wd::phase::waiting);
    zassert_equal(h.st.why, 0U);
}

/* Both senders must have finished something, not just one: a baseline taken on the heartbeat alone
 * would arm while the acquisition sender had never worked. */
ZTEST(tof_task_watchdog, test_both_senders_must_have_finished_before_the_baseline)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.send_acq = {0, 0};
    h.in.baseline_point = true;

    (void)h.tick(1, advance{.send_acq = false, .l7 = false});
    zassert_equal(h.st.current, wd::phase::waiting);

    h.in.send_acq = {1, 1};
    (void)h.tick(1, advance{.send_acq = false, .l7 = false});
    zassert_equal(h.st.current, wd::phase::armed);
}

/* A baseline taken mid-cycle would make the first judgement after the grace period about work that
 * began before anybody was watching. */
ZTEST(tof_task_watchdog, test_the_baseline_is_not_taken_while_something_is_in_flight)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.acquisition = {11, 10};
    h.in.baseline_point = true;

    (void)h.tick(1, advance{.acq = false, .l7 = false});
    zassert_equal(h.st.current, wd::phase::waiting);

    h.in.acquisition = {11, 11};
    (void)h.tick(1, advance{.acq = false, .l7 = false});
    zassert_equal(h.st.current, wd::phase::armed);
}

/* An image built without the L7 ULD must not be reset for the absence of a sensor it was never
 * built to talk to, even if something leaves a stray count behind. */
ZTEST(tof_task_watchdog, test_an_image_without_the_l7_is_never_asked_for_l7_progress)
{
    harness h{};
    h.arm(false);

    h.in.l7.begun = 1;
    for (int i{0}; i < 30; ++i)
        zassert_true(h.tick(1000, advance{.l7 = false}));

    zassert_false(h.st.l7_watching);
    zassert_equal(h.st.why & (wd::stuck_l7 | wd::silent_l7), 0U);
}

/* A healthy board, indefinitely. The suite is worth little if the ordinary case trips anything. */
ZTEST(tof_task_watchdog, test_a_working_board_is_fed_indefinitely)
{
    harness h{};
    h.arm();
    h.in.l7.begun = 1;
    h.in.l7.ended = 1;

    for (int i{0}; i < 500; ++i)
        zassert_true(h.tick(100), "nothing here is wrong");
    zassert_equal(h.st.current, wd::phase::armed);
}

/* Counters wrap, and a wrapped counter has moved rather than gone backwards. */
ZTEST(tof_task_watchdog, test_a_counter_wrap_is_progress_and_not_an_event)
{
    harness h{};
    h.arm();
    h.in.acquisition = {0xFFFFFFFEU, 0xFFFFFFFEU};
    h.in.send_acq = {0xFFFFFFFFU, 0xFFFFFFFFU};
    h.in.zcan_loops = 0xFFFFFFFFU;
    (void)h.tick(100, advance{.l7 = false});

    for (int i{0}; i < 20; ++i)
        zassert_true(h.tick(100, advance{.l7 = false}));
    zassert_equal(h.st.current, wd::phase::armed);
}

/* So does the clock. */
ZTEST(tof_task_watchdog, test_the_millisecond_clock_may_wrap_without_firing)
{
    harness h{};
    h.arm();
    h.in.now_ms = 0xFFFFF000U;
    (void)h.tick(100, advance{.l7 = false});

    for (int i{0}; i < 100; ++i)
        zassert_true(h.tick(500, advance{.l7 = false}));
}

/* THE LATCH. A board that hangs intermittently would otherwise get an unlimited number of chances
 * to look healthy between hangs, and the reset would never happen. */
ZTEST(tof_task_watchdog, test_a_stopped_feed_never_resumes_however_healthy_things_look)
{
    harness h{};
    h.arm();
    zassert_true(h.run_until_stopped(30, 1000, advance{.health = false, .l7 = false}));

    h.healthy_pre_l7();
    for (int i{0}; i < 50; ++i)
        zassert_false(h.tick(100), "the reset is the point");
    zassert_equal(h.st.current, wd::phase::stopped);
}

/* Nothing is judged inside the grace period. */
ZTEST(tof_task_watchdog, test_the_grace_period_judges_nothing)
{
    harness h{};
    h.healthy_pre_l7();
    h.in.baseline_point = true;
    (void)h.tick(1, advance{.l7 = false});
    zassert_equal(h.st.current, wd::phase::armed);

    h.in.acquisition.begun = h.in.acquisition.ended + 1;
    zassert_true(h.tick(h.b.grace_ms - 10,
                        advance{.acq = false, .send_acq = false, .send_workq = false,
                                .health = false, .zcan = false, .l7 = false}));
}

/* The suspend set is a constant so that no call site can widen it. */
ZTEST(tof_task_watchdog, test_the_long_operation_suspend_set_covers_the_chain_and_nothing_else)
{
    constexpr uint32_t expected{wd::stuck_cycle | wd::silent_cycle | wd::stuck_l7 | wd::silent_l7};
    zassert_equal(wd::kLongOperationSuspends, expected);
    zassert_equal(wd::kLongOperationSuspends & wd::silent_zcan, 0U);
    zassert_equal(wd::kLongOperationSuspends & (wd::stuck_send_acq | wd::silent_send_acq), 0U);
    zassert_equal(wd::kLongOperationSuspends & (wd::stuck_send_workq | wd::silent_send_workq), 0U);
    zassert_equal(wd::kLongOperationSuspends & (wd::stuck_health | wd::silent_health), 0U);
}

ZTEST(tof_task_watchdog, test_every_reason_has_its_own_name)
{
    zassert_true(strcmp(wd::reason_name(wd::stuck_cycle), "stuck_cycle") == 0);
    zassert_true(strcmp(wd::reason_name(wd::silent_cycle), "silent_cycle") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_send_acq), "stuck_send_acq") == 0);
    zassert_true(strcmp(wd::reason_name(wd::silent_send_acq), "silent_send_acq") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_send_workq), "stuck_send_workq") == 0);
    zassert_true(strcmp(wd::reason_name(wd::silent_send_workq), "silent_send_workq") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_health), "stuck_health") == 0);
    zassert_true(strcmp(wd::reason_name(wd::silent_health), "silent_health") == 0);
    zassert_true(strcmp(wd::reason_name(wd::stuck_l7), "stuck_l7") == 0);
    zassert_true(strcmp(wd::reason_name(wd::silent_l7), "silent_l7") == 0);
    zassert_true(strcmp(wd::reason_name(wd::silent_zcan), "silent_zcan") == 0);
    zassert_true(strcmp(wd::reason_name(wd::long_operation_over), "long_operation_over") == 0);
    zassert_true(strcmp(wd::reason_name(wd::feed_api_failed), "feed_api_failed") == 0);
}
