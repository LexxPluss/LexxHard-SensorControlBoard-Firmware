/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
     * zero, because the baseline happens before the first open.
     *
     * ACQUISITION IS EXPECTED, set here rather than per case, because an ordinary board is
     * acquiring. The production feeder's atomic defaults to expected for the same reason, and the
     * cases that are ABOUT the suspension clear it explicitly. */
    void healthy_pre_l7()
    {
        in.acquisition_expected = true;
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

/* ---- commissioning stops acquisition ---- */

/* KOKO'S SECOND CASE. Commissioning quiesces acquisition, and that thread is also what sends
 * 0x214-0x216 and the cycle health frame, so a pass silences `acquisition` AND `send_acq`. The
 * header used to claim commissioning blocks no thread. */
ZTEST(tof_task_watchdog, test_a_commissioning_pass_that_stops_acquisition_does_not_reset_the_board)
{
    harness h;
    h.arm();

    h.in.acquisition_expected = false;
    for (int i = 0; i < 60; ++i) {
        zassert_true(h.tick(500, advance{.acq = false, .send_acq = false}),
                     "reset at %u ms during commissioning", h.in.now_ms);
    }
    zassert_equal(h.st.why, 0U, "");
}

/* AND A PROOF THAT FAILS LEAVES ACQUISITION STOPPED BY DESIGN, with no end-of-operation to wait
 * for. That is why this is a state input rather than a declared long operation: a declaration would
 * expire and reset a board that is behaving exactly as specified. */
ZTEST(tof_task_watchdog, test_acquisition_left_stopped_by_a_failed_proof_never_times_out)
{
    harness h;
    h.arm();

    h.in.acquisition_expected = false;
    for (int i = 0; i < 400; ++i)  // 200 s, far past every bound including long_operation_ms
        zassert_true(h.tick(500, advance{.acq = false, .send_acq = false}), "at %u ms", h.in.now_ms);
    zassert_equal(h.st.current, wd::phase::armed, "");
}

/* THE WORKQUEUE HEARTBEAT IS NOT THE ACQUISITION THREAD, so stopping acquisition does not excuse
 * it. One summed suspend set would have hidden a dead 0x217 for the length of every commissioning
 * pass. */
ZTEST(tof_task_watchdog, test_a_stopped_acquisition_does_not_excuse_the_workqueue_heartbeat)
{
    harness h;
    h.arm();

    h.in.acquisition_expected = false;
    const bool fed = h.tick(h.b.send_silence_ms + 1,
                            advance{.acq = false, .send_acq = false, .send_workq = false});
    zassert_false(fed, "a stopped acquisition excused the 0x217 heartbeat");
    zassert_true((h.st.why & wd::silent_send_workq) != 0, "why 0x%08x", h.st.why);
}

/* A STOPPED ACQUISITION ALSO STOPS ASKING THE L7 FOR GRIDS, so "it has completed nothing lately" is
 * the expected state. An earlier version left silent_l7 out of the suspend set and refused to feed
 * about ten seconds into a perfectly ordinary commissioning pass on a board whose L7 had run. */
ZTEST(tof_task_watchdog, test_a_stopped_acquisition_does_not_reset_on_l7_silence)
{
    harness h;
    h.arm();
    /* The L7 has run, so it is being watched: this is the case the suspension is for. */
    (void)h.tick(10);
    zassert_true(h.st.l7_watching, "the L7 monitor must be running for this case to mean anything");

    h.in.acquisition_expected = false;
    for (int i = 0; i < 60; ++i) {
        zassert_true(h.tick(500, advance{.acq = false, .send_acq = false, .l7 = false}),
                     "reset at %u ms on L7 silence during a commissioning pass", h.in.now_ms);
    }
}

/* BUT AN L7 OPERATION THAT WAS ALREADY IN FLIGHT IS STILL A HANG. Acquisition stopping does not
 * un-start an operation that had begun, and that is whose thread is sitting in the ULD. Only the
 * silence half is suspended; the in-flight half is not. */
ZTEST(tof_task_watchdog, test_a_stopped_acquisition_does_not_excuse_an_l7_operation_in_flight)
{
    harness h;
    h.arm();
    (void)h.tick(10);

    h.in.acquisition_expected = false;
    ++h.in.l7.begun;  // went in, never came out

    /* ONE SAMPLE FIRST, so the operation is in flight and young. The in-flight bound is measured
     * from when THIS operation began, so a case that begins one and reads the verdict in the same
     * sample proves nothing about the bound -- it used to pass only because the clock was the last
     * completion, which is the defect this timing replaced. */
    const bool fed_young =
        h.tick(1, advance{.acq = false, .send_acq = false, .l7 = false});
    zassert_true(fed_young, "an L7 operation one millisecond old was already called stuck");

    const bool fed = h.tick(h.b.l7_ms + 1, advance{.acq = false, .send_acq = false, .l7 = false});
    zassert_false(fed, "a stopped acquisition excused an L7 operation stuck in flight");
    zassert_true((h.st.why & wd::stuck_l7) != 0, "why 0x%08x", h.st.why);
}

/* ---- the sender is nested inside the things that would have excused it ---- */

/* WHY THERE IS NO "THE HOST IS AWAY" EXEMPTION, demonstrated instead of asserted.
 *
 * An earlier version of this module took a host_present input and suspended the send reasons while
 * it was false, on the stated premise that the cycle, the health work and the L7 "do not go through
 * the host". That premise is false, and this case is the proof. In tof_acquisition, progress::end
 * for the cycle is the LAST statement of run_cycle(), after the hook that sends; end for the health
 * activity is after the hook that sends the cliff health frame. A sender blocked in can_send()'s
 * K_FOREVER completion wait therefore freezes the OUTER counters as well: the cycle and the health
 * work are both in flight, under their own bounds, and they latch on their own.
 *
 * So the exemption could not have saved the board. It only removed the one reason word that would
 * have named the sender -- the single most useful thing in the tombstone. */
ZTEST(tof_task_watchdog, test_a_wedged_sender_latches_the_cycle_and_health_that_enclose_it)
{
    harness h;
    h.arm();

    /* One pass enters the cycle, enters the send, and nothing comes out of any of it again. The
     * zcan loop and the workqueue keep running: they are other threads, and their liveness is
     * exactly why this does not look like a dead board from the outside. */
    ++h.in.acquisition.begun;
    ++h.in.send_acq.begun;
    ++h.in.health.begun;

    const bool stopped = h.run_until_stopped(
        40, 250, advance{.acq = false, .send_acq = false, .health = false, .l7 = false});
    zassert_true(stopped, "a sender wedged forever never stopped the feed");

    /* BOTH of the enclosing counters, from one blocked call. The sender is the proximate cause and
     * the cycle is in flight only because it is waiting on the sender -- the tombstone says so, and
     * that pairing is what tells a reader to look at the send path rather than at acquisition.
     *
     * The health work is wedged in the same place but is NOT in this word, and the reason is worth
     * knowing: the latch records the reasons true at the moment feeding stopped, health_ms is 3000
     * against cycle_ms and send_ms at 2000, so the tighter pair trips a second earlier and the
     * phase goes to stopped before health is ever judged. A tombstone naming fewer activities than
     * are actually stuck is therefore normal; it names the fastest, not all of them. */
    zassert_true((h.st.why & wd::stuck_send_acq) != 0, "why 0x%08x", h.st.why);
    zassert_true((h.st.why & wd::stuck_cycle) != 0,
                 "the cycle encloses the send, so it is in flight too: why 0x%08x", h.st.why);
    zassert_equal(h.st.why & wd::stuck_health, 0U,
                  "health has the looser bound and must not have been reached: why 0x%08x",
                  h.st.why);
}

/* AND SUSPENDING THE ACQUISITION DOES NOT BUY SILENCE EITHER, which is the other half of the reason
 * this is not fixable by widening a suspension set. A commissioning pass legitimately clears
 * acquisition_expected, and that does hide the cycle and the sender -- but the health work is a
 * different thread, is not suspended, and its own send is wedged in the same place. Whatever is
 * widened next, the reset still happens, because the fault is an unbounded wait and not a
 * misjudgement here. The fix belongs in the senders. */
ZTEST(tof_task_watchdog, test_suspending_acquisition_does_not_hide_a_wedged_sender_elsewhere)
{
    harness h;
    h.arm();

    h.in.acquisition_expected = false;
    ++h.in.health.begun;

    const bool stopped = h.run_until_stopped(
        40, 250, advance{.acq = false, .send_acq = false, .health = false, .l7 = false});
    zassert_true(stopped, "a wedged health send was excused by an unrelated suspension");
    zassert_true((h.st.why & wd::stuck_health) != 0, "why 0x%08x", h.st.why);
    zassert_equal(h.st.why & (wd::stuck_cycle | wd::stuck_send_acq), 0U,
                  "the suspension must still cover what it covers: why 0x%08x", h.st.why);
}

/* ------------------------------------------- what a suspension and a bound measure ----- */

/* A PAUSE IS NOT A GAP THE RESUMED WORK OWNS, and it was being charged for it.
 *
 * A commissioning pass clears acquisition_expected for seconds -- longer than cycle_silence_ms,
 * which is the whole reason it has to be suspended at all. The mask hid the reasons while it ran,
 * but the clocks they are measured against kept running, so the first sample after acquisition came
 * back read "nothing has completed for eight seconds" and latched silent_cycle before the resumed
 * thread had been given a single opportunity to complete anything. A board that commissioned
 * successfully reset itself on the way back. */
ZTEST(tof_task_watchdog, test_a_resumed_acquisition_is_not_judged_on_the_pause_it_sat_out)
{
    harness h;
    h.arm();

    /* The pass: well past cycle_silence_ms with the cycle and its sender stopped, which is what
     * try_stop() leaves behind. */
    h.in.acquisition_expected = false;
    for (int i{0}; i < 8; ++i)
        zassert_true(h.tick(1000, advance{.acq = false, .send_acq = false, .l7 = false}),
                     "the suspension itself must feed");

    /* Back, and nothing has completed yet because the thread has only just been started. This is
     * the sample that used to latch. */
    h.in.acquisition_expected = true;
    zassert_true(h.tick(1, advance{.acq = false, .send_acq = false, .l7 = false}),
                 "the first sample after the pause judged the pause: why 0x%08x", h.st.why);

    /* And it stays fed while the resumed activity gets going. */
    for (int i{0}; i < 5; ++i)
        zassert_true(h.tick(100, advance{.l7 = false}), "why 0x%08x", h.st.why);
    zassert_equal(h.st.current, wd::phase::armed);
}

/* THE CONTROL, because the case above would also pass if the suspension simply disarmed the
 * watchdog. An acquisition that comes back on paper and then does nothing is still a fault, and it
 * must be caught on the resumed activity's own clock. */
ZTEST(tof_task_watchdog, test_an_acquisition_that_resumes_and_then_dies_is_still_caught)
{
    harness h;
    h.arm();

    h.in.acquisition_expected = false;
    for (int i{0}; i < 8; ++i)
        (void)h.tick(1000, advance{.acq = false, .send_acq = false, .l7 = false});

    h.in.acquisition_expected = true;
    const bool stopped = h.run_until_stopped(
        40, 500, advance{.acq = false, .send_acq = false, .l7 = false});
    zassert_true(stopped, "a resumed acquisition that completed nothing was fed forever");
    zassert_true((h.st.why & wd::silent_cycle) != 0, "why 0x%08x", h.st.why);
}

/* A LONG OPERATION LEAVES THE SAME GAP, and it is suspended by a different path. Both halves of the
 * cycle and the L7 are masked by a declaration, so both of their clocks are held. */
ZTEST(tof_task_watchdog, test_a_declared_operation_does_not_leave_a_gap_behind_it)
{
    harness h;
    h.arm();

    h.in.long_operation = true;
    h.in.long_operation_began_ms = h.in.now_ms;
    for (int i{0}; i < 8; ++i)
        zassert_true(h.tick(1000, advance{.acq = false, .l7 = false}),
                     "the declaration itself must feed: why 0x%08x", h.st.why);

    h.in.long_operation = false;
    zassert_true(h.tick(1, advance{.acq = false, .l7 = false}),
                 "the first sample after the operation judged the operation: why 0x%08x", h.st.why);
}

/* THE IN-FLIGHT BOUND BELONGS TO THE OPERATION THAT IS INSIDE, not to the gap before it.
 *
 * Both halves of the judgement used to read the same clock -- the last completion -- so an activity
 * that had legitimately completed nothing for longer than its in-flight bound declared the next
 * operation stuck the instant it began. On the sender that is not a corner case: a quiet bus
 * completes nothing for seconds, and then one frame goes out and is immediately a hang. */
ZTEST(tof_task_watchdog, test_the_in_flight_bound_measures_this_operation_not_the_gap_before_it)
{
    harness h;
    h.arm();

    /* A quiet sender: nothing completes for longer than send_ms, but still inside send_silence_ms
     * so the silence half has nothing to say yet. */
    zassert_true(h.tick(h.b.send_ms + 500, advance{.send_acq = false, .l7 = false}),
                 "why 0x%08x", h.st.why);

    /* Now one send goes in. It is one millisecond old. */
    ++h.in.send_acq.begun;
    zassert_true(h.tick(1, advance{.send_acq = false, .l7 = false}),
                 "a send one millisecond old was judged on the gap before it: why 0x%08x",
                 h.st.why);

    /* It completes, the way an ordinary send does. */
    ++h.in.send_acq.ended;
    zassert_true(h.tick(1, advance{.send_acq = false, .l7 = false}), "why 0x%08x", h.st.why);
    zassert_equal(h.st.current, wd::phase::armed);
}

/* AND THE CONTROL FOR THAT ONE: a send that really does not come out is still caught, on its own
 * clock. Without this the case above is satisfied by never judging a sender at all. */
ZTEST(tof_task_watchdog, test_a_send_that_never_returns_is_still_caught_on_its_own_clock)
{
    harness h;
    h.arm();

    ++h.in.send_acq.begun;  // in, and never out
    const bool stopped =
        h.run_until_stopped(40, 500, advance{.send_acq = false, .l7 = false});
    zassert_true(stopped, "a send that never returned was fed forever");
    zassert_true((h.st.why & wd::stuck_send_acq) != 0, "why 0x%08x", h.st.why);
}
