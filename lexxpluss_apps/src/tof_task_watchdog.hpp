/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Feeding the IWDG only while the work it is supposed to protect keeps finishing.
 *
 * WHAT IT REPLACES. The product feeds the hardware watchdog from a k_timer expiry, which on Zephyr
 * runs in interrupt context, unconditionally. That protects against the scheduler stopping and
 * against nothing else: every thread-level hang this project has actually hit -- an I2C transaction
 * that never completes, a cycle that never returns -- leaves the timer running and the board fed,
 * alive and doing nothing, for as long as anybody is willing to wait.
 *
 * TWO WAYS A TASK DIES, and an earlier version of this file only caught one of them.
 *
 * It can hang INSIDE its work: `begun != ended` with `ended` frozen. That is the I2C wedge, and it
 * is what the diagnostic images were built to catch.
 *
 * Or it can stop being scheduled BETWEEN cycles, at which point `begun == ended` and the activity
 * looks exactly like one that is idle. A thread that dies cleanly between two cycles is invisible
 * to an in-flight test and can stay dead forever while the board is fed. So every activity that is
 * PERIODIC after commissioning also has a silence bound: `ended` not moving for long enough is a
 * fault whether or not anything was inside. The two bounds are separate because they mean different
 * things and a reader after a reset wants to know which one it was.
 *
 * THE DANGER, and the reason much of this file is about not firing. A watchdog that resets a
 * healthy board is worse than one that never fires, because it takes the machine out of service and
 * hides its own cause. Two starting conditions make that easy to get wrong.
 *
 * The first is boot. A post-DFU boot legitimately blocks every CAN sender until the robot PC is
 * back on the bus, so "nothing has been sent" is the normal state of a board that is working
 * perfectly and waiting. Nothing is judged until a BASELINE has been taken, and the baseline
 * requires every watched activity to have actually finished once on this boot. Before that the
 * answer is always feed.
 *
 * The second is the long operations that are part of ordinary bring-up: an ULD download is about
 * two seconds per sensor and commissioning is longer. They are declared rather than inferred, and
 * -- this is the part an earlier version got wrong -- a declaration suspends a FIXED, NAMED set of
 * activities and nothing else. Downloading firmware to an L7 says nothing about whether the CAN
 * heartbeat is still going out, and a declaration that bought thirty seconds of silence for every
 * unrelated task would be a hole shaped exactly like the hang it is meant to survive.
 *
 * THE L7 IS WATCHED FROM ITS FIRST OPEN, not from the baseline. The baseline is taken after
 * commissioning and BEFORE any L7 has been opened, so at that moment an L7 has necessarily
 * completed nothing; requiring otherwise would make the documented arming point unreachable. So the
 * L7 monitor starts when the first open BEGINS, and from that instant a first open that never
 * returns is a fault -- which matters, because that single operation is the one this whole
 * investigation started from.
 *
 * THE TWO CAN SENDERS ARE WATCHED SEPARATELY, because the board has two and they can fail
 * independently: the acquisition slot carries 0x214, 0x215, 0x216 and the cycle health frame, and
 * the system workqueue carries the 0x217 heartbeat. One summed counter would let a live heartbeat
 * refresh the timestamp of a wedged acquisition sender forever. The diagnostic images split them
 * for exactly this reason and the product logic keeps the split.
 *
 * IT LATCHES. Once this decides to stop feeding, this boot never feeds again, whatever the counters
 * do afterwards. A board that hangs intermittently would otherwise get an unlimited number of
 * chances to look healthy between hangs, and the reset it was supposed to cause never happens.
 *
 * WHAT IT DOES NOT DO. It does not decide whether the chain is usable -- enumeration owns that. It
 * answers one question, every time it is asked: may the watchdog be fed right now.
 */

#pragma once

#include <stdint.h>

namespace lexxhard::tof_task_watchdog {

/* Why feeding stopped. A bitmask because more than one can be true, and which ones were true is the
 * first thing anybody wants after an unexplained reset. `stuck_` is a task that went in and did not
 * come out; `silent_` is a periodic task that stopped running at all. */
enum reason : uint32_t {
    stuck_cycle          = 1U << 0,
    silent_cycle         = 1U << 1,
    stuck_send_acq       = 1U << 2,
    silent_send_acq      = 1U << 3,
    stuck_send_workq     = 1U << 4,
    silent_send_workq    = 1U << 5,
    stuck_health         = 1U << 6,
    silent_health        = 1U << 7,
    stuck_l7             = 1U << 8,
    silent_l7            = 1U << 9,
    silent_zcan          = 1U << 10,
    long_operation_over  = 1U << 11,
};

enum class phase : uint8_t {
    waiting,   // no baseline yet. Always feeds, judges nothing
    armed,     // baseline taken, bounds apply
    stopped,   // latched. Never feeds again this boot
};

/* One watched activity. `begun != ended` means something is inside it. Both wrap, and every
 * comparison here is either equality or unsigned subtraction, so a wrap is not an event. */
struct progress {
    uint32_t begun{0};
    uint32_t ended{0};
};

/* WHAT A DECLARED LONG OPERATION MAY SUSPEND, fixed here rather than supplied by the caller.
 *
 * An ULD download and a commissioning pass hold the chain, so they hold acquisition and the L7. They
 * do not hold the CAN heartbeat, the health work item or the zcan loop, and a caller able to say
 * otherwise would be able to buy silence for tasks the operation never touched. Making it a
 * constant means no call site can widen it. */
inline constexpr uint32_t kLongOperationSuspends{stuck_cycle | silent_cycle | stuck_l7 | silent_l7};

struct bounds {
    /* After the baseline, before anything is judged. Covers the first cycle of each activity. */
    uint32_t grace_ms{2000};
    /* In-flight: something went in and has not come out. */
    uint32_t cycle_ms{2000};
    uint32_t send_ms{2000};
    uint32_t health_ms{3000};
    uint32_t l7_ms{5000};
    /* Silence: a periodic activity has completed nothing for this long, in flight or not. Longer
     * than the in-flight bounds because a periodic task is allowed to be late before it is
     * declared dead, and because the 5 Hz grid is the slowest thing being watched. */
    uint32_t cycle_silence_ms{5000};
    uint32_t send_silence_ms{5000};
    uint32_t health_silence_ms{5000};
    uint32_t l7_silence_ms{10000};
    uint32_t zcan_silence_ms{2000};
    /* The cap on a declared long operation. Generous enough for two ULD downloads and a
     * commissioning pass, and finite because an unbounded declaration is not a bound. */
    uint32_t long_operation_ms{30000};
};

struct input {
    uint32_t now_ms{0};
    /* Automatic commissioning has finished and no L7 has been opened yet. */
    bool baseline_point{false};
    /* Does this image expect an L7 at all? False for a build without the ULD or with no grid
     * position, and then no L7 progress is ever required or judged. */
    bool l7_expected{false};
    progress acquisition{};
    /* The two senders, watched separately. `send_acq` carries 0x214/0x215/0x216 and the cycle
     * health frame from the acquisition slot; `send_workq` carries the 0x217 heartbeat from the
     * system workqueue. */
    progress send_acq{};
    progress send_workq{};
    progress health{};
    progress l7{};
    /* Free-running, so it has no inside to be stuck in: silence is its whole symptom. */
    uint32_t zcan_loops{0};
    bool long_operation{false};
    uint32_t long_operation_began_ms{0};
};

struct state {
    phase current{phase::waiting};
    uint32_t why{0};
    /* When each activity's `ended` was last seen to move. */
    uint32_t acq_seen_ms{0};
    uint32_t send_acq_seen_ms{0};
    uint32_t send_workq_seen_ms{0};
    uint32_t health_seen_ms{0};
    uint32_t zcan_seen_ms{0};
    uint32_t l7_seen_ms{0};
    uint32_t armed_ms{0};
    /* The L7 monitor starts at the first open rather than at the baseline; see the header. */
    bool l7_watching{false};
    progress last_acq{};
    progress last_send_acq{};
    progress last_send_workq{};
    progress last_health{};
    progress last_l7{};
    uint32_t last_zcan{0};
};

/* Returns whether the watchdog may be fed now, and advances `st`. Call it from a thread, never from
 * an interrupt: an ISR that can feed is an ISR that keeps a hung system alive, which is the whole
 * failure being fixed. */
bool feed_allowed(state &st, const bounds &b, const input &in);

const char *reason_name(reason r);

}  // namespace lexxhard::tof_task_watchdog
