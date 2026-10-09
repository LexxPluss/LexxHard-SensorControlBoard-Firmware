/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
 * THAT COVERS BOOT AND NOT A HOST THAT LEAVES LATER, AND THIS LAYER CANNOT COVER IT. The baseline
 * is a one-time arming condition; after it, a host that goes away used to block the senders again
 * and reset the board about ten seconds later, once per host restart, on a product whose other half
 * treats host loss as ordinary (ros_heartbeat_timeout in board_controller.cpp).
 *
 * It is not fixable here, and an earlier version of this file tried. The reason is the shape of the
 * call chain rather than a missing condition: runtime_progress::end(acquisition) is the LAST statement
 * of run_cycle(), after the hook that sends, and end(health) is after the send in the health work.
 * So a sender that cannot return freezes the CYCLE and HEALTH counters too, not just the send ones.
 * Suspending the send reasons while the host is away therefore changes nothing: stuck_cycle and
 * stuck_health latch anyway. Widening the suspension to cover them would hide a genuinely broken
 * acquisition, which is most of what this layer is for.
 *
 * THE FIX IS IN THE SEND PATH, AND IT TOOK TWO PARTS. An earlier version of this comment claimed
 * one, and it was wrong in a way worth recording rather than quietly correcting.
 *
 * The first part is a bounded send: the callback form of can_send(), or any completion wait that
 * can expire. Without it a sender parks in k_sem_take(&ctx.done, K_FOREVER), with the timeout
 * bounding only the wait for a free mailbox. That is zcan_bounded_send::send(), with all fifteen
 * senders converted to it.
 *
 * THAT ALONE WAS NOT ENOUGH, which is the correction. Bounding one send does not bound a pass: every
 * poller in zcan_main::run() drained its queue with `while (k_msgq_get(..., K_NO_WAIT) == 0)`, so
 * with nothing acknowledging and all three mailboxes occupied, each send waits out its mailbox
 * timeout before returning -EAGAIN and the drain rate falls to a few messages per second -- below
 * what the producers generate (the IMU alone runs at 40 Hz), so the queue never empties and the pass
 * never ends. A beacon at the bottom of that loop stops being updated exactly as before. Worse, and
 * independently of any watchdog: in zcan_board::poll() and zcan_actuator::poll() the transmit drain
 * runs BEFORE the receive drain that feeds the controller queues, so a pass that never finishes its
 * transmit half never consumes the host's control frames -- including after the host comes back.
 *
 * The second part is therefore a per-pass budget, zcan_poll_budget: a bounded number of messages per
 * queue per pass, so a pass completes in bounded time whether or not anything is acknowledging, and
 * the receive drains behind the transmit drains always run. Together with calling end() on a refused
 * send (see runtime_progress.hpp), host loss keeps every counter moving and is not a reset condition
 * here at all -- no gate needed anywhere, and the safe state for a missing host stays where it
 * already is, in board_controller.
 *
 * WHAT IS STILL TRUE REGARDLESS: this layer does not survive an unbounded sender, and nothing here
 * makes it do so. It judges progress; a thread parked forever inside its own work cycle has no
 * progress to judge, and no suspension set can tell that apart from a dead one. If a sender is ever
 * added that waits on completion without a timeout, the reset comes back, and it comes back as a
 * watchdog bug rather than as the send bug it is. The CMake gate in lexxpluss_apps/CMakeLists.txt
 * exists to stop that at configure time rather than on a vehicle.
 *
 * The second is bring-up work that legitimately takes far longer than a cycle. Exactly ONE such
 * operation is declared today and the comment says which rather than gesturing at a category:
 * verifying the stored L7 blob, which hashes 86 KB on the main stack before any baseline exists.
 * The other two candidates do not need declaring and are named here so nobody adds them by reflex.
 * The ULD download at open() takes about two seconds per sensor and is covered by the L7 in-flight
 * bound, which is longer than that.
 *
 * Commissioning is NOT one of them, and an earlier version of this header said it was harmless on
 * the grounds that it "blocks no thread, so the periodic work continues". That was wrong:
 * commissioning stops acquisition, and acquisition is the thread that sends 0x214-0x216 and the
 * cycle health frame. It is handled by `acquisition_expected` instead of by a declaration, because
 * a proof that fails leaves acquisition stopped by design and a declaration has no end to wait
 * for.
 *
 * A declaration suspends a FIXED, NAMED set of activities and nothing else -- this is the part an
 * earlier version got wrong. Hashing a blob says nothing about whether the CAN heartbeat is still
 * going out, and a declaration that bought thirty seconds of silence for every unrelated task would
 * be a hole shaped exactly like the hang it is meant to survive. It is also bounded BEFORE the
 * baseline as well as after it, because the one declared operation runs before the baseline.
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
    /* Not produced by feed_allowed(). The feeder raises it when the driver itself refuses the feed,
     * which ends in the same reset and would otherwise leave no record at all. It lives in this
     * enum because the tombstone carries one reason word and a reader should not need two. */
    feed_api_failed      = 1U << 12,
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

/* What a stopped acquisition suspends.
 *
 * send_acq is in here because it IS the acquisition thread: runtime_progress attributes 0x214-0x216 and
 * the cycle health frame to that slot, so an acquisition that is not running cannot be sending.
 *
 * silent_l7 is in here because a stopped acquisition is also what stops asking the L7 for grids, so
 * "it has completed nothing lately" is the expected state rather than a fault. stuck_l7 is NOT:
 * an operation that was already in flight when acquisition stopped is still in flight, and that is
 * a hang whoever asked for it. The two halves of the L7 judgement answer different questions and
 * only one of them is suspended.
 *
 * The workqueue heartbeat and the health work are NOT in here: different threads, still judged. */
inline constexpr uint32_t kAcquisitionStoppedSuspends{stuck_cycle | silent_cycle | stuck_send_acq |
                                                      silent_send_acq | silent_l7};

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

    /* IS ACQUISITION SUPPOSED TO BE RUNNING? Same shape as l7_expected, and for a sharper reason.
     *
     * Commissioning STOPS acquisition: `tof cliff prove` quiesces it and tof_acq::try_stop() joins
     * that thread, which is also the thread that sends 0x214-0x216 and the cycle health frame. So a
     * commissioning pass silences `acquisition` AND `send_acq`, and the header used to claim that
     * commissioning "blocks no thread, so the periodic work continues", which is wrong.
     *
     * Declaring the pass as a long operation does not cover it either: a proof that FAILS leaves
     * acquisition stopped by design, and then the silence never ends -- there is no end-of-operation
     * to wait for. The question the watchdog has to ask is not "has acquisition progressed" but "is
     * acquisition supposed to be progressing", which only the caller knows. */
    bool acquisition_expected{false};
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
