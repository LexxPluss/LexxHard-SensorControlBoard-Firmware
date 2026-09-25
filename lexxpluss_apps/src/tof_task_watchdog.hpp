/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Feeding the IWDG only while the work it is supposed to protect keeps finishing.
 *
 * WHAT IT REPLACES. The product feeds the hardware watchdog from a timer, unconditionally. That
 * protects against the scheduler stopping and against nothing else: every thread-level hang this
 * project has actually hit -- an I2C transaction that never completes, a cycle that never returns --
 * leaves the timer running and the board fed, alive and doing nothing, for as long as anybody is
 * willing to wait. The hang-isolation images fed on task progress instead and that is what turned
 * those hangs into resets somebody could read afterwards. This is that idea as product code.
 *
 * THE DANGER IN IT, and the reason most of this file is about not firing. A watchdog that resets a
 * healthy board is worse than one that never fires, because it takes the machine out of service and
 * hides its own cause. Two starting conditions make that easy to get wrong.
 *
 * The first is boot. A post-DFU boot legitimately blocks every CAN sender until the robot PC is
 * back on the bus, so "nothing has been sent" is the normal state of a board that is working
 * perfectly and waiting. Judging it would reset the board for the host's absence. So nothing is
 * judged until a BASELINE has been taken, and the baseline requires every watched activity to have
 * completed at least once on this boot -- not to be configured, not to be expected, to have
 * actually finished. Before that the answer is always feed.
 *
 * The second is the long operations that are part of ordinary bring-up. Downloading 86 KB of ULD
 * firmware to an L7 takes about two seconds per sensor over I2C, and commissioning takes longer
 * still. Both dwarf any per-cycle bound. They are declared rather than inferred, and the window
 * they get is bounded: a declared long operation suspends the bounds it covers and has a cap of its
 * own, because "the firmware said it was busy" is exactly the shape of excuse a hang would offer.
 *
 * WHERE THE BASELINE IS TAKEN. After automatic commissioning has finished and before the first L7
 * is opened. Earlier and the chain is still being enumerated, which is one of the long operations;
 * later and the first L7 open -- the single operation most likely to hang, and the one this whole
 * investigation started from -- would fall outside the watched window.
 *
 * IT LATCHES. Once this decides to stop feeding, this boot never feeds again, whatever the counters
 * do afterwards. A watchdog that can change its mind gives a board that hangs intermittently an
 * indefinite number of chances to look healthy between hangs, and the reset it was supposed to
 * cause never happens. The reset is the point.
 *
 * WHAT IT DOES NOT DO. It does not know which subsystem matters more, and it does not decide
 * whether the chain is usable -- enumeration owns that. It answers one question, every time it is
 * asked: may the watchdog be fed right now.
 */

#pragma once

#include <stdint.h>

namespace lexxhard::tof_task_watchdog {

/* Why feeding stopped. A bitmask because more than one can be true, and which ones were true is the
 * first thing anybody wants from the forensics block after an unexplained reset. */
enum reason : uint32_t {
    stuck_cycle          = 1U << 0,
    stuck_send           = 1U << 1,
    stuck_health         = 1U << 2,
    stuck_zcan           = 1U << 3,
    stuck_l7             = 1U << 4,
    long_operation_over  = 1U << 5,
};

enum class phase : uint8_t {
    waiting,   // no baseline yet. Always feeds, judges nothing
    armed,     // baseline taken, bounds apply
    stopped,   // latched. Never feeds again this boot
};

/* One watched activity. `begun != ended` means something is inside it; a hang is that condition
 * persisting while `ended` does not move. Both wrap, and every comparison here is either equality
 * or unsigned subtraction, so a wrap is not an event. */
struct progress {
    uint32_t begun{0};
    uint32_t ended{0};
};

struct bounds {
    /* After the baseline, before anything is judged. Covers the first cycle of each activity. */
    uint32_t grace_ms{2000};
    uint32_t cycle_ms{2000};
    uint32_t send_ms{2000};
    uint32_t health_ms{3000};
    uint32_t zcan_ms{2000};
    uint32_t l7_ms{5000};
    /* The cap on a declared long operation. Generous enough for two ULD downloads and a
     * commissioning pass, and finite because an unbounded declaration is not a bound. */
    uint32_t long_operation_ms{30000};
};

struct input {
    uint32_t now_ms{0};
    /* Automatic commissioning has finished and no L7 has been opened yet. The baseline is taken on
     * the first evaluation where this is true and every activity has moved. */
    bool baseline_point{false};
    /* Does this image expect an L7 to make progress? False for a build without the ULD, or with no
     * grid position, and then L7 progress is neither required nor judged. */
    bool l7_expected{false};
    progress acquisition{};
    progress can_send{};
    progress health{};
    progress l7{};
    /* Free-running, so unlike the others it is judged on movement alone: there is no "inside" a
     * zcan loop to be stuck in. */
    uint32_t zcan_loops{0};
    /* A declared long operation -- an ULD download, a commissioning pass. Suspends the bounds of
     * the activities it runs inside, and is itself bounded. */
    bool long_operation{false};
    uint32_t long_operation_began_ms{0};
};

struct state {
    phase current{phase::waiting};
    uint32_t why{0};
    /* When each activity's `ended` was last seen to move. */
    uint32_t acq_seen_ms{0};
    uint32_t send_seen_ms{0};
    uint32_t health_seen_ms{0};
    uint32_t zcan_seen_ms{0};
    uint32_t l7_seen_ms{0};
    uint32_t armed_ms{0};
    progress last_acq{};
    progress last_send{};
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
