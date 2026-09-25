/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Six heartbeats, written at the points where work actually finishes, read by the watchdog feeder.
 *
 * WHY NEW COUNTERS RATHER THAN THE EXISTING STATISTICS. The acquisition layer, the publisher and the
 * CAN sink all keep tallies already, and reusing them was the obvious move. It is also the one that
 * puts a safety decision on top of fields whose meaning is owned by somebody else: a tally that is
 * reset for a diagnostic, or counted once per frame instead of once per flush, changes what the
 * watchdog believes without anyone touching the watchdog. These six exist only for this, are written
 * nowhere else, and are never reset.
 *
 * WHAT A HEARTBEAT IS. A pair -- begun and ended -- because the two failures it has to tell apart
 * need both. A task that went in and did not come out leaves begun ahead of ended. A task that
 * stopped being scheduled leaves them equal and stops moving, which is indistinguishable from idle
 * unless you also watch the clock. The zcan loop is the exception and has one counter, because there
 * is no inside to a loop iteration to be stuck in.
 *
 * THE TWO CAN SENDERS ARE DIFFERENT SLOTS OF THE SAME FUNCTION. Everything goes through one send(),
 * and which sender it is depends on who called it: the acquisition thread carries 0x214, 0x215,
 * 0x216 and the cycle health frame, and the system work queue carries the 0x217 heartbeat. They can
 * wedge independently, and one summed counter lets a live heartbeat refresh a dead sender's
 * timestamp forever, so the slot is decided from the calling thread exactly as the diagnostic images
 * decided it.
 *
 * CONCURRENCY, STATED HONESTLY. Each counter is an atomic and every writer increments its own, so no
 * increment is lost. A read of one activity's pair is NOT atomic across both words, and this does
 * not pretend otherwise: `ended` is read before `begun`, which is the order that cannot produce the
 * impossible reading of more completions than starts. The reading it can produce is a pair that
 * looks in-flight for one sample when the work had just finished. That is harmless by construction:
 * every bound the feeder applies requires the condition to persist for seconds across many samples,
 * and the next sample sees the completion and refreshes the timestamp. A seqlock would buy exactness
 * nobody needs at the cost of a write barrier on the acquisition hot path.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_progress {

enum class activity : uint8_t {
    acquisition,   // one cycle of the ToF acquisition loop
    send_acq,      // a CAN send issued from the acquisition slot
    send_workq,    // a CAN send issued from the system work queue
    health,        // one health work item
    l7,            // one ULD operation: open, configure, start, read or stop
    count,
};

struct pair {
    uint32_t begun{0};
    uint32_t ended{0};
};

struct snapshot {
    pair at[static_cast<size_t>(activity::count)]{};
    uint32_t zcan_loops{0};
};

/* Called on entry to and exit from the work. Every call site is a real production completion point;
 * there is no path that increments `ended` without the work having returned. */
void begin(activity a);
void end(activity a);

/* The free-running loop. One counter, because an iteration has no inside. */
void zcan_tick();

/* Which of the two CAN slots the calling thread is. Exposed so the send path can be explicit about
 * it rather than hiding the rule inside begin(). */
activity current_send_slot();

snapshot read();

}  // namespace lexxhard::tof_progress
