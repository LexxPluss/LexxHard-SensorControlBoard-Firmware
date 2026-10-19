/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * How much work one pass of the zcan loop may do, and how long a send may wait for a mailbox.
 *
 * WHY A BUDGET EXISTS AT ALL. Every poller in zcan_main::run() drained its queue with
 * `while (k_msgq_get(..., K_NO_WAIT) == 0)`, which terminates only when the producer stops. That
 * is fine while the robot PC is answering, because each send completes in microseconds and the
 * queue empties faster than it fills. It is not fine when the host goes away: the three bxCAN
 * mailboxes stay occupied, every further send waits out its full timeout before returning
 * -EAGAIN, and the drain rate collapses to a handful of messages per second. The IMU alone
 * produces at 40 Hz, so the queue never empties and the loop never finishes a pass.
 *
 * TWO THINGS BREAK WHEN A PASS NEVER FINISHES, and the second one is the worse of the two.
 *
 *   - Nothing downstream of the stuck poller runs. In zcan_board::poll() the telemetry drain
 *     comes first and the RX drain that feeds can_controller::msgq_control comes after it, and
 *     zcan_actuator::poll() has the same shape. So a host that goes away stops the board from
 *     consuming the host's control frames -- including after it comes back. That is not a
 *     recoverable state, and no watchdog is involved in it.
 *   - Any progress beacon placed in zcan_main::run() stops being updated, so a watchdog that
 *     judges liveness from it resets a board whose only fault is that its host rebooted. Host
 *     loss is an ordinary condition elsewhere in this application (board_controller's
 *     ros_heartbeat_timeout owns the safe state for it) and is not a fault a reset can fix.
 *
 * WHAT THE BUDGET GUARANTEES. A pass processes at most kTxPerPass messages per transmit queue
 * and kRxPerPass per receive queue, so the work in a pass no longer depends on how fast any
 * producer runs, and a pass therefore completes whether or not the host is answering. Across
 * the eleven pollers there are fourteen send sites, which puts the mailbox waiting in a pass
 * where every send is refused at about 14 ms. That is the waiting alone and not a bound on the
 * pass: scheduling, the per-frame work and the driver's own K_FOREVER mutex sit on top of it, so
 * read 14 ms as the order of magnitude rather than a guarantee. Even padded generously it stays
 * far inside the watchdog's silence judgements, which are seconds, and short enough that the RX
 * drains sitting behind the transmit drains still run on every pass.
 *
 * WHY kTxPerPass IS ONE. The loop sleeps a microsecond between passes, so one message per queue
 * per pass is still thousands per second on a healthy bus -- far above any producer here, the
 * fastest of which is the IMU at 40 Hz. Raising it buys no throughput that is wanted and costs
 * proportionally more worst-case pass time. The receive side gets a larger budget because its
 * work is local and cheap (copy a frame, hand it to a controller queue) and falling behind on
 * control frames is worse than spending a few more microseconds.
 *
 * WHAT A REFUSED SEND MEANS FOR A CALLER, since the budget makes refusals ordinary rather than
 * exceptional: it means this pass did not deliver that frame, not that the work failed. Periodic
 * telemetry drops it and sends fresher data next pass. A request/response path must not drop it
 * -- see zcan_dfu and zcan_actuator_service, which hold the unsent response and retry it on a
 * later pass, because a response that is dropped is never regenerated. Code that reports work
 * progress must report the work as finished either way: a send that returned -EAGAIN has
 * returned, and treating it as still in flight is what makes a quiet bus look like a hung thread.
 */

#pragma once

#include <zephyr/kernel.h>

namespace lexxhard::zcan_poll_budget {

/* Messages taken from one transmit queue in one pass of zcan_main::run(). */
inline constexpr int kTxPerPass{1};

/* Frames taken from one receive queue in one pass. Larger than the transmit budget because the
 * work is local: nothing here waits on the bus. */
inline constexpr int kRxPerPass{8};

/* How long a send may wait for a free TX mailbox before giving up with -EAGAIN. Three mailboxes
 * and a 1 Mbit/s bus put a frame on the wire in roughly 60-130 us, so this is several frame
 * times: long enough that a momentarily busy controller is not treated as a refusal, short
 * enough that a bus with nobody acknowledging cannot stretch a pass.
 *
 * It bounds only the wait for a mailbox. It is not an upper bound on the whole call: the bxCAN
 * driver takes its own k_mutex_lock(&data->inst_mutex, K_FOREVER) inside send. Holds across it
 * are short, but anyone needing a hard bound on the call still has that mutex to account for. */
inline constexpr k_timeout_t kMailboxWait{K_MSEC(1)};

}  // namespace lexxhard::zcan_poll_budget

// vim: set expandtab shiftwidth=4:
