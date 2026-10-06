/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * A CAN transmit that cannot wait forever.
 *
 * WHAT WAS WRONG. Every sender in this application called can_send() with a null callback and a
 * 100 ms timeout, and the obvious reading of that call is wrong in a way that matters. Zephyr
 * implements the null-callback form as api->send(...) followed by k_sem_take(&ctx.done, K_FOREVER)
 * (drivers/can/can_common.c), and the timeout is handed to api->send, where the bxCAN driver spends
 * it waiting for a free TX mailbox and nothing else. So the 100 ms bounds getting INTO a mailbox,
 * and waiting for the frame to be acknowledged on the wire is unbounded.
 *
 * That is not a theoretical gap. bxCAN retransmits automatically until a frame is acknowledged, so
 * a bus with no other node awake -- a host that has been switched off, a harness pulled for
 * service, a controller rebooting -- never completes a single transmission. The three mailboxes
 * fill, and every telemetry thread that sends stops where it stands, with no timeout to recover on.
 * The board looks alive and publishes nothing. Worse, the threads are stopped INSIDE their own work
 * cycles, so a watchdog that judges progress sees the cycle as hung and resets a board whose only
 * fault is that its host went away for a moment.
 *
 * WHAT THIS DOES. It passes a callback, which makes can_send() return as soon as the frame is in a
 * hardware mailbox, and keeps the same timeout for the wait to get there. A vanished host now makes
 * sends fail with -EAGAIN once the mailboxes are full, promptly and forever, instead of blocking.
 * Nothing waits on the wire.
 *
 * WHAT IT COSTS, stated rather than discovered later. The return value changes meaning: it was "the
 * frame was acknowledged by some node", it is now "the frame was accepted by the controller". A
 * caller that wants delivery evidence cannot read it off this return value any more, and the three
 * counters below are where that evidence lives instead -- a bus with nobody listening shows a
 * rising `failed` while `queued` keeps up, which the old form could only express by hanging. Twelve
 * of the fifteen converted call sites discarded the return value entirely, so for those this gives
 * up nothing that was being used.
 *
 * The frame may be referenced only until can_send() returns; the bxCAN driver writes it into the
 * mailbox registers inside api->send, so the callers' stack frames remain safe.
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include <stdint.h>

namespace lexxhard::zcan_bounded_send {

/* Queue one frame for transmission and return without waiting for the wire.
 *
 * Returns 0 when the controller accepted the frame, -EAGAIN when no mailbox came free within the
 * timeout, or whatever the driver reports for a bus that is down or stopped. It never blocks for
 * longer than the timeout. */
int send(const device *dev, const can_frame *frame, k_timeout_t timeout);

/* Transmit outcomes since boot. Saturating rather than wrapping, because these are read to answer
 * "is anything getting through" and a wrap reads as health. */
struct counts {
    uint32_t queued{0};     // accepted by the controller
    uint32_t refused{0};    // never reached a mailbox: -EAGAIN, bus off, not started
    uint32_t completed{0};  // acknowledged on the wire
    uint32_t failed{0};     // the controller gave up on it
};

counts snapshot();

/* Only for tests, which need each case to start from zero. */
void reset_counts();

}  // namespace lexxhard::zcan_bounded_send
