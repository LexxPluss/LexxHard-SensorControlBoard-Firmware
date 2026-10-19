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
 * WHAT THIS DOES, AND ONLY THIS. It passes a callback, which makes can_send() return as soon as the
 * frame is in a hardware mailbox, and keeps the same timeout for the wait to get there. A vanished
 * host now makes sends fail with -EAGAIN once the mailboxes are full, promptly and repeatedly,
 * instead of blocking. What is removed is the UNBOUNDED WAIT FOR ACKNOWLEDGEMENT, and that is the
 * whole claim.
 *
 * It is NOT "the call cannot exceed the timeout", and the reason is NOT the one first written here.
 * can_stm32_bxcan.c takes its own k_mutex_lock(&data->inst_mutex, K_FOREVER) inside send, so a
 * caller can wait on that mutex with no timeout of its own -- but a sender waiting for
 * acknowledgement never held it: the driver unlocks it immediately before returning, and the ACK
 * wait happens above the driver, in can_common.c. An earlier version of this comment claimed the
 * mutex holders used to park forever and are now bounded. They never parked; the mutex is a
 * separate unbounded wait that this change neither causes nor removes, and holds across it are
 * short. The claim this commit is entitled to is exactly one sentence long: THE UNBOUNDED WAIT FOR
 * ACKNOWLEDGEMENT IS GONE. Anyone needing a hard upper bound on the whole call still has that mutex
 * to account for.
 *
 * WHAT IT COSTS, stated rather than discovered later. The return value changes meaning: it was "the
 * frame was acknowledged by some node", it is now "the frame was accepted by the controller". A
 * caller that wants delivery evidence cannot read it off this return value any more. Twelve of the
 * fifteen converted call sites discarded the return value entirely, so for those this gives up
 * nothing that was being used.
 *
 * WHAT A SILENT BUS ACTUALLY LOOKS LIKE in the counters, because the plausible guess is wrong: NOT
 * a rising `failed`. With nothing acknowledging, bxCAN keeps retransmitting and the three mailboxes
 * stay occupied, so no completion callback ever runs -- `completed` and `failed` both stay where
 * they were. What moves is `queued`, which stops at three, and `refused`, which rises on every
 * attempt after that. So the signature of a bus with nobody listening is REFUSED RISING WHILE
 * QUEUED IS FROZEN. `failed` rises only when the controller itself gives up on a frame, which is a
 * different fault. A reader looking only at `failed` would call a dead bus healthy.
 *
 * AND A CALLER MUST NOT COMMIT DELIVERY STATE ON A ZERO. This is a live hazard, not a caution:
 * tof_grid_publisher clears a source's pending recovery flags and its last_error only once every
 * frame of that grid returned 0, which under the old form meant the health frame had been
 * acknowledged. Under this one it means the frames were accepted, so a grid that is queued and then
 * fails asynchronously loses those flags with nothing to recover them from -- the counters here are
 * global and cannot say which source or which generation it was. That publisher is on the
 * production wiring branch, not this one, and correcting it is part of integrating the two.
 *
 * The frame may be referenced only until can_send() returns; the bxCAN driver writes it into the
 * mailbox registers inside api->send, so the callers' stack frames remain safe.
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include <limits.h>
#include <stdint.h>

#include <zephyr/sys/atomic.h>

namespace lexxhard::zcan_bounded_send {

/* Queue one frame for transmission and return without waiting for the wire.
 *
 * Returns 0 when the controller accepted the frame, -EAGAIN when no mailbox came free within the
 * timeout, or whatever the driver reports for a bus that is down or stopped. A zero is NOT evidence
 * of delivery; see the header note on committing state. It never waits for acknowledgement, which
 * is not the same as never exceeding the timeout -- the driver's own mutex has no timeout. */
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

namespace detail {

/* THE CEILING IS INT32_MAX AND NOT UINT32_MAX, which is a correctness matter rather than a taste
 * one. atomic_t is `long`: 32 bits and SIGNED on this target. An earlier version of this counter
 * incremented `seen + 1` in that signed type and compared against
 * static_cast<atomic_val_t>(UINT32_MAX) -- which narrows to -1, so the ceiling was never reached
 * and signed overflow at INT32_MAX was the only way out, which is undefined behaviour. The
 * arithmetic below is unsigned throughout and stops at the largest value the atomic can hold
 * without the sign question arising. A counter that stops counting after two billion frames is not
 * one anybody reads for precision. */
inline constexpr uint32_t kCountCeiling{INT32_MAX};

/* Exposed so the ceiling can be tested at the boundary rather than by sending two billion frames. */
inline void saturating_bump(atomic_t &c)
{
    for (;;) {
        const atomic_val_t seen{atomic_get(&c)};
        const uint32_t now{static_cast<uint32_t>(seen)};
        if (now >= kCountCeiling)
            return;
        /* now + 1U is at most kCountCeiling, which is representable in atomic_t, so this
         * conversion is not the narrowing one that was wrong before. */
        if (atomic_cas(&c, seen, static_cast<atomic_val_t>(now + 1U)))
            return;
    }
}

}  // namespace detail

}  // namespace lexxhard::zcan_bounded_send
