/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/*
 * The CAN transport for Package ToF frames: can2, two configured identifiers, and nothing else.
 *
 * NOT IN A PRODUCT IMAGE. It is compiled under ENABLE_TOF_PACKAGE_BENCH, where a human starts the
 * acquisition at a shell, and under ENABLE_TOF_PACKAGE_AUTO, where a thread does it at boot;
 * neither is set by any product configuration, so no product path transmits a Package frame. The
 * runtime answered the two things the bench could not -- its own thread with its own measured
 * stack -- and left the identifier question below exactly where it was: open.
 *
 * NO IDENTIFIER HAS A DEFAULT, AND NONE IS COMPILED IN. Nothing has been registered for this
 * stream: 0x21A and 0x21B appear in a note as candidates and are not an allocation. A number
 * written into this header would BECOME the registration by being the only one anybody could find,
 * which is the trap the consumer's router (LexxHard-SCBDriver, tof_package_can_router.hpp) is kept
 * out of for the same reason. So both identifiers arrive from the caller, zero is refused as
 * "unset" even though identifier zero is legal on a bus, and the pair is printed at the shell
 * before the first frame so that whoever runs it can check them against the registry at the moment
 * of use.
 *
 * THE LISTEN WINDOW IS EVIDENCE, NOT BOOKKEEPING. Before streaming onto a live bus, this module can
 * listen on both identifiers and report whether anything is already arriving on them. A duplicate
 * table of this robot's allocated identifiers was the obvious alternative and was rejected: it
 * would be a second copy of an allocation that lives in the zcan_* headers and the generated
 * contract, and a second copy is how the two drift -- this project has fixed that defect more than
 * once. Reading the actual bus cannot drift.
 *
 * WHAT SILENCE DOES NOT PROVE, stated here because the temptation to over-read it is obvious:
 * nothing arriving during the window does NOT mean the number is unallocated. It means nobody
 * transmitted on it while we were listening. An identifier that is allocated to a node that is
 * idle, absent, or polled only on demand is silent and still taken. The window catches the case
 * that is actively harmful -- transmitting into somebody else's live traffic -- and the registry is
 * what settles ownership.
 *
 * IT DOES NOT CONFIGURE can2. zcan_main owns the bitrate, the mode and the start; configuring it
 * again here would fight whichever ran second. Same rule as tof_cliff_can, and the same reason.
 */

#include <stdint.h>

#if defined(ENABLE_TOF_PACKAGE_BENCH) || defined(ENABLE_TOF_PACKAGE_AUTO)

#include "tof_package_stream.hpp"

namespace lexxhard::tof_package_can {

/* Which identifiers carry this stream. The width is part of the identity, not a formatting detail:
 * an 11-bit and a 29-bit frame with the same number are different identifiers, and the consumer's
 * router refuses the other width as somebody else's traffic. */
struct identifiers {
    uint32_t data_id{0};
    uint32_t health_id{0};
    bool extended{false};
};

/* Looks up can2 and checks readiness. -ENODEV if it is not ready. */
int init();

/* Validates and stores the pair. -EINVAL for a zero identifier, for two that are equal, or for a
 * number that does not fit the configured width. The refusals mirror the consumer's router so that
 * a pair one end accepts is a pair the other end accepts. */
int configure(const identifiers &ids);

identifiers current();

/* Adds a receive filter on each configured identifier, waits `window_ms`, and reports how many
 * frames arrived on either of them. Removes its filters before returning, including on failure.
 *
 * -EPERM before a successful init() and configure(). A non-zero count is NOT an error here -- it is
 * the finding, and what to do about it is the caller's decision. See the header on what silence
 * does and does not prove. */
int listen_for_traffic(int32_t window_ms, uint32_t *frames_seen);

/* The sink to hand tof_package_stream::publisher. Valid only after a successful init() and
 * configure(); its callbacks return -EPERM otherwise, rather than sending on identifier zero.
 *
 * WHAT A SUCCESSFUL SEND MEANS, because the publisher's counters inherit it: a transmit mailbox
 * accepted the frame. It does NOT mean the frame won arbitration, was acknowledged by another node,
 * or reached the bus at all. Those outcomes arrive later, in the driver's own context, and are
 * counted by transmit_errors(). So a grid this module reported as sent may still have failed on the
 * wire.
 *
 * AN ABANDONED GRID CANNOT ARRIVE COMPLETE, and the earlier wording here overstated it. What may
 * still reach the bus after the publisher gives up is the FRAGMENT it had already queued: those
 * frames were accepted and the controller will drain them. The frame that was refused is not
 * retried and no later frame of that grid is even built, so the grid is permanently short and the
 * consumer refuses it. The producer's counts are the producer's view; the consumer's are the only
 * account of what arrived. */
tof_package_stream::sink sink();

/* Completion failures the driver has reported since init(). Zero does NOT mean every frame
 * arrived -- a frame still sitting in a mailbox has reported nothing yet. */
uint32_t transmit_errors();

}  // namespace lexxhard::tof_package_can

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO
