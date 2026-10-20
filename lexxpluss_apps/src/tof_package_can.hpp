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

/* Package CAN transport on can2 for bench or auto builds; neither is a default
 * product configuration. Supply both IDs explicitly: zero means unset and
 * 0x21A/0x21B remain candidates. A listen window rejects observed traffic;
 * silence does not establish ownership. Consult the allocation registry.
 * Do not duplicate that registry or reconfigure can2: zcan_main owns it.
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

/* The publisher sink requires successful init() and configure(), else -EPERM.
 * Success means mailbox acceptance, not arbitration, ACK or wire delivery;
 * transmit_errors() counts later completion failures.
 * Abandonment may leave queued fragments on the bus, but the refused frame is
 * not retried and no later frame is built, so that grid cannot arrive complete.
 * Only the consumer can account for complete grids received.
 */
tof_package_stream::sink sink();

/* Completion failures the driver has reported since init(). Zero does NOT mean every frame
 * arrived -- a frame still sitting in a mailbox has reported nothing yet. */
uint32_t transmit_errors();

}  // namespace lexxhard::tof_package_can

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO
