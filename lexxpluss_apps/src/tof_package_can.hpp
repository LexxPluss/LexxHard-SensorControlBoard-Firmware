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

/* Package CAN transport on can2. Callers supply both IDs; zero is invalid.
 * The allocation lives in tof_can_ids.hpp. zcan_main configures can2. */

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

/* Listen on both IDs for window_ms, then remove the filters. A non-zero count
 * reports traffic; the caller decides whether to transmit. Requires init/configure. */
int listen_for_traffic(int32_t window_ms, uint32_t *frames_seen);

/* Requires init/configure. Success means mailbox acceptance, not delivery;
 * transmit_errors() counts later failures. Abandoned grids may leave queued
 * fragments, but cannot complete because no later frames are built. */
tof_package_stream::sink sink();

/* Completion failures the driver has reported since init(). Zero does NOT mean every frame
 * arrived -- a frame still sitting in a mailbox has reported nothing yet. */
uint32_t transmit_errors();

}  // namespace lexxhard::tof_package_can

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO
