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
 * The Package ToF grid PAYLOAD: 64 zones of 16-bit distance and 8-bit raw status, as one flat
 * 192-byte block.
 *
 * WHY 16 AND 8, AND WHY THAT IS NOT A CHOICE. The requirements document
 * (Requirements -- Package-ToF sensing: SCB firmware + SCBDriver) specifies "fixed-width integer
 * types throughout (uint16 distances, uint8 status)" and FW-2 says firmware does not threshold or
 * map. A 12-bit distance field cannot satisfy both: it has to either refuse a value above 4094 or
 * clamp it, and both ALTER THE RAW DATA that FW-2 says firmware must not alter. So the widths here
 * come from the requirement, not from a trade-off, and a 12-bit layout would need an explicit
 * requirement exception. The 12-bit module tof_grid_l1b_codec is kept as candidate evidence for
 * what that exception would buy, and is not a compliant implementation.
 *
 * WHAT THIS MODULE REFUSES TO DECIDE, because those decisions are not approved yet:
 *
 * - NO FRAMES. It produces and consumes 192 bytes, full stop. "32 data frames" is a CONDITIONAL
 *   consequence of a 6-byte payload per frame (192 / 6), and the per-frame header -- how much of
 *   the 8 bytes identity takes, and therefore how many frames a grid needs -- is undecided. In
 *   particular the full 32-bit acquisition sequence the requirement asks for does not fit a 4-byte
 *   payload alongside two zones, so the header question is open and this file must not prejudge it.
 * - NO FRAGMENT COUNT, NO METADATA ASSOCIATION. Whether a grid closes on a health frame, and how a
 *   metadata frame is bound to the data frames, belongs to the contract.
 * - NO VALIDITY JUDGEMENT. 0xFFFF is carried verbatim: it is not treated as invalid, not clamped,
 *   and it does not change the status byte. WHICH CASES PRODUCE 0xFFFF is an upstream conversion
 *   rule (no target detected, status not trustworthy, no data -- three different things that must
 *   not be merged) and is deliberately absent here.
 * - NO STATUS INTERPRETATION. All 256 values travel untouched. Per ST UM3038, 0 means "data not
 *   updated" and 255 means "no target detected"; collapsing invalid states into 0 would destroy the
 *   consumer's only discriminator, so this module does not look at the value at all.
 *
 * BYTE ORDER IS A CANDIDATE CONVENTION, NOT A SETTLED FORMAT. The requirement does not specify it.
 * This module fixes one -- distance most significant byte first, then the status byte, zones
 * ascending -- and pins it with golden vectors so that a change is loud rather than silent. It is
 * chosen to match the existing contract's big-endian 12-bit packing (tof_grid_packer.cpp, "the
 * zcan_uss.hpp scheme") because a second byte order on one bus is a defect waiting for whoever
 * writes the second decoder. Saying the wire format is decided would be wrong: only this one field
 * ordering is, and only as a proposal.
 *
 * NOTHING CAN BE OUT OF DOMAIN. Every uint16 is a representable distance and every uint8 a
 * representable status, so pack() has no content-based refusal -- only a buffer-size one. That is
 * the whole point of the 16-bit field, and it is why this module is shorter than the 12-bit one.
 *
 * Pure: no Zephyr, no vendor headers, no allocation. Tested on native_sim against vectors from an
 * independent implementation.
 */

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_package_grid {

inline constexpr size_t kZones{64};
inline constexpr size_t kDistanceBytes{2};
inline constexpr size_t kStatusBytes{1};
inline constexpr size_t kBytesPerZone{kDistanceBytes + kStatusBytes};  // 3
/* 192. The one number this module exists to establish. */
inline constexpr size_t kPayloadBytes{kZones * kBytesPerZone};

struct grid {
    /* Raw, as measured. Not reduced, not clamped, not validity-checked. */
    uint16_t distance_mm[kZones]{};
    /* Raw ULD per-zone status. Never rewritten. */
    uint8_t status[kZones]{};
};

enum class result : uint8_t {
    ok,
    /* The only failure mode. Not exactly kPayloadBytes bytes were supplied, so the request is
     * refused before anything is written -- a short buffer must not decode to a grid whose tail
     * zones read 0 mm with status 0, which an upstream rule could not tell from a measurement. */
    payload_length_mismatch,
};

/* Writes exactly kPayloadBytes. On failure nothing is written. */
result pack(const grid &in, uint8_t *payload, size_t payload_len);

/* Decodes exactly kPayloadBytes. On failure `out` is untouched. */
result unpack(const uint8_t *payload, size_t payload_len, grid &out);

const char *result_name(result r);

}
