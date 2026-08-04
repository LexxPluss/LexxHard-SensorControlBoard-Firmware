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

#include "tof_grid_packer.hpp"

namespace lexxhard::tof_grid {

namespace {

// VL53L7CX target_status policy, per the contract: 5 is trusted, 6 and 9
// are low-confidence and configurable, everything else is untrustworthy.
bool zone_trusted(uint8_t target_status, bool accept_low_confidence)
{
    if (target_status == 5)
        return true;
    if (target_status == 6 || target_status == 9)
        return accept_low_confidence;
    return false;
}

}

completed_verified_grid completed_verified_grid::from_read(const sensor_read &read,
                                                           bool accept_low_confidence)
{
    completed_verified_grid grid;
    // The transmit obligation. A read that fails any leg produces a
    // non-admitted grid, which the packer refuses: partial or unverified
    // data cannot reach the bus through this path.
    if (!read.complete || !read.io_success || !read.model_verified || read.source_id > 1)
        return grid;

    grid.source_id = read.source_id;
    grid.generation = read.generation;
    grid.status_flags = read.recovered_flags & 0x0F;    // byte-3 low nibble only
    grid.chain_position = read.chain_position & 0x0F;   // one nibble each on the wire
    grid.boards_detected = read.boards_detected & 0x0F;
    grid.last_error = read.last_error;

    uint8_t valid{0};
    for (size_t i{0}; i < kZones; ++i) {
        if (!zone_trusted(read.target_status[i], accept_low_confidence)) {
            grid.zones[i] = kInvalidSentinel;
            continue;
        }
        grid.zones[i] = read.zones_mm[i] > kMaxValidMm ? kMaxValidMm : read.zones_mm[i];
        ++valid;
    }
    grid.valid_zone_count = valid;
    grid.is_admitted = true;
    return grid;
}

bool packer::pack(const completed_verified_grid &grid,
                  can_frame_out (&data)[kDataFrames],
                  can_frame_out &health)
{
    if (!grid.is_admitted)
        return false;

    for (size_t chunk{0}; chunk < kDataFrames; ++chunk) {
        const uint16_t *z{&grid.zones[chunk * kZonesPerFrame]};
        uint8_t *b{data[chunk].bytes};
        b[0] = grid.generation;
        b[1] = static_cast<uint8_t>(grid.source_id << 4 | chunk);
        // 12-bit big-endian packing, byte-for-byte the zcan_uss.hpp scheme.
        b[2] = static_cast<uint8_t>((z[0] & 0xFF0) >> 4);
        b[3] = static_cast<uint8_t>((z[0] & 0x00F) << 4 | (z[1] >> 8));
        b[4] = static_cast<uint8_t>(z[1] & 0x0FF);
        b[5] = static_cast<uint8_t>((z[2] & 0xFF0) >> 4);
        b[6] = static_cast<uint8_t>((z[2] & 0x00F) << 4 | (z[3] >> 8));
        b[7] = static_cast<uint8_t>(z[3] & 0x0FF);
    }

    uint8_t *h{health.bytes};
    h[0] = grid.generation;
    h[1] = static_cast<uint8_t>(grid.source_id << 4); // low nibble reserved, 0
    h[2] = grid.valid_zone_count;
    h[3] = grid.status_flags;
    h[4] = static_cast<uint8_t>(grid.boards_detected << 4 | grid.chain_position);
    h[5] = grid.last_error;
    h[6] = 0; // reserved, MUST be 0 on transmit
    h[7] = 0;
    return true;
}

}
