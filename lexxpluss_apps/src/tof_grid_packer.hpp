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

// ToF grid CAN packer (AMRSW-2322). Pure logic, no Zephyr dependencies:
// tested on native_sim against the golden vectors in docs/can/, whose test
// pins the contract SHA-256. See docs/can/tof_can_wire_contract.md -- the
// wire format is owned by that document, not by this file.
//
// Split of obligations, deliberately reflected in the types:
//
// - completed_verified_grid::from_read() is the CONTROLLER's transmit
//   obligation ("a generation may only be transmitted after one complete,
//   successful, model-verified 64-zone read"). Nothing else can mint an
//   admitted grid: every mutating path is private.
// - packer::pack() proves BYTE FORMAT only, and refuses a grid that was
//   not admitted. It cannot re-check the obligation -- by the time bytes
//   are made, that proof is carried by the type.

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_grid {

inline constexpr size_t kZones{64};
inline constexpr size_t kDataFrames{16};
inline constexpr size_t kZonesPerFrame{4};
inline constexpr uint16_t kInvalidSentinel{0xFFF};
inline constexpr uint16_t kMaxValidMm{4094};

// One raw acquisition attempt, as the acquisition thread hands it over.
// Flags are asserted by the caller; from_read() only judges them.
struct sensor_read {
    bool complete;          // all 64 zones present in this read
    bool io_success;        // no I2C error anywhere in the transaction set
    bool model_verified;    // device id verified at this chain position
    uint8_t source_id;      // 0 or 1, per the contract mapping
    uint8_t generation;
    uint8_t chain_position;   // diagnostics only, low nibble on the wire
    uint8_t boards_detected;  // diagnostics only, high nibble on the wire
    uint8_t recovered_flags;  // contract byte-3 flags: recovered/chain-level only
    uint8_t last_error;       // device/driver specific, 0 = none
    uint16_t zones_mm[kZones];      // raw ULD distances
    uint8_t target_status[kZones];  // raw ULD per-zone status
};

class completed_verified_grid {
public:
    // The gate. Returns a non-admitted grid unless the read satisfies the
    // transmit obligation (complete && io_success && model_verified) and
    // carries a representable source_id. Zone reduction to the wire
    // domain happens here: target_status 5 is trusted, 6 and 9 only when
    // accept_low_confidence, everything else becomes the invalid
    // sentinel; valid distances clamp to 4094 mm.
    static completed_verified_grid from_read(const sensor_read &read,
                                             bool accept_low_confidence);

    bool admitted() const { return is_admitted; }
    uint8_t source() const { return source_id; }
    uint8_t gen() const { return generation; }
    uint8_t valid_zones() const { return valid_zone_count; }
    const uint16_t (&wire_zones() const)[kZones] { return zones; }

private:
    completed_verified_grid() = default;
    friend class packer;

    bool is_admitted{false};
    uint8_t source_id{0};
    uint8_t generation{0};
    uint8_t valid_zone_count{0};
    uint8_t status_flags{0};
    uint8_t chain_position{0};
    uint8_t boards_detected{0};
    uint8_t last_error{0};
    uint16_t zones[kZones]{};
};

struct can_frame_out {
    uint8_t bytes[8]; // DLC is always 8, both frame types
};

class packer {
public:
    // 16 data frames plus the health frame that closes the grid, exactly
    // as the contract lays them out. Returns false -- and writes nothing
    // -- for a grid that was not admitted.
    static bool pack(const completed_verified_grid &grid,
                     can_frame_out (&data)[kDataFrames],
                     can_frame_out &health);
};

}
