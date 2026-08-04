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

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_contract_vectors.h"
#include "tof_grid_packer.hpp"

using namespace lexxhard::tof_grid;

// The firmware-side half of the cross-repository pin. Editing the contract
// regenerates the vectors with a new SHA and fails this assertion until the
// literal below is updated deliberately -- the same mechanism SCBDriver
// already has in test_tof_grid_assembler.cpp. Until this test existed the
// lock was one-sided.
ZTEST(tof_grid_packer, test_contract_sha_pin)
{
    zassert_equal(0, strcmp(tof_contract::kContractSha256,
        "9c09ebe2c5962c7d16a30102fd3e8c0c3f16b4259669039d9890afa882c0a37b"));
    zassert_equal(0, strcmp(tof_contract::kContractVersion, "2026-08-02e"));
    // The packer's own constants must agree with the contract's.
    zassert_equal(kInvalidSentinel, tof_contract::kInvalidSentinel);
    zassert_equal(kMaxValidMm, tof_contract::kMaxValidMm);
    zassert_equal(kZones, tof_contract::kZones);
    zassert_equal(kDataFrames, tof_contract::kChunksPerGrid);
}

namespace {

// Builds the acquisition-side input that must reproduce a golden grid
// vector: wire zones equal to 4095 come from untrusted target status (the
// raw distance is deliberately garbage to prove it is ignored), everything
// else is a trusted status-5 reading.
sensor_read read_for_vector(const tof_contract::GridVector &vector)
{
    sensor_read read{};
    read.complete = true;
    read.io_success = true;
    read.model_verified = true;
    read.source_id = vector.source_id;
    read.generation = vector.generation;
    read.recovered_flags = vector.health_frame.bytes[3];
    read.boards_detected = vector.health_frame.bytes[4] >> 4;
    read.chain_position = vector.health_frame.bytes[4] & 0x0F;
    read.last_error = vector.health_frame.bytes[5];
    for (size_t i{0}; i < kZones; ++i) {
        if (vector.zones_mm[i] == tof_contract::kInvalidSentinel) {
            read.target_status[i] = 0;
            read.zones_mm[i] = 1234;
        } else {
            read.target_status[i] = 5;
            read.zones_mm[i] = vector.zones_mm[i];
        }
    }
    return read;
}

sensor_read minimal_good_read()
{
    sensor_read read{};
    read.complete = true;
    read.io_success = true;
    read.model_verified = true;
    read.source_id = 0;
    read.generation = 1;
    for (size_t i{0}; i < kZones; ++i) {
        read.target_status[i] = 5;
        read.zones_mm[i] = 100;
    }
    return read;
}

}

ZTEST_SUITE(tof_grid_packer, NULL, NULL, NULL, NULL, NULL);

// Byte-exact against all four golden grid vectors: 16 data frames plus the
// closing health frame, nothing derived from this repository's prose.
ZTEST(tof_grid_packer, test_golden_grid_vectors_byte_exact)
{
    for (const auto &vector : tof_contract::kGridVectors) {
        auto const grid{completed_verified_grid::from_read(read_for_vector(vector), false)};
        zassert_true(grid.admitted(), "%s: not admitted", vector.name);
        zassert_equal(grid.valid_zones(), vector.expected_valid_zone_count,
                      "%s: valid_zone_count", vector.name);
        can_frame_out data[kDataFrames];
        can_frame_out health;
        zassert_true(packer::pack(grid, data, health), "%s: pack refused", vector.name);
        for (size_t i{0}; i < kDataFrames; ++i)
            zassert_equal(0, memcmp(data[i].bytes, vector.data_frames[i].bytes, 8),
                          "%s: data frame %u", vector.name, static_cast<unsigned>(i));
        zassert_equal(0, memcmp(health.bytes, vector.health_frame.bytes, 8),
                      "%s: health frame", vector.name);
    }
}

// The transmit obligation, leg by leg: any failed leg yields a
// non-admitted grid, and the packer writes nothing for it.
ZTEST(tof_grid_packer, test_gate_refuses_each_failed_leg)
{
    for (int leg{0}; leg < 4; ++leg) {
        sensor_read read{minimal_good_read()};
        if (leg == 0)
            read.complete = false;
        else if (leg == 1)
            read.io_success = false;
        else if (leg == 2)
            read.model_verified = false;
        else
            read.source_id = 2; // unrepresentable on the wire
        auto const grid{completed_verified_grid::from_read(read, false)};
        zassert_false(grid.admitted(), "leg %d admitted", leg);

        can_frame_out data[kDataFrames];
        can_frame_out health;
        memset(data, 0xA5, sizeof data);
        memset(&health, 0xA5, sizeof health);
        zassert_false(packer::pack(grid, data, health), "leg %d packed", leg);
        zassert_equal(data[0].bytes[0], 0xA5, "leg %d wrote data", leg);
        zassert_equal(health.bytes[0], 0xA5, "leg %d wrote health", leg);
    }
}

// Distance semantics: a trusted long reading clamps to 4094 and stays a
// reading; only untrusted status makes the 0xFFF sentinel. A trusted zone
// must never round up into the sentinel.
ZTEST(tof_grid_packer, test_clamp_is_not_the_sentinel)
{
    sensor_read read{minimal_good_read()};
    read.zones_mm[0] = 4095;
    read.zones_mm[1] = 60000;
    read.zones_mm[2] = 4094;
    read.zones_mm[3] = 0; // zero is a valid reading, not a sentinel
    auto const grid{completed_verified_grid::from_read(read, false)};
    zassert_true(grid.admitted());
    zassert_equal(grid.wire_zones()[0], kMaxValidMm);
    zassert_equal(grid.wire_zones()[1], kMaxValidMm);
    zassert_equal(grid.wire_zones()[2], kMaxValidMm);
    zassert_equal(grid.wire_zones()[3], 0);
    zassert_equal(grid.valid_zones(), 64);
}

// target_status policy: 5 always trusted; 6 and 9 only under the
// low-confidence policy; anything else is the sentinel either way.
ZTEST(tof_grid_packer, test_low_confidence_policy)
{
    sensor_read read{minimal_good_read()};
    read.target_status[0] = 6;
    read.target_status[1] = 9;
    read.target_status[2] = 4;
    read.target_status[3] = 0;

    auto const strict{completed_verified_grid::from_read(read, false)};
    zassert_equal(strict.wire_zones()[0], kInvalidSentinel);
    zassert_equal(strict.wire_zones()[1], kInvalidSentinel);
    zassert_equal(strict.wire_zones()[2], kInvalidSentinel);
    zassert_equal(strict.wire_zones()[3], kInvalidSentinel);
    zassert_equal(strict.valid_zones(), 60);

    auto const lenient{completed_verified_grid::from_read(read, true)};
    zassert_equal(lenient.wire_zones()[0], 100);
    zassert_equal(lenient.wire_zones()[1], 100);
    zassert_equal(lenient.wire_zones()[2], kInvalidSentinel);
    zassert_equal(lenient.wire_zones()[3], kInvalidSentinel);
    zassert_equal(lenient.valid_zones(), 62);
}

// Wire hygiene the contract insists on: byte-1 low nibble and bytes 6/7 of
// the health frame are reserved zeros, flags fit in the byte-3 low nibble
// even if the caller hands over garbage in the high bits.
ZTEST(tof_grid_packer, test_health_reserved_fields_and_flag_mask)
{
    sensor_read read{minimal_good_read()};
    read.recovered_flags = 0xFF;
    read.chain_position = 0xF2;  // only the low nibble may survive
    read.boards_detected = 0xF6;
    auto const grid{completed_verified_grid::from_read(read, false)};
    can_frame_out data[kDataFrames];
    can_frame_out health;
    zassert_true(packer::pack(grid, data, health));
    zassert_equal(health.bytes[1] & 0x0F, 0);
    zassert_equal(health.bytes[3], 0x0F);
    zassert_equal(health.bytes[4], 0x62);
    zassert_equal(health.bytes[6], 0);
    zassert_equal(health.bytes[7], 0);
}
