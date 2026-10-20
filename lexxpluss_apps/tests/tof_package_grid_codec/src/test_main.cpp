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

/*
 * Tests for the Package ToF grid payload codec.
 *
 * Two jobs. One: prove the codec carries 64 zones of uint16 distance and uint8 status verbatim,
 * with every field isolated from every other. Two: establish the ONE number this module exists for
 * -- the payload is 192 bytes -- without letting any frame-level consequence creep in as a claim.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_package_grid_codec.hpp"

#include "tof_pkg_grid_payload_vectors.h"

using namespace lexxhard::tof_package_grid;

namespace {

grid filler_grid()
{
    grid g{};
    for (size_t z{0}; z < kZones; ++z) {
        g.distance_mm[z] = static_cast<uint16_t>(z * 719 + 13);
        g.status[z] = static_cast<uint8_t>(z * 13 + 3);
    }
    return g;
}

bool grids_equal(const grid &a, const grid &b)
{
    for (size_t z{0}; z < kZones; ++z) {
        if (a.distance_mm[z] != b.distance_mm[z] || a.status[z] != b.status[z])
            return false;
    }
    return true;
}

}  // namespace

/* THE CLAIM THIS MODULE MAKES, and the one it refuses to make.
 *
 * Makes: 64 zones at 3 bytes is a 192-byte payload, with no padding anywhere.
 * Refuses: anything about frames. The 32-data-frame figure is computed below purely to record that
 * it is CONDITIONAL on a 6-byte payload per frame -- a premise that belongs to the header decision,
 * which is not made, and which this module contains no constant for. */
ZTEST(tof_package_grid_codec, test_the_payload_is_192_bytes)
{
    zassert_equal(kZones, 64U);
    zassert_equal(kDistanceBytes, 2U);
    zassert_equal(kStatusBytes, 1U);
    zassert_equal(kBytesPerZone, 3U);
    zassert_equal(kPayloadBytes, 192U, "payload is %zu bytes, not 192", kPayloadBytes);
    zassert_equal(kPayloadBytes, kZones * kBytesPerZone);

    /* No padding: the per-zone size divides the payload exactly, which is why this codec has no
     * reserved field to verify on the way back in. */
    zassert_equal(kPayloadBytes % kBytesPerZone, 0U);

    /* Conditional, and stated as such. IF a frame carried 6 payload bytes THEN a grid would need
     * 32 of them -- and 3 divides 6, so no zone would span a frame. Neither 6 nor 32 appears
     * anywhere in the module under test. */
    constexpr size_t kHypotheticalPayloadPerFrame{6};
    zassert_equal(kPayloadBytes / kHypotheticalPayloadPerFrame, 32U);
    zassert_equal(kPayloadBytes % kHypotheticalPayloadPerFrame, 0U);
    zassert_equal(kHypotheticalPayloadPerFrame % kBytesPerZone, 0U);

    /* The generator derived the same three numbers from a different implementation. */
    zassert_equal(kZones, pkg_grid_vectors::kZones);
    zassert_equal(kBytesPerZone, pkg_grid_vectors::kBytesPerZone);
    zassert_equal(kPayloadBytes, pkg_grid_vectors::kPayloadBytes);
}

ZTEST(tof_package_grid_codec, test_golden_vectors_pack)
{
    for (size_t v{0}; v < pkg_grid_vectors::kVectorCount; ++v) {
        const auto &vec{pkg_grid_vectors::kVectors[v]};
        grid in{};
        for (size_t z{0}; z < kZones; ++z) {
            in.distance_mm[z] = vec.distance_mm[z];
            in.status[z] = vec.status[z];
        }
        uint8_t payload[kPayloadBytes];
        memset(payload, 0xA5, sizeof payload);
        zassert_equal(pack(in, payload, sizeof payload), result::ok, "%s", vec.name);
        for (size_t i{0}; i < kPayloadBytes; ++i) {
            zassert_equal(payload[i], vec.payload[i], "%s byte %zu: got 0x%02x want 0x%02x",
                          vec.name, i, payload[i], vec.payload[i]);
        }
    }
}

ZTEST(tof_package_grid_codec, test_golden_vectors_unpack)
{
    for (size_t v{0}; v < pkg_grid_vectors::kVectorCount; ++v) {
        const auto &vec{pkg_grid_vectors::kVectors[v]};
        grid out{};
        zassert_equal(unpack(vec.payload, kPayloadBytes, out), result::ok, "%s", vec.name);
        for (size_t z{0}; z < kZones; ++z) {
            zassert_equal(out.distance_mm[z], vec.distance_mm[z], "%s zone %zu distance",
                          vec.name, z);
            zassert_equal(out.status[z], vec.status[z], "%s zone %zu status", vec.name, z);
        }
    }
}

/* THE BYTE ORDER, pinned explicitly rather than only through the vectors, so that a reader sees
 * what the convention IS and a change fails with an obvious message.
 *
 * It is a CANDIDATE: the requirements document does not specify a byte order. What is required is
 * only the field widths. */
ZTEST(tof_package_grid_codec, test_byte_order_is_the_candidate_convention)
{
    grid in{};
    in.distance_mm[0] = 0x1234;
    in.status[0] = 0x5A;
    in.distance_mm[1] = 0xFFFE;
    in.status[1] = 0x00;

    uint8_t payload[kPayloadBytes];
    zassert_equal(pack(in, payload, sizeof payload), result::ok);

    /* Distance most significant byte first, then the status byte. */
    zassert_equal(payload[0], 0x12, "distance high byte");
    zassert_equal(payload[1], 0x34, "distance low byte");
    zassert_equal(payload[2], 0x5A, "status byte");
    zassert_equal(payload[3], 0xFF);
    zassert_equal(payload[4], 0xFE);
    zassert_equal(payload[5], 0x00);

    /* Zones ascend: zone z starts at 3z. */
    for (size_t z{0}; z < kZones; ++z)
        zassert_equal(payload[z * kBytesPerZone + 2], in.status[z], "zone %zu status offset", z);
}

/* Every distance a uint16 can hold, carried verbatim. There is no out-of-domain value and no
 * clamping: that is what the 16-bit field is for. */
ZTEST(tof_package_grid_codec, test_every_distance_value_survives)
{
    for (unsigned d{0}; d <= 0xFFFFU; ++d) {
        grid in{filler_grid()};
        for (size_t z{0}; z < kZones; ++z)
            in.distance_mm[z] = static_cast<uint16_t>(d);

        uint8_t payload[kPayloadBytes];
        zassert_equal(pack(in, payload, sizeof payload), result::ok);
        grid out{};
        zassert_equal(unpack(payload, sizeof payload, out), result::ok);
        zassert_true(grids_equal(in, out), "distance 0x%04x did not survive", d);
    }
}

ZTEST(tof_package_grid_codec, test_every_status_value_survives)
{
    for (unsigned s{0}; s <= 0xFFU; ++s) {
        grid in{filler_grid()};
        for (size_t z{0}; z < kZones; ++z)
            in.status[z] = static_cast<uint8_t>(s);

        uint8_t payload[kPayloadBytes];
        zassert_equal(pack(in, payload, sizeof payload), result::ok);
        grid out{};
        zassert_equal(unpack(payload, sizeof payload, out), result::ok);
        zassert_true(grids_equal(in, out), "status 0x%02x did not survive", s);
    }
}

/* 0xFFFF IS DATA HERE, NOT A VERDICT. It is not clamped, not read as invalid, and it does not touch
 * the status byte. Which cases produce it is an upstream conversion rule that this module must not
 * anticipate -- including the combinations an upstream rule might forbid. */
ZTEST(tof_package_grid_codec, test_the_invalid_sentinel_is_carried_verbatim)
{
    struct pairing {
        const char *what;
        uint16_t distance;
        uint8_t status;
    };
    /* The last two are deliberately contradictory in the upstream sense. A codec that "fixed"
     * either one would be making the judgement FW-2 says firmware does not make. */
    static constexpr pairing kPairs[]{
        {"no target detected", 0xFFFF, 255},
        {"sentinel with data-not-updated", 0xFFFF, 0},
        {"sentinel with a valid status", 0xFFFF, 5},
        {"zero distance with no target", 0x0000, 255},
    };

    /* Zone 0 and 63 are the payload's first and last three bytes; 1 and 31 are interior. */
    static constexpr size_t kZonesUnderTest[]{0, 1, 31, 63};

    for (const auto &p : kPairs) {
        for (size_t z : kZonesUnderTest) {
            grid in{filler_grid()};
            in.distance_mm[z] = p.distance;
            in.status[z] = p.status;

            uint8_t payload[kPayloadBytes];
            zassert_equal(pack(in, payload, sizeof payload), result::ok);
            grid out{};
            zassert_equal(unpack(payload, sizeof payload, out), result::ok);
            zassert_equal(out.distance_mm[z], p.distance, "%s at zone %zu", p.what, z);
            zassert_equal(out.status[z], p.status, "%s at zone %zu changed the status", p.what, z);
            zassert_true(grids_equal(in, out), "%s at zone %zu disturbed another field", p.what, z);
        }
    }
}

/* FIELD ISOLATION, both ways. Writing a zone's distance must not touch any status, writing a
 * status must not touch any distance, and neither must touch a neighbouring zone -- the failure an
 * off-by-one byte offset produces. */
ZTEST(tof_package_grid_codec, test_distance_and_status_are_isolated)
{
    const grid base{filler_grid()};

    for (size_t z{0}; z < kZones; ++z) {
        /* Distance only. */
        grid in{base};
        in.distance_mm[z] = 0xBEEF;
        uint8_t payload[kPayloadBytes];
        zassert_equal(pack(in, payload, sizeof payload), result::ok);
        grid out{};
        zassert_equal(unpack(payload, sizeof payload, out), result::ok);
        for (size_t k{0}; k < kZones; ++k) {
            zassert_equal(out.status[k], base.status[k],
                          "distance at zone %zu changed status at zone %zu", z, k);
            zassert_equal(out.distance_mm[k], k == z ? 0xBEEF : base.distance_mm[k],
                          "distance at zone %zu leaked into zone %zu", z, k);
        }

        /* Status only. */
        in = base;
        in.status[z] = 0xA7;
        zassert_equal(pack(in, payload, sizeof payload), result::ok);
        zassert_equal(unpack(payload, sizeof payload, out), result::ok);
        for (size_t k{0}; k < kZones; ++k) {
            zassert_equal(out.distance_mm[k], base.distance_mm[k],
                          "status at zone %zu changed distance at zone %zu", z, k);
            zassert_equal(out.status[k], k == z ? 0xA7 : base.status[k],
                          "status at zone %zu leaked into zone %zu", z, k);
        }
    }
}

/* A single wrong bit must land in exactly one field of exactly one zone. This is the strongest
 * statement available about the layout without enumerating it. */
ZTEST(tof_package_grid_codec, test_one_flipped_bit_changes_exactly_one_field)
{
    const grid in{filler_grid()};
    uint8_t reference[kPayloadBytes];
    zassert_equal(pack(in, reference, sizeof reference), result::ok);

    for (size_t bit{0}; bit < kPayloadBytes * 8U; ++bit) {
        uint8_t payload[kPayloadBytes];
        memcpy(payload, reference, sizeof payload);
        payload[bit >> 3] ^= static_cast<uint8_t>(1U << (bit & 7U));

        grid out{};
        zassert_equal(unpack(payload, sizeof payload, out), result::ok, "bit %zu", bit);

        size_t changed_distance{0};
        size_t changed_status{0};
        size_t which{kZones};
        for (size_t z{0}; z < kZones; ++z) {
            if (out.distance_mm[z] != in.distance_mm[z]) {
                ++changed_distance;
                which = z;
            }
            if (out.status[z] != in.status[z]) {
                ++changed_status;
                which = z;
            }
        }
        zassert_equal(changed_distance + changed_status, 1U,
                      "bit %zu changed %zu distances and %zu statuses", bit, changed_distance,
                      changed_status);
        zassert_equal(which, (bit / 8U) / kBytesPerZone, "bit %zu landed in zone %zu", bit, which);

        /* And the field it landed in is the one the byte offset predicts. */
        const size_t byte_in_zone{(bit / 8U) % kBytesPerZone};
        if (byte_in_zone == 2U)
            zassert_equal(changed_status, 1U, "bit %zu is a status byte", bit);
        else
            zassert_equal(changed_distance, 1U, "bit %zu is a distance byte", bit);
    }
}

/* A buffer of the wrong size is refused, and refusing writes nothing. A short payload that decoded
 * would give the tail zones 0 mm with status 0 -- which no upstream rule could tell from a
 * measurement, because 0 mm is a representable distance and 0 is a defined status. */
ZTEST(tof_package_grid_codec, test_wrong_length_buffers_are_refused_and_write_nothing)
{
    const grid in{filler_grid()};
    uint8_t full[kPayloadBytes];
    zassert_equal(pack(in, full, sizeof full), result::ok);

    /* The witness the output is compared against. EVERY zone, both fields -- an earlier version of
     * this test checked only zone 0, which would have passed a refusal that wrote 63 zones and
     * left the first alone. "Writes nothing" is a statement about the whole grid or it is not a
     * statement. */
    grid witness{filler_grid()};
    witness.distance_mm[0] = 0x0123;
    witness.status[0] = 0x45;
    witness.distance_mm[kZones - 1] = 0xFEDC;
    witness.status[kZones - 1] = 0xBA;

    static constexpr size_t kLengths[]{0, 1, 3, 6, 96, 191, 193, 384};
    for (size_t len : kLengths) {
        grid out{witness};
        zassert_equal(unpack(full, len, out), result::payload_length_mismatch,
                      "length %zu decoded", len);
        zassert_true(grids_equal(out, witness), "length %zu wrote into out", len);
    }

    /* A null payload is the same refusal even when the length is right: a decoder must not
     * dereference it. */
    grid null_out{witness};
    zassert_equal(unpack(nullptr, kPayloadBytes, null_out), result::payload_length_mismatch);
    zassert_true(grids_equal(null_out, witness), "a null payload wrote into out");

    /* pack refuses the same way, and a short buffer is never partly filled. */
    uint8_t shortbuf[kPayloadBytes - 1];
    memset(shortbuf, 0xA5, sizeof shortbuf);
    zassert_equal(pack(in, shortbuf, sizeof shortbuf), result::payload_length_mismatch);
    for (size_t i{0}; i < sizeof shortbuf; ++i)
        zassert_equal(shortbuf[i], 0xA5, "byte %zu written on refusal", i);

    uint8_t longbuf[kPayloadBytes + 1];
    memset(longbuf, 0xA5, sizeof longbuf);
    zassert_equal(pack(in, longbuf, sizeof longbuf), result::payload_length_mismatch);
    for (size_t i{0}; i < sizeof longbuf; ++i)
        zassert_equal(longbuf[i], 0xA5, "byte %zu written on refusal", i);

    zassert_equal(pack(in, nullptr, kPayloadBytes), result::payload_length_mismatch);
}

/* There is no content-based refusal, and that is a property worth asserting: every uint16 is a
 * distance and every uint8 a status, so nothing this codec is handed can be rejected on its
 * value. A future reader looking for the 12-bit module's distance_out_of_domain should find this
 * test instead of silence. */
ZTEST(tof_package_grid_codec, test_no_value_is_ever_refused)
{
    static constexpr uint16_t kDistances[]{0x0000, 0x0FFF, 0x1000, 0x7FFF, 0x8000, 0xFFFE, 0xFFFF};
    static constexpr uint8_t kStatuses[]{0x00, 0x05, 0x7F, 0x80, 0xFE, 0xFF};

    for (uint16_t d : kDistances) {
        for (uint8_t s : kStatuses) {
            grid in{};
            for (size_t z{0}; z < kZones; ++z) {
                in.distance_mm[z] = d;
                in.status[z] = s;
            }
            uint8_t payload[kPayloadBytes];
            zassert_equal(pack(in, payload, sizeof payload), result::ok,
                          "distance 0x%04x status 0x%02x refused", d, s);
            grid out{};
            zassert_equal(unpack(payload, sizeof payload, out), result::ok);
            zassert_true(grids_equal(in, out));
        }
    }
}

ZTEST(tof_package_grid_codec, test_result_names)
{
    zassert_equal(0, strcmp(result_name(result::ok), "ok"));
    zassert_equal(0, strcmp(result_name(result::payload_length_mismatch),
                            "payload_length_mismatch"));
}

ZTEST_SUITE(tof_package_grid_codec, NULL, NULL, NULL, NULL, NULL);
