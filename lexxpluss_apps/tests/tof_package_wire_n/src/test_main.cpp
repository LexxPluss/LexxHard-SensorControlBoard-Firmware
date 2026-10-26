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
 * Candidate N's wire layout, development version, against golden vectors from an independent
 * Python implementation plus a single-bit audit of every frame byte.
 *
 * THE CASES TO READ FIRST are the two that are about the format rather than the arithmetic:
 *
 * - test_a_diagnostic_never_touches_a_grid: an independent diagnostic must not reassemble and must
 *   not refresh a published grid's timestamp or its health. If it could, a failed acquisition would
 *   reuse the last good sequence and rewrite that grid's verdict from OK to ERROR after a consumer
 *   had already acted on it.
 * - test_one_flipped_flag_bit_cannot_turn_a_diagnostic_into_metadata: the reason sequence 0 is
 *   reserved, and the price -- one number skipped, once in the device's life. The skip is now in
 *   the issuer (tof_seq_issuer::kFirstNumber), so this suite pins the refusals and no longer
 *   describes a number the issuer would hand out.
 *
 * SCOPE: THESE EIGHT BYTES, NOTHING MORE. The behavioural rules -- any arrival order, a missing
 * frame, a conflict, and the diagnostic's isolation -- are proved in tests/tof_package_chain, which
 * decodes these frames into the REAL associator and the REAL grid codec. They are not proved here,
 * because a second set of reassembly rules written next to the assertions is not evidence about the
 * module that will actually run.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_pkg_wire_n_vectors.h"
#include "tof_package_wire_n.hpp"

using namespace lexxhard::tof_package_wire_n;

namespace {

/* A byte pattern that is not a legal frame, so "untouched" is checkable rather than assumed. */
constexpr uint8_t kPoison{0xA5};

void poison(uint8_t *buffer, size_t len)
{
    memset(buffer, kPoison, len);
}

bool all_poison(const uint8_t *buffer, size_t len)
{
    for (size_t i{0}; i < len; ++i)
        if (buffer[i] != kPoison)
            return false;
    return true;
}

data_frame poisoned_data()
{
    data_frame f{};
    f.sequence = 0xDEADBEEF;
    f.zone = 63;
    f.distance_mm = 0xBEEF;
    f.status = 0x5A;
    return f;
}

bool same_data(const data_frame &a, const data_frame &b)
{
    return a.sequence == b.sequence && a.zone == b.zone && a.distance_mm == b.distance_mm &&
           a.status == b.status;
}

health_frame poisoned_health()
{
    health_frame f{};
    f.sequence = 0xDEADBEEF;
    f.state = sensor_state::degraded;
    f.purpose = health_purpose::grid_metadata;
    f.inactive = true;
    return f;
}

bool same_health(const health_frame &a, const health_frame &b)
{
    return a.sequence == b.sequence && a.state == b.state && a.purpose == b.purpose &&
           a.inactive == b.inactive;
}

/* Frame builders. Deliberately the ONLY helpers left in this suite: the behavioural rules --
 * arrival order, a missing frame, a conflict, the diagnostic's isolation -- are proved in
 * tests/tof_package_chain against the REAL associator. An earlier version of this file proved them
 * against a collector written here, and that collector was wrong in a way the real module is not:
 * it cleared its slot on a conflict without retiring the sequence, so a full replay of the same
 * acquisition published a grid the real path refuses as stale. A second set of reassembly rules is
 * not evidence about the first. */

void build_metadata(uint32_t seq, sensor_state state, uint8_t (&frame)[kFrameBytes])
{
    health_frame h{};
    h.sequence = seq;
    h.state = state;
    h.purpose = health_purpose::grid_metadata;
    h.inactive = false;
    zassert_equal(result::ok, encode_health(h, frame, kFrameBytes));
}

void build_diagnostic(sensor_state state, bool inactive, uint8_t (&frame)[kFrameBytes])
{
    health_frame h{};
    h.sequence = 0;
    h.state = state;
    h.purpose = health_purpose::independent_diagnostic;
    h.inactive = inactive;
    zassert_equal(result::ok, encode_health(h, frame, kFrameBytes));
}

}  // namespace

ZTEST(tof_package_wire_n, test_golden_data_frames_encode)
{
    for (const auto &v : pkg_wire_n_vectors::kDataVectors) {
        data_frame in{};
        in.sequence = v.sequence;
        in.zone = v.zone;
        in.distance_mm = v.distance_mm;
        in.status = v.status;

        uint8_t frame[kFrameBytes];
        poison(frame, sizeof frame);
        zassert_equal(result::ok, encode_data(in, frame, sizeof frame), "%s", v.name);
        zassert_equal(0, memcmp(frame, v.frame, kFrameBytes), "%s", v.name);
    }
    zassert_equal(pkg_wire_n_vectors::kDataVectorCount,
                  sizeof pkg_wire_n_vectors::kDataVectors / sizeof(pkg_wire_n_vectors::data_vector));
}

ZTEST(tof_package_wire_n, test_golden_data_frames_decode)
{
    for (const auto &v : pkg_wire_n_vectors::kDataVectors) {
        data_frame out{poisoned_data()};
        zassert_equal(result::ok, decode_data(v.frame, kFrameBytes, out), "%s", v.name);
        zassert_equal(v.sequence, out.sequence, "%s", v.name);
        zassert_equal(v.zone, out.zone, "%s", v.name);
        zassert_equal(v.distance_mm, out.distance_mm, "%s", v.name);
        zassert_equal(v.status, out.status, "%s", v.name);
    }
}

ZTEST(tof_package_wire_n, test_golden_health_frames_both_ways)
{
    for (const auto &v : pkg_wire_n_vectors::kHealthVectors) {
        health_frame in{};
        in.sequence = v.sequence;
        in.state = static_cast<sensor_state>(v.state);
        in.purpose = v.purpose == 0 ? health_purpose::grid_metadata
                                    : health_purpose::independent_diagnostic;
        in.inactive = v.inactive != 0;

        uint8_t frame[kFrameBytes];
        poison(frame, sizeof frame);
        zassert_equal(result::ok, encode_health(in, frame, sizeof frame), "%s", v.name);
        zassert_equal(0, memcmp(frame, v.frame, kFrameBytes), "%s", v.name);

        health_frame out{poisoned_health()};
        zassert_equal(result::ok, decode_health(v.frame, kFrameBytes, out), "%s", v.name);
        zassert_true(same_health(in, out), "%s", v.name);
    }
}

ZTEST(tof_package_wire_n, test_golden_refusals_name_the_reason_and_write_nothing)
{
    for (const auto &v : pkg_wire_n_vectors::kDataRefusals) {
        const data_frame witness{poisoned_data()};
        data_frame out{witness};
        const result r = decode_data(v.frame, kFrameBytes, out);
        zassert_not_equal(result::ok, r, "%s", v.name);
        zassert_equal(0, strcmp(result_name(r), v.expected), "%s: got %s wanted %s", v.name,
                      result_name(r), v.expected);
        zassert_true(same_data(witness, out), "%s left out modified", v.name);
    }
    for (const auto &v : pkg_wire_n_vectors::kHealthRefusals) {
        const health_frame witness{poisoned_health()};
        health_frame out{witness};
        const result r = decode_health(v.frame, kFrameBytes, out);
        zassert_not_equal(result::ok, r, "%s", v.name);
        zassert_equal(0, strcmp(result_name(r), v.expected), "%s: got %s wanted %s", v.name,
                      result_name(r), v.expected);
        zassert_true(same_health(witness, out), "%s left out modified", v.name);
    }
}

ZTEST(tof_package_wire_n, test_every_data_bit_flip_lands_in_exactly_one_field)
{
    data_frame base{};
    base.sequence = 0x01020304;
    base.zone = 0x15;
    base.distance_mm = 0x0607;
    base.status = 0x08;

    uint8_t canonical[kFrameBytes];
    zassert_equal(result::ok, encode_data(base, canonical, sizeof canonical));

    for (size_t byte{0}; byte < kFrameBytes; ++byte) {
        for (unsigned bit{0}; bit < 8U; ++bit) {
            uint8_t frame[kFrameBytes];
            memcpy(frame, canonical, sizeof frame);
            frame[byte] = static_cast<uint8_t>(frame[byte] ^ (1U << bit));

            data_frame out{poisoned_data()};
            const result r = decode_data(frame, kFrameBytes, out);

            if (byte == kZoneOffset && bit >= 6U) {
                /* The only flips this layout refuses: a reserved bit carries no meaning yet, so a
                 * decoder that ignored it could never be given one. */
                zassert_equal(result::reserved_bits_set, r, "byte %u bit %u", (unsigned)byte, bit);
                continue;
            }
            zassert_equal(result::ok, r, "byte %u bit %u", (unsigned)byte, bit);

            const bool in_sequence = byte < kSequenceBytes;
            const bool in_zone = byte == kZoneOffset;
            const bool in_distance = byte == kDistanceOffset || byte == kDistanceOffset + 1;
            const bool in_status = byte == kStatusOffset;

            const uint32_t seq_bit = in_sequence
                                         ? (1UL << ((kSequenceBytes - 1U - byte) * 8U + bit))
                                         : 0UL;
            const uint16_t dist_bit =
                in_distance
                    ? static_cast<uint16_t>(1U << ((byte == kDistanceOffset ? 8U : 0U) + bit))
                    : 0U;

            zassert_equal(base.sequence ^ seq_bit, out.sequence, "byte %u bit %u", (unsigned)byte,
                          bit);
            zassert_equal(in_zone ? (base.zone ^ (1U << bit)) : base.zone, out.zone,
                          "byte %u bit %u", (unsigned)byte, bit);
            zassert_equal(base.distance_mm ^ dist_bit, out.distance_mm, "byte %u bit %u",
                          (unsigned)byte, bit);
            zassert_equal(in_status ? (base.status ^ (1U << bit)) : base.status, out.status,
                          "byte %u bit %u", (unsigned)byte, bit);
        }
    }
}

ZTEST(tof_package_wire_n, test_every_health_bit_flip_is_accounted_for)
{
    uint8_t canonical[kFrameBytes];
    build_metadata(0x01020304, sensor_state::degraded, canonical);

    for (size_t byte{0}; byte < kFrameBytes; ++byte) {
        for (unsigned bit{0}; bit < 8U; ++bit) {
            uint8_t frame[kFrameBytes];
            memcpy(frame, canonical, sizeof frame);
            frame[byte] = static_cast<uint8_t>(frame[byte] ^ (1U << bit));

            health_frame out{poisoned_health()};
            const result r = decode_health(frame, kFrameBytes, out);

            if (byte < kSequenceBytes) {
                /* A flipped sequence bit is undetectable here by construction: the number is the
                 * identity, and nothing in one frame can check it. That is what the 64 zones plus
                 * metadata agreeing on one number is for. */
                zassert_equal(result::ok, r, "byte %u bit %u", (unsigned)byte, bit);
                zassert_equal(health_purpose::grid_metadata, out.purpose);
                continue;
            }
            if (byte == kStateOffset) {
                /* state 1 becomes 0 when bit 0 flips, and 3 or more otherwise. */
                if (bit == 0U) {
                    zassert_equal(result::ok, r, "state bit %u", bit);
                    zassert_equal(sensor_state::ok, out.state);
                } else {
                    zassert_equal(result::state_out_of_range, r, "state bit %u", bit);
                }
                continue;
            }
            if (byte == kFlagsOffset) {
                if (bit == 0U)
                    zassert_equal(result::diagnostic_sequence_not_zero, r, "flags bit %u", bit);
                else if (bit == 1U)
                    zassert_equal(result::inactive_without_diagnostic, r, "flags bit %u", bit);
                else
                    zassert_equal(result::reserved_bits_set, r, "flags bit %u", bit);
                continue;
            }
            zassert_equal(result::reserved_bits_set, r, "byte %u bit %u", (unsigned)byte, bit);
        }
    }
}

ZTEST(tof_package_wire_n, test_one_flipped_flag_bit_cannot_turn_a_diagnostic_into_metadata)
{
    uint8_t diagnostic[kFrameBytes];
    build_diagnostic(sensor_state::error, false, diagnostic);

    /* Clearing the one bit that says "independent" leaves a frame that is metadata for the grid
     * numbered 0 -- well formed in every other respect. Reserving sequence 0 is what makes it
     * refusable, and this is the case that pins that reason. */
    uint8_t flipped[kFrameBytes];
    memcpy(flipped, diagnostic, sizeof flipped);
    flipped[kFlagsOffset] = static_cast<uint8_t>(flipped[kFlagsOffset] & ~kFlagStandalone);
    health_frame out{};
    zassert_equal(result::metadata_sequence_zero, decode_health(flipped, kFrameBytes, out));

    /* THE PRICE, AND WHERE IT IS PAID. A grid numbered 0 cannot be sent at all under this layout,
     * in any of the three frame roles below. The skip therefore lives where numbers are born --
     * tof_seq_issuer::kFirstNumber is 1 -- and these refusals are what makes a 0 arriving from
     * anywhere else (a corrupted field, a foreign sender) refusable rather than decodable. */
    data_frame first{};
    first.sequence = 0;
    first.zone = 0;
    uint8_t frame[kFrameBytes];
    zassert_equal(result::data_sequence_zero, encode_data(first, frame, sizeof frame));
    health_frame first_metadata{};
    first_metadata.sequence = 0;
    first_metadata.state = sensor_state::ok;
    first_metadata.purpose = health_purpose::grid_metadata;
    zassert_equal(result::metadata_sequence_zero,
                  encode_health(first_metadata, frame, sizeof frame));
}

ZTEST(tof_package_wire_n, test_buffer_refusals_touch_nothing)
{
    static constexpr size_t kWrongLengths[]{0, 1, kFrameBytes - 1, kFrameBytes + 1, 16};
    data_frame in{};
    in.sequence = 1;
    health_frame health{};
    health.sequence = 1;
    health.state = sensor_state::ok;
    health.purpose = health_purpose::grid_metadata;

    for (const size_t len : kWrongLengths) {
        uint8_t frame[32];
        poison(frame, sizeof frame);
        zassert_equal(result::frame_length_mismatch, encode_data(in, frame, len), "len %u",
                      (unsigned)len);
        zassert_equal(result::frame_length_mismatch, encode_health(health, frame, len), "len %u",
                      (unsigned)len);
        zassert_true(all_poison(frame, sizeof frame), "len %u wrote bytes", (unsigned)len);

        data_frame out{poisoned_data()};
        const data_frame witness{out};
        zassert_equal(result::frame_length_mismatch, decode_data(frame, len, out));
        zassert_true(same_data(witness, out));
    }

    zassert_equal(result::null_buffer, encode_data(in, nullptr, kFrameBytes));
    zassert_equal(result::null_buffer, encode_health(health, nullptr, kFrameBytes));
    data_frame out{};
    zassert_equal(result::null_buffer, decode_data(nullptr, kFrameBytes, out));
    health_frame health_out{};
    zassert_equal(result::null_buffer, decode_health(nullptr, kFrameBytes, health_out));
    /* A null with a wrong length is still a null: the buffer is checked before its size, so the
     * reason a caller is told is the one it can act on. */
    zassert_equal(result::null_buffer, decode_data(nullptr, 0, out));
}

ZTEST(tof_package_wire_n, test_reserved_bits_are_written_zero_not_left_alone)
{
    uint8_t frame[kFrameBytes];
    data_frame in{};
    in.sequence = 0xFFFFFFFF;
    in.zone = 63;
    in.distance_mm = 0xFFFF;
    in.status = 0xFF;
    poison(frame, sizeof frame);
    zassert_equal(result::ok, encode_data(in, frame, sizeof frame));
    zassert_equal(0, frame[kZoneOffset] & kZoneReservedMask);

    health_frame health{};
    health.sequence = 0xFFFFFFFF;
    health.state = sensor_state::error;
    health.purpose = health_purpose::grid_metadata;
    poison(frame, sizeof frame);
    zassert_equal(result::ok, encode_health(health, frame, sizeof frame));
    zassert_equal(0, frame[kFlagsOffset] & kFlagsReservedMask);
    zassert_equal(0, frame[kHealthReservedOffset]);
    zassert_equal(0, frame[kHealthReservedOffset + 1]);
}

ZTEST(tof_package_wire_n, test_an_out_of_range_zone_is_refused_on_encode_and_unreachable_on_decode)
{
    data_frame in{};
    in.sequence = 1;
    uint8_t frame[kFrameBytes];
    for (unsigned zone{64}; zone <= 255U; ++zone) {
        in.zone = static_cast<uint8_t>(zone);
        poison(frame, sizeof frame);
        zassert_equal(result::zone_out_of_range, encode_data(in, frame, sizeof frame), "zone %u",
                      zone);
        zassert_true(all_poison(frame, sizeof frame), "zone %u wrote bytes", zone);
    }
    /* And the other direction cannot produce one: six bits are 0..63 exactly. Asserted rather than
     * reasoned about, because "unreachable" is the kind of claim that rots. */
    for (unsigned raw{0}; raw < 256U; ++raw) {
        uint8_t probe[kFrameBytes]{0, 0, 0, 1, static_cast<uint8_t>(raw), 0, 0, 0};
        data_frame out{};
        const result r = decode_data(probe, kFrameBytes, out);
        if ((raw & kZoneReservedMask) != 0U)
            zassert_equal(result::reserved_bits_set, r, "raw %u", raw);
        else
            zassert_true(r == result::ok && out.zone < kZones, "raw %u", raw);
    }
}

ZTEST(tof_package_wire_n, test_every_distance_and_status_survives_untouched)
{
    data_frame in{};
    in.sequence = 0x89ABCDEF;
    in.zone = 41;
    for (uint32_t d{0}; d <= 0xFFFFU; ++d) {
        in.distance_mm = static_cast<uint16_t>(d);
        in.status = static_cast<uint8_t>(d & 0xFFU);
        uint8_t frame[kFrameBytes];
        zassert_equal(result::ok, encode_data(in, frame, sizeof frame), "distance %u",
                      (unsigned)d);
        data_frame out{};
        zassert_equal(result::ok, decode_data(frame, kFrameBytes, out));
        zassert_true(same_data(in, out), "distance %u", (unsigned)d);
    }
    for (unsigned s{0}; s < 256U; ++s) {
        in.distance_mm = 0xFFFF;
        in.status = static_cast<uint8_t>(s);
        uint8_t frame[kFrameBytes];
        zassert_equal(result::ok, encode_data(in, frame, sizeof frame), "status %u", s);
        data_frame out{};
        zassert_equal(result::ok, decode_data(frame, kFrameBytes, out));
        zassert_equal(s, out.status, "status %u", s);
        /* 0xFFFF alongside "valid" and alongside "no target" are both carried. This layer does not
         * reconcile a contradiction, because reconciling it would destroy the evidence. */
        zassert_equal(0xFFFFU, out.distance_mm, "status %u", s);
    }
}

ZTEST(tof_package_wire_n, test_the_frame_count_follows_from_the_layout)
{
    zassert_equal(kZones, kDataFramesPerGrid);
    zassert_equal(kZones + 1U, kFramesPerGrid);
    zassert_equal(pkg_wire_n_vectors::kFramesPerGrid, kFramesPerGrid);
    /* 65 frames of 8 bytes for 64 zones of 3 payload bytes: the full sequence in every frame costs
     * 4 of the 8, which is what candidate N buys its identity with. Stated as arithmetic rather
     * than as a bus-load claim -- the load depends on a frame rate and a bitrate this module does
     * not know. */
    zassert_equal(192U, kZones * 3U);
    zassert_equal(520U, kFramesPerGrid * kFrameBytes);
}

ZTEST(tof_package_wire_n, test_names_exist_for_every_result_and_purpose)
{
    static constexpr result kAll[]{
        result::ok,
        result::frame_length_mismatch,
        result::null_buffer,
        result::zone_out_of_range,
        result::reserved_bits_set,
        result::state_out_of_range,
        result::inactive_without_diagnostic,
        result::diagnostic_sequence_not_zero,
        result::metadata_sequence_zero,
        result::data_sequence_zero,
        result::purpose_out_of_range,
    };
    for (const result r : kAll)
        zassert_not_equal(0, strcmp(result_name(r), "unknown"), "a result has no name");
    zassert_equal(0, strcmp(purpose_name(health_purpose::grid_metadata), "grid_metadata"));
    zassert_equal(0, strcmp(purpose_name(health_purpose::independent_diagnostic),
                            "independent_diagnostic"));
}

ZTEST(tof_package_wire_n, test_an_out_of_domain_purpose_is_refused_not_normalised)
{
    /* The flag bit is set only for independent_diagnostic, so before this refusal existed EVERY
     * other value fell through to "not a diagnostic" and encoded as grid metadata -- a caller's
     * bug became a frame claiming to describe an acquisition, which is the one thing the two
     * purposes exist to keep apart. */
    for (unsigned raw{2}; raw < 256U; ++raw) {
        health_frame in{};
        in.sequence = 7;
        in.state = sensor_state::ok;
        in.purpose = static_cast<health_purpose>(raw);
        uint8_t frame[kFrameBytes];
        poison(frame, sizeof frame);
        zassert_equal(result::purpose_out_of_range, encode_health(in, frame, sizeof frame),
                      "purpose %u", raw);
        zassert_true(all_poison(frame, sizeof frame), "purpose %u wrote bytes", raw);
    }
    /* The domain is checked before everything else, so an out-of-domain purpose is reported as
     * itself rather than as whichever cross-field rule it happened to trip. */
    health_frame also_zero{};
    also_zero.sequence = 0;
    also_zero.state = sensor_state::error;
    also_zero.purpose = static_cast<health_purpose>(9);
    also_zero.inactive = true;
    uint8_t frame[kFrameBytes];
    zassert_equal(result::purpose_out_of_range, encode_health(also_zero, frame, sizeof frame));

    /* Decode cannot produce one: the purpose comes from a single bit. Asserted over every legal
     * flag byte rather than reasoned about. */
    for (unsigned flags{0}; flags < 4U; ++flags) {
        uint8_t probe[kFrameBytes]{0, 0, 0, 1, 0, static_cast<uint8_t>(flags), 0, 0};
        health_frame out{};
        const result r = decode_health(probe, kFrameBytes, out);
        if (r == result::ok)
            zassert_true(out.purpose == health_purpose::grid_metadata ||
                             out.purpose == health_purpose::independent_diagnostic,
                         "flags %u", flags);
        else
            zassert_not_equal(result::purpose_out_of_range, r, "flags %u", flags);
    }
}

ZTEST_SUITE(tof_package_wire_n, NULL, NULL, NULL, NULL, NULL);
