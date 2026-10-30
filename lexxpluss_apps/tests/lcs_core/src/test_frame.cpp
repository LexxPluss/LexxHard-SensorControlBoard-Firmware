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

#include <array>
#include <cstddef>
#include <cstdint>

#include <zephyr/ztest.h>

#include "lcs_core_test_support.hpp"
#include "lcs_crc16.hpp"
#include "lcs_frame.hpp"
#include "lcs_timing.hpp"

using namespace lcs_test;
namespace lcs = lexxhard::lcs;

ZTEST_SUITE(lcs_core_frame, NULL, NULL, NULL, NULL, NULL);

/* Golden frames computed once by an independent script. */
static constexpr std::array<uint8_t, 12> kGoldenUplink0{0x4B, 0x0C, 0x00, 0xC0, 0x01, 0x02,
                                                        0x03, 0xFF, 0xFE, 0xFD, 0xF0, 0x42};
static constexpr std::array<uint8_t, 12> kGoldenUplink1{0x4B, 0x0C, 0x01, 0x82, 0x7F, 0xFF,
                                                        0xFF, 0x80, 0x00, 0x00, 0xFB, 0xB7};
static constexpr std::array<uint8_t, 8> kGoldenDownlink0{0x4B, 0x08, 0xFF, 0x08, 0x12, 0x34, 0x54, 0x4B};
static constexpr std::array<uint8_t, 8> kGoldenDownlink1{0x4B, 0x08, 0x01, 0x10, 0x00, 0x00, 0x90, 0x58};

template <size_t N>
static Bytes to_bytes(const std::array<uint8_t, N> &array)
{
    Bytes bytes;
    for (const uint8_t byte : array) {
        bytes.push(byte);
    }
    return bytes;
}

ZTEST(lcs_core_frame, test_lcs_core_c1_crc_check_value)
{
    const std::array<uint8_t, 9> text{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    zassert_equal(lcs::crc16(text.data(), text.size()), 0x29B1);
    zassert_equal(ref_crc16(to_bytes(text), text.size()), 0x29B1, "the test CRC must agree with the check value");
}

ZTEST(lcs_core_frame, test_lcs_core_c2_crc_empty)
{
    const std::array<uint8_t, 1> none{0};
    zassert_equal(lcs::crc16(none.data(), 0), 0xFFFF);
}

ZTEST(lcs_core_frame, test_lcs_core_c3_golden_uplink)
{
    zassert_equal(lcs::encode_uplink(uplink_of(0, 0xC0, 0x010203, 0xFFFEFD)), kGoldenUplink0);
    zassert_equal(lcs::encode_uplink(uplink_of(1, 0x82, 0x7FFFFF, 0x800000)), kGoldenUplink1);

    Rig rig;
    rig.feed(to_bytes(kGoldenUplink0), 0);
    rig.expect_last(uplink_of(0, 0xC0, 0x010203, 0xFFFEFD), 1);
    rig.feed(to_bytes(kGoldenUplink1), 0);
    rig.expect_last(uplink_of(1, 0x82, 0x7FFFFF, 0x800000), 2);
    check_stats(rig.rx.stats(), Expect{.delivered = 2});
}

ZTEST(lcs_core_frame, test_lcs_core_c3_golden_downlink)
{
    zassert_equal(lcs::encode_downlink(0xFF, 0x08, 0x1234), kGoldenDownlink0);
    zassert_equal(lcs::encode_downlink(0x01, 0x10, 0x0000), kGoldenDownlink1);

    Rig rig;
    rig.feed(to_bytes(kGoldenDownlink0), 0);
    rig.feed(to_bytes(kGoldenDownlink1), 0);
    zassert_equal(rig.out_count, 0U, "a downlink is never returned");
    check_stats(rig.rx.stats(), Expect{.foreign_downlink = 2});
}

ZTEST(lcs_core_frame, test_lcs_core_c4_crc_big_endian)
{
    zassert_equal(kGoldenUplink0[10], 0xF0);
    zassert_equal(kGoldenUplink0[11], 0x42);
    zassert_equal(lcs::crc16(kGoldenUplink0.data(), 10), 0xF042);
    zassert_equal(kGoldenDownlink1[6], 0x90);
    zassert_equal(kGoldenDownlink1[7], 0x58);
    zassert_equal(lcs::crc16(kGoldenDownlink1.data(), 6), 0x9058);
}

ZTEST(lcs_core_frame, test_lcs_core_c5_len_flip_breaks_crc)
{
    for (int bit{0}; bit < 8; ++bit) {
        std::array<uint8_t, 12> frame{kGoldenUplink0};
        frame[1] = static_cast<uint8_t>(frame[1] ^ (1U << bit));
        zassert_not_equal(lcs::crc16(frame.data(), 10), 0xF042, "LEN is inside the CRC range, bit %d", bit);
    }
}

ZTEST(lcs_core_frame, test_lcs_core_c6_bit_flip_every_position)
{
    for (size_t position{0}; position < kGoldenUplink0.size(); ++position) {
        for (int bit{0}; bit < 8; ++bit) {
            std::array<uint8_t, 12> frame{kGoldenUplink0};
            frame[position] = static_cast<uint8_t>(frame[position] ^ (1U << bit));
            const uint16_t stored{static_cast<uint16_t>((frame[10] << 8) | frame[11])};
            zassert_not_equal(lcs::crc16(frame.data(), 10), stored, "position %u bit %d",
                              static_cast<unsigned>(position), bit);
        }
    }
}

ZTEST(lcs_core_frame, test_lcs_core_c7_channel_raw_24bit)
{
    static constexpr std::array<uint32_t, 4> kValues{0x000000, 0x7FFFFF, 0x800000, 0xFFFFFF};
    for (const uint32_t ch1 : kValues) {
        for (const uint32_t ch2 : kValues) {
            Rig rig;
            rig.feed(uplink_frame(0, 0xC0, ch1, ch2), 0);
            zassert_equal(rig.out_count, 1U);
            const Uplink &got{rig.out[0]};
            const uint32_t got1{(static_cast<uint32_t>(got.ch1[0]) << 16) |
                                (static_cast<uint32_t>(got.ch1[1]) << 8) | got.ch1[2]};
            const uint32_t got2{(static_cast<uint32_t>(got.ch2[0]) << 16) |
                                (static_cast<uint32_t>(got.ch2[1]) << 8) | got.ch2[2]};
            zassert_equal(got1, ch1, "ch1 raw 24 bits");
            zassert_equal(got2, ch2, "ch2 raw 24 bits");
        }
    }
}

ZTEST(lcs_core_frame, test_lcs_core_deadline_matches_oracle)
{
    zassert_equal(oracle_deadline_ticks(2), 7U);
    zassert_equal(oracle_deadline_ticks(8), 9U);
    zassert_equal(oracle_deadline_ticks(12), 11U);
    for (uint32_t len{2}; len <= 12; ++len) {
        zassert_equal(lcs::deadline_ticks(len), oracle_deadline_ticks(len), "len %u", static_cast<unsigned>(len));
    }
    zassert_equal(lcs::len_deadline_ticks(13), oracle_deadline_ticks(12), "a LEN above kMaxLen is capped");
    zassert_equal(lcs::len_deadline_ticks(255), oracle_deadline_ticks(12), "a LEN above kMaxLen is capped");
}
