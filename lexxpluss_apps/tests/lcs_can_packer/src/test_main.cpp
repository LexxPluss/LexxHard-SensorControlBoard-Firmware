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
#include <optional>

#include <zephyr/ztest.h>

#include "lcs_can_packer.hpp"
#include "lcs_frame.hpp"

using lexxhard::lcs::CanFrame;
using lexxhard::lcs::CanPacker;
using lexxhard::lcs::Uplink;

ZTEST_SUITE(lcs_can_packer, NULL, NULL, NULL, NULL, NULL);

namespace {

using Bytes3 = std::array<uint8_t, 3>;

/* Splits a 24-bit value into its bytes, most significant first. */
Bytes3 be24(const uint32_t value)
{
    return Bytes3{static_cast<uint8_t>((value >> 16) & 0xFFU), static_cast<uint8_t>((value >> 8) & 0xFFU),
                  static_cast<uint8_t>(value & 0xFFU)};
}

Uplink make_uplink(const uint8_t sender, const uint8_t status, const uint32_t ch1, const uint32_t ch2)
{
    return Uplink{sender, status, be24(ch1), be24(ch2)};
}

/* Expected frame, byte by byte, with explicit indices and the expected ID literals. */
void expect_frame(const std::optional<CanFrame> &frame, const Uplink &in, const uint8_t counter)
{
    zassert_true(frame.has_value(), "no frame");
    const uint32_t id{in.sender_id == 0 ? 0x21CU : 0x21DU};
    zassert_equal(frame->id, id);
    zassert_equal(frame->dlc, 8);
    zassert_equal(frame->data[0], in.status);
    zassert_equal(frame->data[1], in.ch1[0]);
    zassert_equal(frame->data[2], in.ch1[1]);
    zassert_equal(frame->data[3], in.ch1[2]);
    zassert_equal(frame->data[4], in.ch2[0]);
    zassert_equal(frame->data[5], in.ch2[1]);
    zassert_equal(frame->data[6], in.ch2[2]);
    zassert_equal(frame->data[7], counter);
}

Uplink plain(const uint8_t sender)
{
    return make_uplink(sender, 0x00, 0x112233, 0x445566);
}

/* Packs and discards n frames of a board. */
void burn(CanPacker &packer, const uint8_t sender, const unsigned n)
{
    for (unsigned i = 0; i < n; ++i)
        zassert_true(packer.pack(plain(sender)).has_value());
}

struct Rng {
    uint32_t state;

    uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
};

}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk1_pk2_known_senders_map_to_ids)
{
    CanPacker packer;
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    expect_frame(packer.pack(plain(1)), plain(1), 0);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk3_unknown_senders_get_no_frame)
{
    CanPacker packer;
    for (const uint8_t sender : {uint8_t{2}, uint8_t{0x7F}, uint8_t{0xFF}})
        zassert_false(packer.pack(plain(sender)).has_value());
}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk4_dlc_is_8_in_every_case)
{
    CanPacker packer;
    for (unsigned i = 0; i < 300; ++i) {
        const auto frame = packer.pack(plain(static_cast<uint8_t>(i & 1U)));
        zassert_true(frame.has_value());
        zassert_equal(frame->dlc, 8);
    }
}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk5_status_is_copied_verbatim)
{
    CanPacker packer;
    uint8_t counter{0};
    for (const uint8_t status : {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0xFF}) {
        const Uplink in{make_uplink(0, status, 0x000001, 0x000002)};
        expect_frame(packer.pack(in), in, counter++);
    }
}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk6_channels_are_not_swapped_or_overlapped)
{
    constexpr std::array<uint32_t, 5> kCh1{0x000000, 0x000001, 0x7FFFFF, 0x800000, 0xFFFFFF};
    /* Each Ch1 value meets each Ch2 value: a full cross product of five distinct values. */
    constexpr std::array<uint32_t, 5> kCh2{0x123456, 0xFEDCBA, 0x000000, 0x800001, 0x00FF00};
    CanPacker packer;
    uint8_t counter{0};
    for (const uint32_t ch1 : kCh1) {
        for (const uint32_t ch2 : kCh2) {
            const Uplink in{make_uplink(0, 0x5A, ch1, ch2)};
            expect_frame(packer.pack(in), in, counter++);
        }
    }
}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk7_ch1_is_passed_through_raw)
{
    CanPacker packer;
    const auto frame = packer.pack(make_uplink(1, 0x00, 0x800000, 0x800000));
    zassert_true(frame.has_value());
    zassert_equal(frame->data[1], 0x80);
    zassert_equal(frame->data[2], 0x00);
    zassert_equal(frame->data[3], 0x00);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_pk8_counter_sits_at_byte_7)
{
    CanPacker packer;
    for (const unsigned wanted : {0U, 1U, 254U, 255U}) {
        packer.reset();
        burn(packer, 0, wanted);
        expect_frame(packer.pack(plain(0)), plain(0), static_cast<uint8_t>(wanted));
    }
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc1_first_frame_carries_zero)
{
    CanPacker packer;
    expect_frame(packer.pack(plain(1)), plain(1), 0);
    expect_frame(packer.pack(plain(1)), plain(1), 1);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc2_counter_wraps_at_the_256th_frame)
{
    CanPacker packer;
    for (unsigned i = 0; i < 256; ++i)
        expect_frame(packer.pack(plain(0)), plain(0), static_cast<uint8_t>(i));
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    expect_frame(packer.pack(plain(0)), plain(0), 1);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc3_boards_count_independently)
{
    CanPacker packer;
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    expect_frame(packer.pack(plain(0)), plain(0), 1);
    expect_frame(packer.pack(plain(0)), plain(0), 2);
    expect_frame(packer.pack(plain(1)), plain(1), 0);
    expect_frame(packer.pack(plain(0)), plain(0), 3);
    expect_frame(packer.pack(plain(1)), plain(1), 1);
    /* Wrap of one board does not touch the other. */
    burn(packer, 0, 252);
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    expect_frame(packer.pack(plain(1)), plain(1), 2);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc4_unknown_sender_between_valid_frames_moves_nothing)
{
    CanPacker packer;
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    expect_frame(packer.pack(plain(1)), plain(1), 0);
    uint8_t counter{1};
    for (const uint8_t sender : {uint8_t{2}, uint8_t{3}, uint8_t{0x7F}, uint8_t{0xFF}}) {
        zassert_false(packer.pack(plain(sender)).has_value());
        expect_frame(packer.pack(plain(0)), plain(0), counter);
        expect_frame(packer.pack(plain(1)), plain(1), counter);
        ++counter;
    }
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc5_identical_frames_both_increment)
{
    CanPacker packer;
    const Uplink in{make_uplink(0, 0x01, 0x000064, 0x0000C8)};
    expect_frame(packer.pack(in), in, 0);
    expect_frame(packer.pack(in), in, 1);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc6_reset_restarts_both_counters)
{
    CanPacker packer;
    burn(packer, 0, 7);
    burn(packer, 1, 300);
    packer.reset();
    expect_frame(packer.pack(plain(1)), plain(1), 0);
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    expect_frame(packer.pack(plain(0)), plain(0), 1);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc7_dropped_frame_leaves_a_gap)
{
    CanPacker packer;
    expect_frame(packer.pack(plain(0)), plain(0), 0);
    /* Packed, then lost after the counter step: the result is discarded. */
    const auto lost = packer.pack(plain(0));
    zassert_true(lost.has_value());
    expect_frame(packer.pack(plain(0)), plain(0), 2);
    expect_frame(packer.pack(plain(1)), plain(1), 0);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_fc8_no_input_means_no_frame_and_no_counter_move)
{
    CanPacker packer;
    burn(packer, 0, 5);
    burn(packer, 1, 9);
    /* The packer has no poll or timer entry: only pack() and reset() exist, so silence cannot change it. */
    expect_frame(packer.pack(plain(0)), plain(0), 5);
    expect_frame(packer.pack(plain(1)), plain(1), 9);
}

ZTEST(lcs_can_packer, test_lcs_can_packer_stream_matches_independent_oracle)
{
    Rng rng{0x2545F491U};
    CanPacker packer;
    std::array<uint8_t, 2> expected_counter{0, 0};
    for (unsigned i = 0; i < 20000; ++i) {
        const uint32_t r{rng.next()};
        if (r % 997U == 0) {
            packer.reset();
            expected_counter = {0, 0};
        }
        /* Senders 0..5 and 0xFF: mostly known, some unknown. */
        const uint8_t sender{(r >> 8) % 11U == 10U ? uint8_t{0xFF} : static_cast<uint8_t>((r >> 12) % 6U)};
        const Uplink in{make_uplink(sender, static_cast<uint8_t>(rng.next()), rng.next() & 0xFFFFFFU,
                                    rng.next() & 0xFFFFFFU)};
        const auto frame = packer.pack(in);
        if (sender > 1) {
            zassert_false(frame.has_value());
            continue;
        }
        expect_frame(frame, in, expected_counter[sender]);
        expected_counter[sender] = static_cast<uint8_t>((expected_counter[sender] + 1U) % 256U);
    }
}
