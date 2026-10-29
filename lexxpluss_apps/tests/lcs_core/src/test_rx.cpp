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

using namespace lcs_test;

ZTEST_SUITE(lcs_core_rx, NULL, NULL, NULL, NULL, NULL);

static uint32_t d12()
{
    return oracle_deadline_ticks(12);
}

/* Gap before the next frame: none, the deadline of a 12 B frame, and one tick more. */
static std::array<uint32_t, 3> boundary_gaps()
{
    return {0, d12(), d12() + 1};
}

/* The valid uplink that follows every ignore, skip and error path (the aftermath of RX-15). */
static void expect_aftermath(Rig &rig, const uint32_t tick, const Expect &before, const size_t out_before)
{
    rig.feed(frame_id1(), tick);
    Expect after{before};
    ++after.delivered;
    check_stats(rig.rx.stats(), after);
    rig.expect_last(uplink_id1(), out_before + 1);
}

ZTEST(lcs_core_rx, test_lcs_core_rx1_uplink_id0_alone)
{
    Rig rig;
    rig.feed(frame_id0(), 100);
    rig.expect_last(uplink_id0(), 1);
    check_stats(rig.rx.stats(), Expect{.delivered = 1});
}

ZTEST(lcs_core_rx, test_lcs_core_rx2_uplink_id1_alone)
{
    Rig rig;
    rig.feed(frame_id1(), 100);
    rig.expect_last(uplink_id1(), 1);
    check_stats(rig.rx.stats(), Expect{.delivered = 1});
}

ZTEST(lcs_core_rx, test_lcs_core_rx3_unknown_sender_is_dropped_and_counted)
{
    for (const uint8_t sender : {uint8_t{2}, uint8_t{0x7F}, uint8_t{0xFF}}) {
        Rig rig;
        rig.feed(uplink_frame(sender, 0xC0, 0x010203, 0x040506), 100);
        zassert_equal(rig.out_count, 0U);
        const Expect after_frame{.unknown_sender = 1};
        check_stats(rig.rx.stats(), after_frame);
        for (const Bytes &next : {frame_id0(), frame_id1()}) {
            for (const uint32_t gap : boundary_gaps()) {
                Rig again;
                again.feed(uplink_frame(sender, 0xC0, 0x010203, 0x040506), 100);
                again.feed(next, 100 + gap);
                zassert_equal(again.out_count, 1U);
                zassert_equal(again.out[0].sender_id, next.data[2]);
                check_stats(again.rx.stats(), Expect{.delivered = 1, .unknown_sender = 1});
            }
        }
    }
}

ZTEST(lcs_core_rx, test_lcs_core_rx4_adjacent_slots_both_orders)
{
    for (const uint32_t gap : {0U, 5U, d12() + 1}) {
        Rig forward;
        forward.feed(frame_id0(), 100);
        forward.feed(frame_id1(), 100 + gap);
        zassert_equal(forward.out_count, 2U);
        zassert_true(same_uplink(forward.out[0], uplink_id0()));
        zassert_true(same_uplink(forward.out[1], uplink_id1()));
        check_stats(forward.rx.stats(), Expect{.delivered = 2});

        Rig reverse;
        reverse.feed(frame_id1(), 100);
        reverse.feed(frame_id0(), 100 + gap);
        zassert_equal(reverse.out_count, 2U);
        zassert_true(same_uplink(reverse.out[0], uplink_id1()));
        zassert_true(same_uplink(reverse.out[1], uplink_id0()));
        check_stats(reverse.rx.stats(), Expect{.delivered = 2});
    }
}

/* The own SYNC echo (broadcast) and the downlink of another host look alike to the receiver. */
ZTEST(lcs_core_rx, test_lcs_core_rx6_downlink_alone)
{
    for (const Bytes &downlink : {downlink_frame(0xFF, 0x08, 0x1234), frame_downlink()}) {
        Rig rig;
        rig.feed(downlink, 100);
        zassert_equal(rig.out_count, 0U);
        check_stats(rig.rx.stats(), Expect{.foreign_downlink = 1});
    }
}

ZTEST(lcs_core_rx, test_lcs_core_rx7_downlink_then_uplink)
{
    for (const Bytes &downlink : {downlink_frame(0xFF, 0x08, 0x1234), frame_downlink()}) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(downlink, 100);
            expect_aftermath(rig, 100 + gap, Expect{.foreign_downlink = 1}, 0);
        }
    }
}

ZTEST(lcs_core_rx, test_lcs_core_rx8_uplink_then_downlink)
{
    for (const uint32_t gap : boundary_gaps()) {
        Rig rig;
        rig.feed(frame_id0(), 100);
        rig.feed(frame_downlink(), 100 + gap);
        rig.expect_last(uplink_id0(), 1);
        check_stats(rig.rx.stats(), Expect{.delivered = 1, .foreign_downlink = 1});
    }
}

/* A valid ID0 uplink whose bytes p and p + 1 are replaced by a SOF and a LEN. */
struct InnerSof {
    size_t position;
    uint8_t inner_len;
    uint32_t crc_errors;
};

ZTEST(lcs_core_rx, test_lcs_core_rx9_corrupted_uplink_with_inner_sof)
{
    /* The outer window counts its fault once; an inner window starts inside the span of the outer candidate. */
    static constexpr std::array<InnerSof, 6> kCases{
        {{5, 5, 1}, {5, 12, 1}, {4, 8, 1}, {9, 5, 1}, {9, 12, 1}, {5, 3, 1}}};
    for (const InnerSof &c : kCases) {
        Bytes corrupted{frame_id0()};
        corrupted.data[c.position] = 0x4B;
        corrupted.data[c.position + 1] = c.inner_len;
        zassert_false(ref_window_valid(corrupted, 12), "the outer window must be corrupted");

        Bytes stream{corrupted};
        stream.append(frame_id1(), 0, 12);
        if (c.inner_len >= 4) {
            Bytes inner;
            inner.append(stream, c.position, c.position + c.inner_len);
            zassert_false(ref_window_valid(inner, c.inner_len), "the inner window must be corrupted");
        }

        Rig rig;
        rig.feed(corrupted, 100);
        rig.feed(frame_id1(), 100);
        rig.expect_last(uplink_id1(), 1);
        check_stats(rig.rx.stats(), Expect{.delivered = 1, .crc_error = c.crc_errors});
        rig.rx.expire(100 + d12() + 1);
        zassert_equal(rig.rx.stats().timeout, 0U, "no candidate may be left behind");
    }
}

static uint16_t find_param_with_crc_byte_4b(const bool high)
{
    for (uint32_t param{0}; param < 0x10000U; ++param) {
        const Bytes frame{downlink_frame(1, 0x10, static_cast<uint16_t>(param))};
        if (frame.data[high ? 6 : 7] == 0x4B) {
            return static_cast<uint16_t>(param);
        }
    }
    zassert_true(false, "no downlink with a SOF-valued CRC byte");
    return 0;
}

static uint32_t find_ch2_with_crc_byte_4b(const bool high)
{
    for (uint32_t ch2{0}; ch2 < 0x10000U; ++ch2) {
        const Bytes frame{uplink_frame(1, 0xC0, 0x0ABCDE, ch2)};
        if (frame.data[high ? 10 : 11] == 0x4B) {
            return ch2;
        }
    }
    zassert_true(false, "no uplink with a SOF-valued CRC byte");
    return 0;
}

ZTEST(lcs_core_rx, test_lcs_core_rx10_imitating_bytes_in_downlink)
{
    const std::array<Bytes, 4> downlinks{
        downlink_frame(1, 0x10, 0x4B4B),
        downlink_frame(1, 0x10, find_param_with_crc_byte_4b(true)),
        downlink_frame(1, 0x10, find_param_with_crc_byte_4b(false)),
        downlink_frame(0xFF, 0x08, 0x1234),
    };
    for (const Bytes &downlink : downlinks) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(downlink, 100);
            rig.feed(frame_id1(), 100 + gap);
            rig.expect_last(uplink_id1(), 1);
            check_stats(rig.rx.stats(), Expect{.delivered = 1, .foreign_downlink = 1});
            rig.rx.expire(100 + gap + d12() + 1);
            zassert_equal(rig.rx.stats().timeout, 0U, "no candidate may be left behind");
        }
    }
}

ZTEST(lcs_core_rx, test_lcs_core_rx10_imitating_bytes_in_uplink)
{
    const std::array<Bytes, 3> uplinks{
        uplink_frame(0, 0xC0, 0x4B4B4B, 0x4B4B4B),
        uplink_frame(1, 0xC0, 0x0ABCDE, find_ch2_with_crc_byte_4b(true)),
        uplink_frame(1, 0xC0, 0x0ABCDE, find_ch2_with_crc_byte_4b(false)),
    };
    for (const Bytes &uplink : uplinks) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(uplink, 100);
            rig.feed(frame_id0(), 100 + gap);
            zassert_equal(rig.out_count, 2U);
            zassert_true(same_uplink(rig.out[0], Uplink{uplink.data[2], uplink.data[3],
                                                         {uplink.data[4], uplink.data[5], uplink.data[6]},
                                                         {uplink.data[7], uplink.data[8], uplink.data[9]}}));
            zassert_true(same_uplink(rig.out[1], uplink_id0()));
            check_stats(rig.rx.stats(), Expect{.delivered = 2});
            rig.rx.expire(100 + gap + d12() + 1);
            zassert_equal(rig.rx.stats().timeout, 0U, "no candidate may be left behind");
        }
    }
}

static Bytes frame_with_len(const size_t len)
{
    switch (len) {
    case 9:
        return raw_frame(9, 9, 1, 0x11);
    case 11:
        return raw_frame(11, 11, 1, 0xC1);
    case 13:
        return raw_frame(13, 13, 1, 0x11);
    case 255:
        return raw_frame(12, 255, 1, 0xC1);
    default:
        /* Too short to hold a frame: a downlink-shaped frame whose LEN byte says len. */
        return raw_frame(8, static_cast<uint8_t>(len), 1, 0x11);
    }
}

/* LEN below 4 cannot be a frame: dropped at the LEN byte; the stray bytes behind it count sof_mismatch only. */
ZTEST(lcs_core_rx, test_lcs_core_rx11_len_below_minimum)
{
    for (const size_t len : {size_t{0}, size_t{1}, size_t{2}, size_t{3}}) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(frame_with_len(len), 100);
            zassert_equal(rig.out_count, 0U);
            check_stats_except_sof(rig.rx.stats(), Expect{});
            Expect before{};
            before.sof_mismatch = rig.rx.stats().sof_mismatch;
            expect_aftermath(rig, 100 + gap, before, 0);
        }
    }
}

/* Frame of len bytes (4..7) with a valid CRC; the byte behind LEN is the SOF when with_sof is set. */
static Bytes short_frame(const size_t len, const bool with_sof)
{
    Bytes body;
    body.push(0x4B);
    body.push(static_cast<uint8_t>(len));
    for (size_t i{2}; i < len - 2; ++i) {
        body.push(with_sof && i == 2 ? 0x4B : static_cast<uint8_t>(0x20 + i));
    }
    return with_crc(body);
}

/* LEN 4..7 is an extension point: a CRC-valid frame is consumed silently, whatever its payload holds. */
ZTEST(lcs_core_rx, test_lcs_core_rx11_len_extension_point)
{
    for (size_t len{4}; len <= 7; ++len) {
        for (const bool with_sof : {false, true}) {
            const Bytes frame{short_frame(len, with_sof)};
            zassert_true(ref_window_valid(frame, len));
            for (const uint32_t gap : boundary_gaps()) {
                Rig rig;
                rig.feed(frame, 100);
                zassert_equal(rig.out_count, 0U);
                check_stats(rig.rx.stats(), Expect{});
                expect_aftermath(rig, 100 + gap, Expect{}, 0);
                rig.rx.expire(100 + gap + d12() + 1);
                zassert_equal(rig.rx.stats().timeout, 0U, "no candidate may be left behind, LEN %u",
                              static_cast<unsigned>(len));
            }
        }
    }
}

/* A LEN 4..7 window with a wrong CRC counts once; the uplink that starts inside it is still found. */
ZTEST(lcs_core_rx, test_lcs_core_rx11_len_extension_wrong_crc)
{
    for (size_t len{4}; len <= 7; ++len) {
        Bytes prefix;
        prefix.push(0x4B);
        prefix.push(static_cast<uint8_t>(len));
        prefix.push(0x20);
        Bytes window{prefix};
        window.append(frame_id1(), 0, len - 3);
        zassert_false(ref_window_valid(window, len), "the window must be corrupted");

        Rig rig;
        rig.feed(prefix, 100);
        rig.feed(frame_id1(), 100);
        rig.expect_last(uplink_id1(), 1);
        check_stats(rig.rx.stats(), Expect{.delivered = 1, .crc_error = 1});
    }
}

/* A truncated LEN 6 frame lives for the deadline of 6 bytes, not of 12 and not of the header. */
ZTEST(lcs_core_rx, test_lcs_core_rx11_len_6_truncated_deadline)
{
    const uint32_t d6{oracle_deadline_ticks(6)};
    zassert_true(d6 > oracle_deadline_ticks(2) && d6 < d12());
    const Bytes frame{short_frame(6, false)};
    Bytes window{frame};
    window.size = 3;
    window.append(frame_id1(), 0, 3);
    zassert_false(ref_window_valid(window, 6), "the window must be corrupted");

    Rig alive;
    alive.feed(frame, 1000, 0, 3);
    alive.rx.expire(1000 + d6);
    check_stats(alive.rx.stats(), Expect{});
    alive.rx.expire(1000 + d6 + 1);
    check_stats(alive.rx.stats(), Expect{.timeout = 1});

    Rig late;
    late.feed(frame, 1000, 0, 3);
    late.feed(frame_id1(), 1000 + d6);
    late.expect_last(uplink_id1(), 1);
    check_stats(late.rx.stats(), Expect{.delivered = 1, .crc_error = 1});
}

/* A window of a length other than 8 or 12 with a valid CRC is consumed and dropped without any counter. */
ZTEST(lcs_core_rx, test_lcs_core_rx11_len_between_frame_sizes)
{
    for (const size_t len : {size_t{9}, size_t{11}}) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(frame_with_len(len), 100);
            zassert_equal(rig.out_count, 0U);
            check_stats(rig.rx.stats(), Expect{});
            expect_aftermath(rig, 100 + gap, Expect{}, 0);
        }
    }
}

/* LEN above the maximum: the candidate lives with the deadline of 12 B, is not CRC-checked and then times out. */
ZTEST(lcs_core_rx, test_lcs_core_rx11_len_above_maximum)
{
    for (const size_t len : {size_t{13}, size_t{255}}) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(frame_with_len(len), 100);
            zassert_equal(rig.out_count, 0U);
            check_stats(rig.rx.stats(), Expect{});
            rig.feed(frame_id1(), 100 + gap);
            rig.expect_last(uplink_id1(), 1);
            const uint32_t expired_at_gap{gap > d12() ? 1U : 0U};
            check_stats(rig.rx.stats(), Expect{.delivered = 1, .timeout = expired_at_gap});
            rig.rx.expire(100 + d12());
            check_stats(rig.rx.stats(), Expect{.delivered = 1, .timeout = expired_at_gap});
            rig.rx.expire(100 + d12() + 1);
            check_stats(rig.rx.stats(), Expect{.delivered = 1, .timeout = 1});
            rig.feed(frame_id0(), 100 + d12() + 2);
            rig.expect_last(uplink_id0(), 2);
        }
    }
}

/* Valid CRC but LEN and the direction bit disagree. */
ZTEST(lcs_core_rx, test_lcs_core_rx12_direction_disagrees_with_len)
{
    for (const Bytes &frame : {raw_frame(12, 12, 1, 0x11), raw_frame(8, 8, 1, 0xC1)}) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(frame, 100);
            zassert_equal(rig.out_count, 0U);
            check_stats(rig.rx.stats(), Expect{});
            expect_aftermath(rig, 100 + gap, Expect{}, 0);
        }
    }
}

/* The next frame arrives as one drain, so the old window completes inside it. */
ZTEST(lcs_core_rx, test_lcs_core_rx13_truncated_frame_deadline_boundary)
{
    for (size_t k{2}; k <= 11; ++k) {
        Bytes window;
        window.append(frame_id0(), 0, k);
        window.append(frame_id1(), 0, 12 - k);
        zassert_false(ref_window_valid(window, 12), "the old window must not be valid");

        Rig alive;
        alive.feed(frame_id0(), 1000, 0, k);
        alive.feed(frame_id1(), 1000 + d12());
        alive.expect_last(uplink_id1(), 1);
        check_stats(alive.rx.stats(), Expect{.delivered = 1, .crc_error = 1});

        Rig expired;
        expired.feed(frame_id0(), 1000, 0, k);
        expired.feed(frame_id1(), 1000 + d12() + 1);
        expired.expect_last(uplink_id1(), 1);
        check_stats(expired.rx.stats(), Expect{.delivered = 1, .timeout = 1});
    }
}

/* Only SOF is received: the deadline of the two header bytes applies until LEN arrives. */
ZTEST(lcs_core_rx, test_lcs_core_rx13_header_deadline_boundary)
{
    const uint32_t header{oracle_deadline_ticks(2)};

    Rig expired;
    expired.feed(frame_id0(), 1000, 0, 1);
    expired.feed(frame_id1(), 1000 + header + 1);
    expired.expect_last(uplink_id1(), 1);
    check_stats(expired.rx.stats(), Expect{.delivered = 1, .timeout = 1});

    /* The next SOF is the LEN byte (0x4B, above the maximum) of the old candidate: it lives on to 12 B deadline. */
    Rig alive;
    alive.feed(frame_id0(), 1000, 0, 1);
    alive.feed(frame_id1(), 1000 + header);
    alive.expect_last(uplink_id1(), 1);
    check_stats(alive.rx.stats(), Expect{.delivered = 1});
    alive.rx.expire(1000 + d12());
    check_stats(alive.rx.stats(), Expect{.delivered = 1});
    alive.rx.expire(1000 + d12() + 1);
    check_stats(alive.rx.stats(), Expect{.delivered = 1, .timeout = 1});
}

ZTEST(lcs_core_rx, test_lcs_core_rx14_uart_error_mid_frame)
{
    for (const size_t k : {size_t{1}, size_t{2}, size_t{6}, size_t{11}}) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            rig.feed(frame_id0(), 1000, 0, k);
            rig.rx.on_uart_error(1000);
            rig.feed(frame_id1(), 1000 + gap);
            rig.expect_last(uplink_id1(), 1);
            check_stats(rig.rx.stats(), Expect{.delivered = 1, .uart_error = 1});
        }
    }
}

enum class Prelude { UnknownSender, Downlink, Len9, Len12WithCmdBit, Len8WithStatusBit, UartError, CrcError, kCount };

static void apply_prelude(Rig &rig, const Prelude prelude, const uint32_t tick, Expect &expect, size_t &out_count)
{
    switch (prelude) {
    case Prelude::UnknownSender:
        rig.feed(uplink_frame(2, 0xC0, 0x010203, 0x040506), tick);
        ++expect.unknown_sender;
        break;
    case Prelude::Downlink:
        rig.feed(frame_downlink(), tick);
        ++expect.foreign_downlink;
        break;
    case Prelude::Len9:
        rig.feed(frame_with_len(9), tick);
        break;
    case Prelude::Len12WithCmdBit:
        rig.feed(raw_frame(12, 12, 1, 0x11), tick);
        break;
    case Prelude::Len8WithStatusBit:
        rig.feed(raw_frame(8, 8, 1, 0xC1), tick);
        break;
    case Prelude::UartError:
        rig.feed(frame_id0(), tick, 0, 6);
        rig.rx.on_uart_error(tick);
        ++expect.uart_error;
        break;
    case Prelude::CrcError: {
        Bytes corrupted{frame_id0()};
        corrupted.data[5] = static_cast<uint8_t>(corrupted.data[5] ^ 0x01U);
        zassert_false(ref_window_valid(corrupted, 12));
        rig.feed(corrupted, tick);
        ++expect.crc_error;
        break;
    }
    case Prelude::kCount:
        break;
    }
}

ZTEST(lcs_core_rx, test_lcs_core_rx15_aftermath_of_every_ignore_and_error_path)
{
    for (size_t i{0}; i < static_cast<size_t>(Prelude::kCount); ++i) {
        for (const uint32_t gap : boundary_gaps()) {
            Rig rig;
            Expect expect{};
            size_t out_count{0};
            apply_prelude(rig, static_cast<Prelude>(i), 1000, expect, out_count);
            check_stats(rig.rx.stats(), expect);
            expect_aftermath(rig, 1000 + gap, expect, out_count);
            rig.rx.expire(1000 + gap + d12() + 1);
            zassert_equal(rig.rx.stats().timeout, 0U, "no candidate may be left behind, prelude %u",
                          static_cast<unsigned>(i));
        }
    }
}

/* A flood of SOF bytes exceeds the number of candidates; a valid uplink behind it is still returned. */
ZTEST(lcs_core_rx, test_lcs_core_extra_sof_flood_then_uplink)
{
    Rig rig;
    for (int i{0}; i < 40; ++i) {
        rig.rx.on_byte(0x4B, 1000);
    }
    rig.feed(frame_id0(), 1000);
    rig.expect_last(uplink_id0(), 1);
    zassert_equal(rig.rx.stats().crc_error, 0U);
    rig.rx.expire(1000 + d12() + 1);
    rig.feed(frame_id1(), 1000 + d12() + 2);
    rig.expect_last(uplink_id1(), 2);
}
