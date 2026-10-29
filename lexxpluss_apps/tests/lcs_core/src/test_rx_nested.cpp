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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include <zephyr/ztest.h>

#include "lcs_core_test_support.hpp"

using namespace lcs_test;

ZTEST_SUITE(lcs_core_rx_nested, NULL, NULL, NULL, NULL, NULL);

/*
 * Index 0 of every stream is the SOF of the older candidate A; N is the candidate that starts inside the span of A.
 * Every case runs twice: on the reference model of the nested rule (it validates the hand-computed expectations)
 * and on the receiver under test. Where the two runs differ, the receiver does not implement the rule yet.
 */
#define NESTED_CASE(id, ...)                                         \
    ZTEST(lcs_core_rx_nested, test_lcs_core_##id##_model)            \
    {                                                                \
        using RigT = ModelRig<true>;                                 \
        __VA_ARGS__;                                                 \
    }                                                                \
    ZTEST(lcs_core_rx_nested, test_lcs_core_##id##_dut)              \
    {                                                                \
        using RigT = Rig;                                            \
        __VA_ARGS__;                                                 \
    }

static constexpr uint32_t kT0{1000};

static uint32_t d(const uint32_t len)
{
    return oracle_deadline_ticks(len);
}

static uint32_t d12()
{
    return oracle_deadline_ticks(12);
}

/* An uplink frame whose first frame bytes are found by search: accept() decides on the finished frame. */
template <typename Accept>
static Bytes search_uplink(const uint32_t ch1, const uint32_t ch2_top, const Accept accept)
{
    for (uint32_t tail{0}; tail < 0x10000U; ++tail) {
        const Bytes frame{uplink_frame(0, 0xC0, ch1, (ch2_top << 16) | tail)};
        if (accept(frame)) {
            return frame;
        }
    }
    zassert_true(false, "no uplink frame with the wanted bytes");
    return Bytes{};
}

/* A valid ID0 uplink with a SOF at index p of the payload and the byte next behind it; the other bytes are fillers. */
static Bytes payload_sof_frame(const size_t p, const uint8_t next)
{
    Bytes body;
    body.push(0x4B);
    body.push(12);
    body.push(0);
    body.push(0xC0);
    for (size_t i{4}; i < 10; ++i) {
        body.push(static_cast<uint8_t>(0x50 + i));
    }
    body.data[p] = 0x4B;
    body.data[p + 1] = next;
    return with_crc(body);
}

/* D1: older candidate A of LEN older_len (0x80: oversize), nested candidate N at index s with LEN 4, wrong CRC. */
template <typename RigT>
static void run_d1(const uint8_t older_len, const size_t s)
{
    const size_t span{std::min<size_t>(older_len, 12)};
    Tape<RigT> t{pattern(std::max(span, s + 4), older_len, {{s, 0x4B}, {s + 1, 4}})};
    if (older_len <= 12) {
        assert_window(t.planned, 0, older_len, false);
    }
    assert_window(t.planned, s, 4, false);
    t.feed_all(kT0);
    if (older_len <= 12) {
        t.expect(Expect{.crc_error = 1});
        t.expire(kT0 + d12() + 1);
        t.expect(Expect{.crc_error = 1});
    } else {
        t.expect(Expect{});
        t.expire(kT0 + d12() + 1);
        t.expect(Expect{.timeout = 1});
    }
}

/* D1-01: a SOF right behind the window of A is not covered by A (A has left), so its own fault counts. */
template <typename RigT>
static void run_d1_01()
{
    for (const bool oversize_new : {false, true}) {
        Tape<RigT> t{pattern(16, 12, {{12, 0x4B}, {13, oversize_new ? uint8_t{0x80} : uint8_t{4}}})};
        assert_window(t.planned, 0, 12, false);
        if (!oversize_new) {
            assert_window(t.planned, 12, 4, false);
        }
        t.feed_all(kT0);
        t.expire(kT0 + d12() + 1);
        t.expect(oversize_new ? Expect{.crc_error = 1, .timeout = 1} : Expect{.crc_error = 2});
    }
}

/* D1-10: A is oversize and still in progress, but the SOF at index 12 is outside its span of 12. */
template <typename RigT>
static void run_d1_10()
{
    Tape<RigT> t{pattern(16, 0x80, {{12, 0x4B}, {13, 4}})};
    assert_window(t.planned, 12, 4, false);
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1, .timeout = 1});
}

NESTED_CASE(d1_01, run_d1_01<RigT>())
NESTED_CASE(d1_02, run_d1<RigT>(4, 2))
NESTED_CASE(d1_03, run_d1<RigT>(4, 3))
NESTED_CASE(d1_04, run_d1<RigT>(8, 4))
NESTED_CASE(d1_05, run_d1<RigT>(8, 7))
NESTED_CASE(d1_06, run_d1<RigT>(12, 6))
NESTED_CASE(d1_07, run_d1<RigT>(12, 11))
NESTED_CASE(d1_08, run_d1<RigT>(0x80, 6))
NESTED_CASE(d1_09, run_d1<RigT>(0x80, 11))
NESTED_CASE(d1_10, run_d1_10<RigT>())

/*
 * D2 fate matrix. Unreachable cells: none dropped. The brief's range "A index 2..9" for the foreign downlink of
 * D2-01 is narrowed to index 2 so that N resolves before A (N ends at index 9, A at index 11).
 */

/* D2-01: N is a valid foreign downlink and resolves first, A (LEN 12, wrong CRC) counts crc_error later. */
template <typename RigT>
static void run_d2_01()
{
    Bytes stream{pattern(12, 12)};
    overlay(stream, 2, frame_downlink());
    Tape<RigT> t{stream};
    assert_window(t.planned, 0, 12, false);
    assert_window(t.planned, 2, 8, true);
    t.feed_until(10, kT0);
    t.expect(Expect{.foreign_downlink = 1});
    t.feed_until(12, kT0);
    t.expect(Expect{.crc_error = 1, .foreign_downlink = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1, .foreign_downlink = 1});
}

/* D2-02: A (LEN 4, wrong CRC) counts at index 3, N (valid uplink from index 2) is delivered later. */
template <typename RigT>
static void run_d2_02()
{
    Bytes stream{pattern(14, 4)};
    overlay(stream, 2, frame_id0());
    Tape<RigT> t{stream};
    assert_window(t.planned, 0, 4, false);
    assert_window(t.planned, 2, 12, true);
    t.feed_until(4, kT0);
    t.expect(Expect{.crc_error = 1});
    t.feed_all(kT0);
    t.expect(Expect{.delivered = 1, .crc_error = 1});
    t.rig.expect_last(uplink_id0(), 1);
}

/* D2-03: A is oversize, N is delivered inside A's deadline, A times out when the tick passes its deadline. */
template <typename RigT>
static void run_d2_03()
{
    for (const size_t s : {size_t{2}, size_t{11}}) {
        Bytes stream{pattern(s + 12, 0x80)};
        overlay(stream, s, frame_id0());
        Tape<RigT> t{stream};
        t.feed_all(kT0);
        t.expect(Expect{.delivered = 1});
        t.rig.expect_last(uplink_id0(), 1);
        t.expire(kT0 + d12() + 1);
        t.expect(Expect{.delivered = 1, .timeout = 1});
    }
}

/* D2-04: N starts later than A; A (oversize) expires before the last bytes of N arrive, N is delivered after. */
template <typename RigT>
static void run_d2_04()
{
    Bytes stream{pattern(14, 0x80)};
    overlay(stream, 2, frame_id0());
    Tape<RigT> t{stream};
    const uint32_t n_tick{kT0 + d12() - 3};
    const uint32_t late{kT0 + d12() + 1};
    t.feed_until(2, kT0);
    t.feed_until(7, n_tick);
    t.expect(Expect{});
    t.feed_until(8, late);
    t.expect(Expect{.timeout = 1});
    t.feed_all(late);
    t.expect(Expect{.delivered = 1, .timeout = 1});
    t.rig.expect_last(uplink_id0(), 1);
}

/* D2-05: N (LEN 4, wrong CRC) completes before A (LEN 12, wrong CRC): one fault, one counter. */
template <typename RigT>
static void run_d2_05()
{
    Tape<RigT> t{pattern(12, 12, {{2, 0x4B}, {3, 4}})};
    assert_window(t.planned, 0, 12, false);
    assert_window(t.planned, 2, 4, false);
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1});
}

/* D2-06: A (LEN 4, wrong CRC) counts first, N (LEN 12 from index 2, wrong CRC) completes silently later. */
template <typename RigT>
static void run_d2_06()
{
    Tape<RigT> t{pattern(14, 4, {{2, 0x4B}, {3, 12}})};
    assert_window(t.planned, 0, 4, false);
    assert_window(t.planned, 2, 12, false);
    t.feed_until(4, kT0);
    t.expect(Expect{.crc_error = 1});
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1});
}

/* D2-07: N (LEN 4, wrong CRC) completes first, A (oversize) times out: timeout only. */
template <typename RigT>
static void run_d2_07()
{
    Tape<RigT> t{pattern(6, 0x80, {{2, 0x4B}, {3, 4}})};
    assert_window(t.planned, 2, 4, false);
    t.feed_all(kT0);
    t.expect(Expect{});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/*
 * D2-08: A (LEN 4) stalls after index 2 and expires; N starts later (tick kT0 + 4), so N is still alive when its
 * LEN byte (index 3, after A has left) and the rest of its wrong-CRC window arrive.
 */
template <typename RigT>
static void run_d2_08()
{
    Tape<RigT> t{pattern(6, 4, {{2, 0x4B}, {3, 4}})};
    assert_window(t.planned, 2, 4, false);
    const uint32_t n_tick{kT0 + d(4) - 3};
    const uint32_t late{kT0 + d(4) + 1};
    t.feed_until(2, kT0);
    t.feed_until(3, n_tick);
    t.expect(Expect{});
    t.feed_until(4, late);
    t.expect(Expect{.timeout = 1});
    t.feed_all(late);
    t.expect(Expect{.timeout = 1});
    t.expire(late + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/* D2-09: N (LEN 4) is truncated and expires first, A (LEN 12, wrong CRC) survives and completes. */
template <typename RigT>
static void run_d2_09()
{
    for (const uint32_t gap : {d(4) + 1, d12()}) {
        Tape<RigT> t{pattern(12, 12, {{2, 0x4B}, {3, 4}})};
        assert_window(t.planned, 0, 12, false);
        t.feed_until(4, kT0);
        t.expect(Expect{});
        t.feed_until(5, kT0 + gap);
        t.expect(Expect{});
        t.feed_all(kT0 + gap);
        t.expect(Expect{.crc_error = 1});
        t.expire(kT0 + gap + d12() + 1);
        t.expect(Expect{.crc_error = 1});
    }
}

/* D2-10: A (LEN 4, wrong CRC) counts at index 3, N (LEN 12 from index 2) is left alone and expires silently. */
template <typename RigT>
static void run_d2_10()
{
    Tape<RigT> t{pattern(4, 4, {{2, 0x4B}, {3, 12}})};
    assert_window(t.planned, 0, 4, false);
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1});
}

/* D2-11: A is oversize, N (LEN 4) is truncated and expires first (silently); A times out later. */
template <typename RigT>
static void run_d2_11()
{
    Tape<RigT> t{pattern(4, 0x80, {{2, 0x4B}, {3, 4}})};
    t.feed_all(kT0);
    t.expect(Expect{});
    t.expire(kT0 + d(4) + 1);
    t.expect(Expect{});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/* D2-12: A (LEN 4) expires first, then N (LEN 12, its LEN byte arrives after A has left) expires silently. */
template <typename RigT>
static void run_d2_12()
{
    Tape<RigT> t{pattern(4, 4, {{2, 0x4B}, {3, 12}})};
    const uint32_t n_tick{kT0 + d(4) - 3};
    const uint32_t late{kT0 + d(4) + 1};
    t.feed_until(2, kT0);
    t.feed_until(3, n_tick);
    t.expect(Expect{});
    t.feed_until(4, late);
    t.expect(Expect{.timeout = 1});
    t.expire(n_tick + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/*
 * D2-13: A (oversize) and five nested pairs 0x4B 0x80 at a later tick; A expires, seven more SOFs fill the table
 * and the eighth evicts the oldest nested candidate, which counts nothing.
 */
template <typename RigT>
static void run_d2_13()
{
    Bytes stream;
    for (size_t pair{0}; pair < 6; ++pair) {
        stream.push(0x4B);
        stream.push(0x80);
    }
    for (size_t i{0}; i < 8; ++i) {
        stream.push(0x4B);
    }
    Tape<RigT> t{stream};
    t.feed_until(2, kT0);
    t.feed_until(12, kT0 + 3);
    t.expect(Expect{});
    t.feed_until(19, kT0 + d12() + 1);
    t.expect(Expect{.timeout = 1});
    t.feed_until(20, kT0 + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/* D2-14: a UART error clears A and N together; nothing else is counted and the next frame is delivered. */
template <typename RigT>
static void run_d2_14()
{
    Tape<RigT> t{pattern(4, 0x80, {{2, 0x4B}, {3, 4}})};
    t.feed_all(kT0);
    t.rig.rx.on_uart_error(kT0);
    t.expect(Expect{.uart_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.uart_error = 1});
    t.rig.feed(frame_id1(), kT0 + d12() + 2);
    t.expect(Expect{.delivered = 1, .uart_error = 1});
    t.rig.expect_last(uplink_id1(), 1);
}

/*
 * D2-15: the even older frame V is a valid uplink with SOF at 4 (LEN byte 12: candidate A) and at 6 (LEN byte 12:
 * candidate N); A and N are still in progress when V matches at index 11 and are consumed with it.
 */
template <typename RigT>
static void run_d2_15()
{
    const Bytes frame{search_uplink(0x4B0C4BU, 0x0C, [](const Bytes &f) {
        return f.data[8] != 0x4B && f.data[9] != 0x4B && f.data[10] != 0x4B && f.data[11] != 0x4B;
    })};
    Tape<RigT> t{frame};
    t.feed_all(kT0);
    t.expect(Expect{.delivered = 1});
    t.rig.expect_last(uplink_of_frame(frame), 1);
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.delivered = 1});
    t.rig.feed(frame_id1(), kT0 + d12() + 2);
    t.expect(Expect{.delivered = 2});
    t.rig.expect_last(uplink_id1(), 2);
}

NESTED_CASE(d2_01, run_d2_01<RigT>())
NESTED_CASE(d2_02, run_d2_02<RigT>())
NESTED_CASE(d2_03, run_d2_03<RigT>())
NESTED_CASE(d2_04, run_d2_04<RigT>())
NESTED_CASE(d2_05, run_d2_05<RigT>())
NESTED_CASE(d2_06, run_d2_06<RigT>())
NESTED_CASE(d2_07, run_d2_07<RigT>())
NESTED_CASE(d2_08, run_d2_08<RigT>())
NESTED_CASE(d2_09, run_d2_09<RigT>())
NESTED_CASE(d2_10, run_d2_10<RigT>())
NESTED_CASE(d2_11, run_d2_11<RigT>())
NESTED_CASE(d2_12, run_d2_12<RigT>())
NESTED_CASE(d2_13, run_d2_13<RigT>())
NESTED_CASE(d2_14, run_d2_14<RigT>())
NESTED_CASE(d2_15, run_d2_15<RigT>())

/* D3: a valid frame with a SOF in the payload or in the CRC bytes is delivered once and no counter moves. */
template <typename RigT>
static void run_d3_frame(const Bytes &frame)
{
    Tape<RigT> t{frame};
    t.feed_all(kT0);
    t.expect(Expect{.delivered = 1});
    t.rig.expect_last(uplink_of_frame(frame), 1);
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.delivered = 1});
}

template <typename RigT>
static void run_d3(const size_t p, const uint8_t next)
{
    const Bytes frame{payload_sof_frame(p, next)};
    zassert_true(ref_window_valid(frame, 12), "the frame must be valid");
    /* A window that ends inside the frame must be wrong, else it would swallow the real frame by chance. */
    if (next >= 4 && next <= 12 && p + next <= frame.size) {
        assert_window(frame, p, next, false);
    }
    run_d3_frame<RigT>(frame);
}

template <typename RigT>
static void run_d3_01()
{
    size_t executed{0};
    for (size_t p{4}; p <= 8; ++p) {
        for (uint8_t next{4}; next <= 11; ++next) {
            run_d3<RigT>(p, next);
            ++executed;
        }
    }
    zassert_equal(executed, 40U);
}

/* D3-06: the CRC high byte (index 10) is the SOF, the CRC low byte is its LEN. */
template <typename RigT>
static void run_d3_06()
{
    const Bytes frame{search_uplink(0x555555U, 0x56, [](const Bytes &f) {
        return f.data[8] != 0x4B && f.data[9] != 0x4B && f.data[10] == 0x4B;
    })};
    run_d3_frame<RigT>(frame);
}

/* D3-07: the SOF at index 9 gets the CRC high byte (4..11) as its LEN. */
template <typename RigT>
static void run_d3_07()
{
    const Bytes frame{search_uplink(0x555555U, 0x56, [](const Bytes &f) {
        return f.data[8] != 0x4B && f.data[9] == 0x4B && f.data[10] >= 4 && f.data[10] <= 11;
    })};
    run_d3_frame<RigT>(frame);
}

NESTED_CASE(d3_01, run_d3_01<RigT>())
NESTED_CASE(d3_02, run_d3<RigT>(4, 3))
NESTED_CASE(d3_03, run_d3<RigT>(4, 12))
NESTED_CASE(d3_04, run_d3<RigT>(4, 13))
NESTED_CASE(d3_05, run_d3<RigT>(4, 255))
NESTED_CASE(d3_06, run_d3_06<RigT>())
NESTED_CASE(d3_07, run_d3_07<RigT>())

/* CE-i: a corrupted frame with a payload SOF and an oversize LEN behind it: the parent counts, the nested one not. */
template <typename RigT>
static void run_ce_i()
{
    Bytes frame{payload_sof_frame(4, 0x80)};
    frame.data[8] = static_cast<uint8_t>(frame.data[8] ^ 0x01U);
    assert_window(frame, 0, 12, false);
    Tape<RigT> t{frame};
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1});
}

/* CE-ii: 0x4B 0x4B; the second SOF is the LEN byte of the first, which makes A oversize before N starts. */
template <typename RigT>
static void run_ce_ii()
{
    Bytes stream;
    stream.push(0x4B);
    stream.push(0x4B);
    stream.push(4);
    stream.push(0x53);
    stream.push(0x54);
    assert_window(stream, 1, 4, false);
    Tape<RigT> t{stream};
    t.feed_all(kT0);
    t.expect(Expect{});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/* CE-iii-a: the CRC low byte of a valid frame is the SOF; N starts at the byte that completes the frame. */
template <typename RigT>
static void run_ce_iii_a()
{
    const Bytes frame{search_uplink(0x555555U, 0x56, [](const Bytes &f) {
        return f.data[8] != 0x4B && f.data[9] != 0x4B && f.data[10] != 0x4B && f.data[11] == 0x4B;
    })};
    Tape<RigT> t{frame};
    t.feed_all(kT0);
    t.expect(Expect{.delivered = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.delivered = 1});
    t.rig.feed(frame_id1(), kT0 + d12() + 2);
    t.expect(Expect{.delivered = 2});
    t.rig.expect_last(uplink_id1(), 2);
}

/* CE-iii-b: the same SOF position, but the frame is corrupted: the parent counts, N completes silently. */
template <typename RigT>
static void run_ce_iii_b()
{
    Bytes stream{search_uplink(0x555555U, 0x56, [](const Bytes &f) {
        return f.data[8] != 0x4B && f.data[9] != 0x4B && f.data[10] != 0x4B && f.data[11] == 0x4B;
    })};
    stream.data[6] = static_cast<uint8_t>(stream.data[6] ^ 0x01U);
    stream.push(4);
    stream.push(0x50);
    stream.push(0x51);
    assert_window(stream, 0, 12, false);
    assert_window(stream, 11, 4, false);
    Tape<RigT> t{stream};
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1});
}

/* CE-iv: 16 pairs 0x4B 0x80; the 13th SOF evicts A (counts), the evictions of nested candidates count nothing. */
template <typename RigT>
static void run_ce_iv()
{
    Bytes stream;
    for (size_t pair{0}; pair < 16; ++pair) {
        stream.push(0x4B);
        stream.push(0x80);
    }
    Tape<RigT> t{stream};
    t.feed_until(24, kT0);
    t.expect(Expect{});
    t.feed_until(26, kT0);
    t.expect(Expect{.timeout = 1});
    for (size_t pair{14}; pair <= 16; ++pair) {
        t.feed_until(2 * pair, kT0);
        t.expect(Expect{.timeout = 1});
    }
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.timeout = 1});
}

/* CE-v: a frame hidden behind the corrupted LEN of a downlink in front of it is found. */
template <typename RigT>
static void run_ce_v()
{
    for (const uint8_t corrupted_len : {uint8_t{12}, uint8_t{0x30}}) {
        Bytes downlink{frame_downlink()};
        downlink.data[1] = corrupted_len;
        Bytes stream;
        stream.append(downlink, 0, downlink.size);
        stream.append(frame_id1(), 0, 12);
        Tape<RigT> t{stream};
        t.feed_all(kT0);
        if (corrupted_len == 12) {
            assert_window(t.planned, 0, 12, false);
            t.expect(Expect{.delivered = 1, .crc_error = 1});
        } else {
            t.expect(Expect{.delivered = 1});
        }
        t.rig.expect_last(uplink_id1(), 1);
        t.expire(kT0 + d12() + 1);
        t.expect(corrupted_len == 12 ? Expect{.delivered = 1, .crc_error = 1} : Expect{.delivered = 1, .timeout = 1});
    }
}

/* CE-vii: a junk SOF with LEN 12 covers a corrupted real frame: two faults, one counter (the accepted merge). */
template <typename RigT>
static void run_ce_vii()
{
    Bytes real{frame_id1()};
    real.data[6] = static_cast<uint8_t>(real.data[6] ^ 0x01U);
    Bytes stream{pattern(16, 12)};
    overlay(stream, 4, real);
    assert_window(stream, 0, 12, false);
    assert_window(stream, 4, 12, false);
    Tape<RigT> t{stream};
    t.feed_until(12, kT0);
    t.expect(Expect{.crc_error = 1});
    t.feed_all(kT0);
    t.expect(Expect{.crc_error = 1});
    t.expire(kT0 + d12() + 1);
    t.expect(Expect{.crc_error = 1});
}

NESTED_CASE(ce_i, run_ce_i<RigT>())
NESTED_CASE(ce_ii, run_ce_ii<RigT>())
NESTED_CASE(ce_iii_a, run_ce_iii_a<RigT>())
NESTED_CASE(ce_iii_b, run_ce_iii_b<RigT>())
NESTED_CASE(ce_iv, run_ce_iv<RigT>())
NESTED_CASE(ce_v, run_ce_v<RigT>())
NESTED_CASE(ce_vii, run_ce_vii<RigT>())
