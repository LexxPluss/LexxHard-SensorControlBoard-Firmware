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

#include <array>
#include <cstddef>
#include <cstdint>

#include <zephyr/ztest.h>

#include "lcs_frame.hpp"
#include "lcs_rx.hpp"

namespace lcs_test {

using lexxhard::lcs::RxStats;
using lexxhard::lcs::Uplink;
using lexxhard::lcs::UplinkReceiver;

/*
 * The expected values of the tests do not come from the code under test: the CRC is a table-driven version
 * (the code under test is bitwise), the frames are built with explicit indices, and the deadline is computed in
 * microseconds with literals (500000 bps: 10 bits per byte is 20 us; the margin is 100 %; the drain latency is
 * 500 us; one tick is 100 us).
 */

struct Bytes {
    std::array<uint8_t, 64> data{};
    size_t size{0};

    void push(const uint8_t byte)
    {
        zassert_true(size < data.size(), "test frame too long");
        data[size++] = byte;
    }

    void append(const Bytes &other, const size_t from, const size_t to)
    {
        for (size_t i{from}; i < to; ++i) {
            push(other.data[i]);
        }
    }
};

inline uint16_t ref_crc16(const Bytes &bytes, const size_t count)
{
    static const std::array<uint16_t, 256> table = [] {
        std::array<uint16_t, 256> t{};
        for (uint32_t i{0}; i < 256U; ++i) {
            uint32_t r{i << 8};
            for (int bit{0}; bit < 8; ++bit) {
                r = (r & 0x8000U) != 0U ? ((r << 1) ^ 0x1021U) & 0xFFFFU : (r << 1) & 0xFFFFU;
            }
            t[i] = static_cast<uint16_t>(r);
        }
        return t;
    }();
    uint32_t crc{0xFFFFU};
    for (size_t i{0}; i < count; ++i) {
        crc = ((crc << 8) & 0xFFFFU) ^ table[((crc >> 8) ^ bytes.data[i]) & 0xFFU];
    }
    return static_cast<uint16_t>(crc);
}

/* True when the last two bytes of the first len bytes are the CRC of the bytes before them. */
inline bool ref_window_valid(const Bytes &bytes, const size_t len)
{
    const uint16_t stored{static_cast<uint16_t>((bytes.data[len - 2] << 8) | bytes.data[len - 1])};
    return ref_crc16(bytes, len - 2) == stored;
}

inline Bytes with_crc(Bytes body)
{
    const uint16_t crc{ref_crc16(body, body.size)};
    body.push(static_cast<uint8_t>(crc >> 8));
    body.push(static_cast<uint8_t>(crc & 0xFFU));
    return body;
}

inline Bytes uplink_frame(const uint8_t sender, const uint8_t status, const uint32_t ch1, const uint32_t ch2)
{
    Bytes body;
    body.push(0x4B);
    body.push(12);
    body.push(sender);
    body.push(status);
    body.push(static_cast<uint8_t>(ch1 >> 16));
    body.push(static_cast<uint8_t>(ch1 >> 8));
    body.push(static_cast<uint8_t>(ch1));
    body.push(static_cast<uint8_t>(ch2 >> 16));
    body.push(static_cast<uint8_t>(ch2 >> 8));
    body.push(static_cast<uint8_t>(ch2));
    return with_crc(body);
}

inline Bytes downlink_frame(const uint8_t target, const uint8_t cmd, const uint16_t param)
{
    Bytes body;
    body.push(0x4B);
    body.push(8);
    body.push(target);
    body.push(cmd);
    body.push(static_cast<uint8_t>(param >> 8));
    body.push(static_cast<uint8_t>(param & 0xFFU));
    return with_crc(body);
}

/* Frame of total_len bytes with an arbitrary LEN byte and a valid CRC; the filler never equals the SOF. */
inline Bytes raw_frame(const size_t total_len, const uint8_t len_byte, const uint8_t id, const uint8_t byte3)
{
    Bytes body;
    body.push(0x4B);
    body.push(len_byte);
    body.push(id);
    body.push(byte3);
    for (size_t i{4}; i < total_len - 2; ++i) {
        body.push(static_cast<uint8_t>(0x20 + i));
    }
    return with_crc(body);
}

/* The two reference uplinks (ID0 and ID1) and a downlink to ID1 (a POLL-shaped command). */
inline Bytes frame_id0()
{
    return uplink_frame(0, 0xC0, 0x123456, 0x654321);
}

inline Bytes frame_id1()
{
    return uplink_frame(1, 0xC0, 0x0ABCDE, 0xFEDCBA);
}

inline Bytes frame_downlink()
{
    return downlink_frame(1, 0x10, 0x0000);
}

inline Uplink uplink_of(const uint8_t sender, const uint8_t status, const uint32_t ch1, const uint32_t ch2)
{
    return Uplink{sender, status,
                  {static_cast<uint8_t>(ch1 >> 16), static_cast<uint8_t>(ch1 >> 8), static_cast<uint8_t>(ch1)},
                  {static_cast<uint8_t>(ch2 >> 16), static_cast<uint8_t>(ch2 >> 8), static_cast<uint8_t>(ch2)}};
}

inline Uplink uplink_id0()
{
    return uplink_of(0, 0xC0, 0x123456, 0x654321);
}

inline Uplink uplink_id1()
{
    return uplink_of(1, 0xC0, 0x0ABCDE, 0xFEDCBA);
}

inline bool same_uplink(const Uplink &a, const Uplink &b)
{
    return a.sender_id == b.sender_id && a.status == b.status && a.ch1 == b.ch1 && a.ch2 == b.ch2;
}

/* Deadline of a frame of len bytes in ticks, from the formula in microseconds (see the comment above). */
inline uint32_t oracle_deadline_ticks(const uint32_t len)
{
    const uint32_t frame_us{len * 20U};
    const uint32_t scaled_us100{(frame_us + 20U) * 200U + 100U * 500U};
    return (scaled_us100 + 9999U) / 10000U;
}

struct Expect {
    uint32_t delivered{0};
    uint32_t crc_error{0};
    uint32_t sof_mismatch{0};
    uint32_t timeout{0};
    uint32_t uart_error{0};
    uint32_t foreign_downlink{0};
    uint32_t unknown_sender{0};
};

inline void check_stats(const RxStats &s, const Expect &e)
{
    zassert_equal(s.delivered, e.delivered, "delivered");
    zassert_equal(s.crc_error, e.crc_error, "crc_error");
    zassert_equal(s.sof_mismatch, e.sof_mismatch, "sof_mismatch");
    zassert_equal(s.timeout, e.timeout, "timeout");
    zassert_equal(s.uart_error, e.uart_error, "uart_error");
    zassert_equal(s.foreign_downlink, e.foreign_downlink, "foreign_downlink");
    zassert_equal(s.unknown_sender, e.unknown_sender, "unknown_sender");
}

/* Every counter other than sof_mismatch; the stray bytes after a dropped LEN are counted there (see lcs_rx.hpp). */
inline void check_stats_except_sof(const RxStats &s, Expect e)
{
    e.sof_mismatch = s.sof_mismatch;
    check_stats(s, e);
}

/* A receiver with a record of every uplink it returned. */
struct Rig {
    UplinkReceiver rx{};
    std::array<Uplink, 256> out{};
    size_t out_count{0};

    void feed_byte(const uint8_t byte, const uint32_t tick)
    {
        const auto result{rx.on_byte(byte, tick)};
        if (result.has_value()) {
            zassert_true(out_count < out.size(), "too many uplinks");
            out[out_count++] = *result;
        }
    }

    /* The bytes in [from, to) of a frame, all drained at one tick. */
    void feed(const Bytes &bytes, const uint32_t tick, const size_t from, const size_t to)
    {
        for (size_t i{from}; i < to; ++i) {
            feed_byte(bytes.data[i], tick);
        }
    }

    void feed(const Bytes &bytes, const uint32_t tick)
    {
        feed(bytes, tick, 0, bytes.size);
    }

    void expect_last(const Uplink &uplink, const size_t count) const
    {
        zassert_equal(out_count, count, "returned uplinks");
        zassert_true(same_uplink(out[count - 1], uplink), "returned uplink fields");
    }
};

}
