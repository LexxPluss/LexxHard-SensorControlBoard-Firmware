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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>

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

/* Xorshift generator for the seeded stream tests; the sequence is part of the test and must not change. */
struct Prng {
    uint32_t state;

    uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    uint32_t below(const uint32_t limit)
    {
        return next() % limit;
    }
};

/*
 * Reference model of the candidate set, written independently of UplinkReceiver: it keeps the whole byte history
 * by absolute index (not a ring and a seen counter) and the start index of every candidate. kS21 selects the rule:
 * false is the rule before the nested candidate (every CRC mismatch and every deadline counts), true is the rule of
 * the nested candidate implemented literally:
 *  - a candidate that starts while ANY older candidate in progress (nested or not) has a known LEN and its span
 *    min(LEN, 12) covers the new SOF index is flagged; the LEN byte of the older candidate is applied first;
 *  - a flagged candidate never counts crc_error or timeout (mismatch, deadline, eviction), a match delivers;
 *  - the flag stays when the older candidate leaves; on_uart_error() and reset() clear every candidate.
 */
template <bool kS21>
class RefReceiver {
public:
    std::optional<Uplink> on_byte(const uint8_t byte, const uint32_t tick)
    {
        expire(tick);
        const bool idle{count_ == 0};
        const size_t index{next_index_++};
        history_[index % history_.size()] = byte;
        apply_len_byte(index, byte);
        if (byte == kSofByte) {
            start(index, tick);
        } else if (idle) {
            ++stats_.sof_mismatch;
        }
        return evaluate(index);
    }

    void expire(const uint32_t tick)
    {
        size_t kept{0};
        for (size_t i{0}; i < count_; ++i) {
            if (tick - cands_[i].tick > deadline(cands_[i])) {
                give_up(cands_[i]);
            } else {
                cands_[kept++] = cands_[i];
            }
        }
        count_ = kept;
    }

    void on_uart_error(const uint32_t /* tick */)
    {
        count_ = 0;
        ++stats_.uart_error;
    }

    void reset()
    {
        count_ = 0;
    }

    const RxStats &stats() const
    {
        return stats_;
    }

private:
    static constexpr uint8_t kSofByte{0x4B};
    static constexpr size_t kMinLen{4};
    static constexpr size_t kMaxLenValue{12};
    static constexpr size_t kOversize{13};
    static constexpr size_t kTableSize{12};
    static constexpr size_t kLenUnknown{0};

    struct Cand {
        size_t start;
        uint32_t tick;
        size_t len;
        bool flagged;
    };

    static uint32_t deadline(const Cand &cand)
    {
        const size_t bytes{cand.len == kLenUnknown ? size_t{2} : std::min(cand.len, kMaxLenValue)};
        return oracle_deadline_ticks(static_cast<uint32_t>(bytes));
    }

    /* A flagged candidate is dropped without a counter; the older candidate counts the fault. */
    void give_up(const Cand &cand)
    {
        if (!cand.flagged) {
            ++stats_.timeout;
        }
    }

    void erase(const size_t index)
    {
        for (size_t i{index + 1}; i < count_; ++i) {
            cands_[i - 1] = cands_[i];
        }
        --count_;
    }

    void apply_len_byte(const size_t index, const uint8_t byte)
    {
        size_t kept{0};
        for (size_t i{0}; i < count_; ++i) {
            Cand cand{cands_[i]};
            if (index == cand.start + 1) {
                if (byte < kMinLen) {
                    continue;
                }
                cand.len = byte > kMaxLenValue ? kOversize : byte;
            }
            cands_[kept++] = cand;
        }
        count_ = kept;
    }

    void start(const size_t index, const uint32_t tick)
    {
        if (count_ == cands_.size()) {
            give_up(cands_[0]);
            erase(0);
        }
        bool flagged{false};
        if constexpr (kS21) {
            for (size_t i{0}; i < count_; ++i) {
                const Cand &older{cands_[i]};
                if (older.len != kLenUnknown && index - older.start < std::min(older.len, kMaxLenValue)) {
                    flagged = true;
                }
            }
        }
        cands_[count_++] = Cand{index, tick, kLenUnknown, flagged};
    }

    std::optional<Uplink> evaluate(const size_t index)
    {
        size_t i{0};
        while (i < count_) {
            const Cand cand{cands_[i]};
            if (cand.len < kMinLen || cand.len > kMaxLenValue || index + 1 - cand.start != cand.len) {
                ++i;
                continue;
            }
            Bytes window;
            for (size_t k{0}; k < cand.len; ++k) {
                window.push(history_[(cand.start + k) % history_.size()]);
            }
            if (!ref_window_valid(window, cand.len)) {
                if (!cand.flagged) {
                    ++stats_.crc_error;
                }
                erase(i);
                continue;
            }
            count_ = i;
            return classify(window, cand.len);
        }
        return std::nullopt;
    }

    std::optional<Uplink> classify(const Bytes &window, const size_t len)
    {
        const bool uplink_direction{(window.data[3] & 0x80U) != 0U};
        if (len == 8 && !uplink_direction) {
            ++stats_.foreign_downlink;
        } else if (len == 12 && uplink_direction) {
            if (window.data[2] >= 2) {
                ++stats_.unknown_sender;
                return std::nullopt;
            }
            ++stats_.delivered;
            return Uplink{window.data[2],
                          window.data[3],
                          {window.data[4], window.data[5], window.data[6]},
                          {window.data[7], window.data[8], window.data[9]}};
        }
        return std::nullopt;
    }

    RxStats stats_{};
    std::array<Cand, kTableSize> cands_{};
    size_t count_{0};
    std::array<uint8_t, 64> history_{};
    size_t next_index_{0};
};

/* A receiver (the code under test or a reference model) with a record of every uplink it returned. */
template <typename Receiver>
struct BasicRig {
    Receiver rx{};
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

using Rig = BasicRig<UplinkReceiver>;

template <bool kS21>
using ModelRig = BasicRig<RefReceiver<kS21>>;

/* The uplink that a CRC-valid 12 B uplink frame carries. */
inline Uplink uplink_of_frame(const Bytes &frame)
{
    return Uplink{frame.data[2],
                  frame.data[3],
                  {frame.data[4], frame.data[5], frame.data[6]},
                  {frame.data[7], frame.data[8], frame.data[9]}};
}

/* One byte of a planned stream to be changed: the stream is built by pattern() and then adjusted. */
struct Override {
    size_t index;
    uint8_t value;
};

/*
 * Stream of total bytes. Index 0 is the SOF of the older candidate and index 1 its LEN byte (len_byte); the other
 * bytes are fillers that never equal the SOF, except those set by overrides.
 */
inline Bytes pattern(const size_t total, const uint8_t len_byte, const std::initializer_list<Override> overrides = {})
{
    Bytes bytes;
    for (size_t i{0}; i < total; ++i) {
        bytes.push(i == 0 ? uint8_t{0x4B} : (i == 1 ? len_byte : static_cast<uint8_t>(0x50 + i)));
    }
    for (const Override &o : overrides) {
        zassert_true(o.index < total, "override inside the stream");
        bytes.data[o.index] = o.value;
    }
    return bytes;
}

/* Copies a whole frame into a planned stream at index at. */
inline void overlay(Bytes &stream, const size_t at, const Bytes &frame)
{
    zassert_true(at + frame.size <= stream.size, "overlay inside the stream");
    for (size_t k{0}; k < frame.size; ++k) {
        stream.data[at + k] = frame.data[k];
    }
}

/* The window of len bytes that starts at index start of a planned stream has (or has not) a matching CRC. */
inline void assert_window(const Bytes &stream, const size_t start, const size_t len, const bool valid)
{
    zassert_true(start + len <= stream.size, "window inside the stream");
    Bytes window;
    window.append(stream, start, start + len);
    zassert_equal(ref_window_valid(window, len), valid, "CRC of the window at %u, len %u",
                  static_cast<unsigned>(start), static_cast<unsigned>(len));
}

/*
 * A planned stream and a receiver that is fed from it in steps; every step names the tick at which its bytes are
 * drained.
 */
template <typename RigT>
struct Tape {
    Bytes planned;
    size_t cursor{0};
    RigT rig{};

    explicit Tape(const Bytes &stream) : planned{stream}
    {
    }

    /* Drains the planned bytes up to, excluding, index end at one tick. */
    void feed_until(const size_t end, const uint32_t tick)
    {
        zassert_true(end >= cursor && end <= planned.size, "tape step");
        while (cursor < end) {
            rig.feed_byte(planned.data[cursor++], tick);
        }
    }

    void feed_all(const uint32_t tick)
    {
        feed_until(planned.size, tick);
    }

    void expire(const uint32_t tick)
    {
        rig.rx.expire(tick);
    }

    void expect(const Expect &e) const
    {
        check_stats(rig.rx.stats(), e);
    }
};

}
