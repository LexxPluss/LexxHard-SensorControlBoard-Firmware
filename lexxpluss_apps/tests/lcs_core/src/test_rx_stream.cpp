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

ZTEST_SUITE(lcs_core_rx_stream, NULL, NULL, NULL, NULL, NULL);

static uint32_t d12()
{
    return oracle_deadline_ticks(12);
}

static void expect_monotonic(const RxStats &before, const RxStats &after)
{
    zassert_true(after.delivered >= before.delivered, "delivered");
    zassert_true(after.crc_error >= before.crc_error, "crc_error");
    zassert_true(after.sof_mismatch >= before.sof_mismatch, "sof_mismatch");
    zassert_true(after.timeout >= before.timeout, "timeout");
    zassert_true(after.uart_error >= before.uart_error, "uart_error");
    zassert_true(after.foreign_downlink >= before.foreign_downlink, "foreign_downlink");
}

static bool same_stats(const RxStats &a, const RxStats &b)
{
    return a.delivered == b.delivered && a.crc_error == b.crc_error && a.sof_mismatch == b.sof_mismatch &&
           a.timeout == b.timeout && a.uart_error == b.uart_error && a.foreign_downlink == b.foreign_downlink;
}

ZTEST(lcs_core_rx_stream, test_lcs_core_rx16_counters_are_monotonic)
{
    Rig rig;
    RxStats seen{};
    const auto step = [&] {
        expect_monotonic(seen, rig.rx.stats());
        seen = rig.rx.stats();
    };
    uint32_t tick{1000};

    /* A byte that is not a SOF while no frame is in progress. */
    rig.feed_byte(0x00, tick);
    step();
    check_stats(rig.rx.stats(), Expect{.sof_mismatch = 1});

    tick += 100;
    rig.feed(frame_id0(), tick);
    step();
    tick += 100;
    rig.feed(frame_downlink(), tick);
    step();

    tick += 100;
    Bytes corrupted{frame_id0()};
    corrupted.data[5] = static_cast<uint8_t>(corrupted.data[5] ^ 0x01U);
    rig.feed(corrupted, tick);
    step();

    tick += 100;
    rig.feed(frame_id0(), tick, 0, 3);
    rig.rx.on_uart_error(tick);
    step();

    tick += 100;
    rig.feed(frame_id0(), tick, 0, 5);
    rig.rx.expire(tick + d12() + 1);
    step();
    tick += d12() + 1;
    check_stats(rig.rx.stats(), Expect{.delivered = 1, .crc_error = 1, .sof_mismatch = 1, .timeout = 1,
                                       .uart_error = 1, .foreign_downlink = 1});

    /* reset() drops the frame in progress and keeps every counter. */
    tick += 100;
    rig.feed(frame_id0(), tick, 0, 4);
    const RxStats before_reset{rig.rx.stats()};
    rig.rx.reset();
    zassert_true(same_stats(before_reset, rig.rx.stats()), "reset() must not touch the counters");

    rig.feed(frame_id1(), tick);
    step();
    check_stats(rig.rx.stats(), Expect{.delivered = 2, .crc_error = 1, .sof_mismatch = 1, .timeout = 1,
                                       .uart_error = 1, .foreign_downlink = 1});
}

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

/* The expected uplinks are collected while the stream is built; the receiver is not consulted. */
static void run_random_stream(const uint32_t seed, const bool with_noise)
{
    static std::array<Uplink, 512> expected{};
    size_t expected_count{0};
    uint32_t injected_downlinks{0};
    uint32_t injected_unknown{0};

    Prng prng{seed};
    Rig rig;
    uint32_t now_us{0};

    const auto send = [&](const Bytes &frame) {
        for (size_t i{0}; i < frame.size; ++i) {
            rig.feed_byte(frame.data[i], now_us / 100U);
            now_us += 20U;
        }
    };

    /*
     * Without noise every field byte equal to the SOF is replaced: a SOF followed by a LEN of 4..7 inside a frame
     * opens a window that completes before the frame does and counts crc_error, which is correct but would hide a
     * real error in the "no error counter moves" check.
     */
    const auto scrub = [&](const uint32_t value) {
        uint32_t result{0};
        for (int shift{0}; shift < 32; shift += 8) {
            uint32_t byte{(value >> shift) & 0xFFU};
            if (!with_noise && byte == 0x4BU) {
                byte = 0x4CU;
            }
            result |= byte << shift;
        }
        return result;
    };

    for (int event{0}; event < 300; ++event) {
        /* A gap of 0 ticks is the next byte 20 us after the last one. */
        const uint32_t gap_ticks{prng.below(4) == 0 ? 0U : prng.below(30)};
        now_us += gap_ticks * 100U;
        const uint32_t kind{prng.below(with_noise ? 11U : 9U)};
        if (kind < 4) {
            const uint8_t sender{static_cast<uint8_t>(prng.below(2))};
            const uint8_t status{static_cast<uint8_t>(0x80U | prng.below(0x80))};
            const uint32_t ch1{scrub(prng.next() & 0xFFFFFFU)};
            const uint32_t ch2{scrub(prng.next() & 0xFFFFFFU)};
            send(uplink_frame(sender, status, ch1, ch2));
            expected[expected_count++] = uplink_of(sender, status, ch1, ch2);
        } else if (kind < 7) {
            const uint32_t target{scrub(prng.below(256))};
            const uint32_t cmd{scrub(prng.below(128))};
            const uint32_t param{scrub(prng.below(0x10000))};
            send(downlink_frame(static_cast<uint8_t>(target), static_cast<uint8_t>(cmd), static_cast<uint16_t>(param)));
            ++injected_downlinks;
        } else if (kind < 9) {
            const uint8_t sender{static_cast<uint8_t>(2U + prng.below(254))};
            const uint8_t status{static_cast<uint8_t>(0x80U | prng.below(0x80))};
            const uint32_t ch1{scrub(prng.next() & 0xFFFFFFU)};
            const uint32_t ch2{scrub(prng.next() & 0xFFFFFFU)};
            send(uplink_frame(sender, status, ch1, ch2));
            ++injected_unknown;
        } else {
            const uint32_t count{1U + prng.below(6)};
            for (uint32_t i{0}; i < count; ++i) {
                const uint8_t byte{prng.below(4) == 0 ? uint8_t{0x4B} : static_cast<uint8_t>(prng.below(256))};
                rig.feed_byte(byte, now_us / 100U);
                now_us += 20U;
            }
        }
    }
    rig.rx.expire(now_us / 100U + 1000U);

    zassert_equal(rig.out_count, expected_count, "returned uplinks, seed %u", static_cast<unsigned>(seed));
    for (size_t i{0}; i < expected_count; ++i) {
        zassert_true(same_uplink(rig.out[i], expected[i]), "uplink %u, seed %u", static_cast<unsigned>(i),
                     static_cast<unsigned>(seed));
    }
    const RxStats &stats{rig.rx.stats()};
    zassert_equal(stats.delivered, expected_count);
    zassert_equal(stats.foreign_downlink, injected_downlinks, "foreign downlinks, seed %u",
                  static_cast<unsigned>(seed));
    zassert_equal(stats.unknown_sender, injected_unknown, "unknown senders");
    if (!with_noise) {
        zassert_equal(stats.crc_error, 0U);
        zassert_equal(stats.sof_mismatch, 0U);
        zassert_equal(stats.timeout, 0U);
        zassert_equal(stats.uart_error, 0U);
    }
}

ZTEST(lcs_core_rx_stream, test_lcs_core_rx17_random_stream_without_noise)
{
    run_random_stream(0x12345678U, false);
    run_random_stream(0xCAFEBABEU, false);
}

ZTEST(lcs_core_rx_stream, test_lcs_core_rx17_random_stream_with_noise)
{
    run_random_stream(0x0BADF00DU, true);
    run_random_stream(0x31415926U, true);
}

enum class Previous { ValidUplink, ValidDownlink, UartError, Truncated, StrayInCorruptedWindow };
enum class Next { Id0, Id1, Downlink };

static Bytes next_frame(const Next next)
{
    switch (next) {
    case Next::Id0:
        return frame_id0();
    case Next::Id1:
        return frame_id1();
    case Next::Downlink:
        break;
    }
    return frame_downlink();
}

/* A valid ID0 uplink with a SOF at byte 5 and a LEN of 12 at byte 6: its CRC fails, the inner window stays open. */
static Bytes corrupted_with_inner_sof()
{
    Bytes corrupted{frame_id0()};
    corrupted.data[5] = 0x4B;
    corrupted.data[6] = 12;
    return corrupted;
}

/* Runs one case of the adjacency matrix; the previous event is drained at tick 1000, the next frame at 1000 + gap. */
static void run_adjacency(const Previous previous, const Next next, const uint32_t gap)
{
    const uint32_t tick0{1000};
    const Bytes following{next_frame(next)};
    Rig rig;
    Expect expect{};
    size_t out_count{0};
    bool candidate_left{false};

    switch (previous) {
    case Previous::ValidUplink:
        rig.feed(frame_id0(), tick0);
        ++expect.delivered;
        ++out_count;
        break;
    case Previous::ValidDownlink:
        rig.feed(frame_downlink(), tick0);
        ++expect.foreign_downlink;
        break;
    case Previous::UartError:
        rig.feed(frame_id0(), tick0, 0, 6);
        rig.rx.on_uart_error(tick0);
        ++expect.uart_error;
        break;
    case Previous::Truncated: {
        Bytes window;
        window.append(frame_id0(), 0, 5);
        window.append(following, 0, 7);
        zassert_false(ref_window_valid(window, 12), "the window of the truncated frame must be corrupted");
        rig.feed(frame_id0(), tick0, 0, 5);
        candidate_left = true;
        break;
    }
    case Previous::StrayInCorruptedWindow: {
        const Bytes corrupted{corrupted_with_inner_sof()};
        zassert_false(ref_window_valid(corrupted, 12), "the outer window must be corrupted");
        Bytes window;
        window.append(corrupted, 5, 12);
        window.append(following, 0, 5);
        zassert_false(ref_window_valid(window, 12), "the inner window must be corrupted");
        rig.feed(corrupted, tick0);
        ++expect.crc_error;
        candidate_left = true;
        break;
    }
    }
    check_stats(rig.rx.stats(), expect);

    /* The old candidate either completes inside the next frame (CRC mismatch) or has expired before its first byte. */
    if (candidate_left) {
        if (gap <= d12()) {
            ++expect.crc_error;
        } else {
            ++expect.timeout;
        }
    }
    rig.feed(following, tick0 + gap);
    if (next == Next::Downlink) {
        ++expect.foreign_downlink;
    } else {
        ++expect.delivered;
        ++out_count;
    }
    check_stats(rig.rx.stats(), expect);
    zassert_equal(rig.out_count, out_count);
    if (next != Next::Downlink) {
        zassert_true(same_uplink(rig.out[out_count - 1], next == Next::Id0 ? uplink_id0() : uplink_id1()));
    }
    rig.rx.expire(tick0 + gap + d12() + 1);
    zassert_equal(rig.rx.stats().timeout, expect.timeout, "no candidate may be left behind");
}

/* Previous event leaves no state behind: the gap does not matter, so only gap 0 is run. */
ZTEST(lcs_core_rx_stream, test_lcs_core_rx18_loop1_no_residual_state)
{
    size_t executed{0};
    for (const Previous previous : {Previous::ValidUplink, Previous::ValidDownlink, Previous::UartError}) {
        for (const Next next : {Next::Id0, Next::Id1, Next::Downlink}) {
            run_adjacency(previous, next, 0);
            ++executed;
        }
    }
    zassert_equal(executed, 9U);
}

/* Previous event leaves a candidate in progress: the gap to the deadline decides whether it is still alive. */
ZTEST(lcs_core_rx_stream, test_lcs_core_rx18_loop2_residual_state)
{
    size_t executed{0};
    for (const Previous previous : {Previous::Truncated, Previous::StrayInCorruptedWindow}) {
        for (const Next next : {Next::Id0, Next::Id1, Next::Downlink}) {
            for (const uint32_t gap : {d12(), d12() + 1, 0U}) {
                run_adjacency(previous, next, gap);
                ++executed;
            }
        }
    }
    zassert_equal(executed, 18U);
}
