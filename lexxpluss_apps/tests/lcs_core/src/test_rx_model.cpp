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

ZTEST_SUITE(lcs_core_rx_model, NULL, NULL, NULL, NULL, NULL);

static uint8_t noise_byte(Prng &prng)
{
    const uint32_t pick{prng.below(10)};
    if (pick < 4) {
        return 0x4B;
    }
    if (pick < 6) {
        return static_cast<uint8_t>(prng.below(16));
    }
    return static_cast<uint8_t>(prng.below(256));
}

/* Valid frames of every LEN class the receiver tells apart, with known and unknown IDs. */
static Bytes random_frame(Prng &prng)
{
    const uint8_t id{static_cast<uint8_t>(prng.below(4))};
    switch (prng.below(8)) {
    case 0: {
        const uint8_t status{static_cast<uint8_t>(0x80U | prng.below(0x80))};
        const uint32_t ch1{prng.next() & 0xFFFFFFU};
        const uint32_t ch2{prng.next() & 0xFFFFFFU};
        return uplink_frame(id, status, ch1, ch2);
    }
    case 1: {
        const uint8_t cmd{static_cast<uint8_t>(prng.below(128))};
        const uint16_t param{static_cast<uint16_t>(prng.below(0x10000))};
        return downlink_frame(id, cmd, param);
    }
    case 2:
        return raw_frame(9, 9, id, 0x11);
    case 3:
        return raw_frame(11, 11, id, 0xC1);
    case 4:
        return raw_frame(7, 7, id, 0x11);
    case 5:
        return raw_frame(6, 6, id, 0x11);
    case 6:
        return raw_frame(12, 12, id, 0x11);
    default:
        break;
    }
    return raw_frame(8, 8, id, 0xC1);
}

static void expect_same(const RxStats &a, const RxStats &b, const uint32_t seed, const size_t event)
{
    zassert_equal(a.delivered, b.delivered, "delivered, seed %u event %u", static_cast<unsigned>(seed),
                  static_cast<unsigned>(event));
    zassert_equal(a.crc_error, b.crc_error, "crc_error, seed %u event %u", static_cast<unsigned>(seed),
                  static_cast<unsigned>(event));
    zassert_equal(a.sof_mismatch, b.sof_mismatch, "sof_mismatch, seed %u event %u", static_cast<unsigned>(seed),
                  static_cast<unsigned>(event));
    zassert_equal(a.timeout, b.timeout, "timeout, seed %u event %u", static_cast<unsigned>(seed),
                  static_cast<unsigned>(event));
    zassert_equal(a.uart_error, b.uart_error, "uart_error, seed %u event %u", static_cast<unsigned>(seed),
                  static_cast<unsigned>(event));
    zassert_equal(a.foreign_downlink, b.foreign_downlink, "foreign_downlink, seed %u event %u",
                  static_cast<unsigned>(seed), static_cast<unsigned>(event));
    zassert_equal(a.unknown_sender, b.unknown_sender, "unknown_sender, seed %u event %u", static_cast<unsigned>(seed),
                  static_cast<unsigned>(event));
}

/*
 * Dense random events (bursts with many SOF bytes, valid and corrupted frames of every LEN class, truncated frames,
 * floods of SOF bytes that fill the candidate table, UART errors, resets, gaps around the deadlines): the model
 * with the nested rule and the receiver under test see the same events and must agree on every counter and on every
 * returned uplink after every event. The model without the rule differs only in crc_error and timeout, and only
 * upwards.
 */
static void run_dense(const uint32_t seed)
{
    static Rig dut;
    static ModelRig<false> model_off;
    static ModelRig<true> model_on;
    dut = Rig{};
    model_off = ModelRig<false>{};
    model_on = ModelRig<true>{};
    Prng prng{seed};
    uint32_t tick{1000};

    const auto drain = [&](const uint8_t byte) {
        dut.feed_byte(byte, tick);
        model_off.feed_byte(byte, tick);
        model_on.feed_byte(byte, tick);
    };

    for (size_t event{0}; event < 1500; ++event) {
        tick += prng.below(4) == 0 ? 0U : prng.below(15);
        switch (prng.below(12)) {
        case 0:
        case 1:
        case 2:
        case 3: {
            const uint32_t count{1U + prng.below(14)};
            for (uint32_t i{0}; i < count; ++i) {
                drain(noise_byte(prng));
            }
            break;
        }
        case 4:
        case 5:
        case 6: {
            Bytes frame{random_frame(prng)};
            if (prng.below(4) == 0) {
                const size_t at{prng.below(static_cast<uint32_t>(frame.size))};
                frame.data[at] = static_cast<uint8_t>(frame.data[at] ^ (1U << prng.below(8)));
            }
            size_t count{frame.size};
            if (prng.below(3) == 0) {
                count = 1U + prng.below(static_cast<uint32_t>(frame.size));
            }
            for (size_t i{0}; i < count; ++i) {
                drain(frame.data[i]);
            }
            break;
        }
        case 7:
            if (prng.below(5) == 0) {
                dut.rx.on_uart_error(tick);
                model_off.rx.on_uart_error(tick);
                model_on.rx.on_uart_error(tick);
            } else {
                drain(0x4B);
            }
            break;
        case 8:
            if (prng.below(5) == 0) {
                dut.rx.reset();
                model_off.rx.reset();
                model_on.rx.reset();
            } else {
                drain(0x4B);
            }
            break;
        case 9:
            tick += prng.below(14);
            dut.rx.expire(tick);
            model_off.rx.expire(tick);
            model_on.rx.expire(tick);
            break;
        default: {
            const uint32_t count{1U + prng.below(30)};
            for (uint32_t i{0}; i < count; ++i) {
                drain(0x4B);
                if (prng.below(2) == 0) {
                    drain(0x80);
                }
            }
            break;
        }
        }
        expect_same(dut.rx.stats(), model_on.rx.stats(), seed, event);
        zassert_equal(dut.out_count, model_on.out_count, "returned uplinks, seed %u event %u",
                      static_cast<unsigned>(seed), static_cast<unsigned>(event));
        const RxStats &off{model_off.rx.stats()};
        const RxStats &on{model_on.rx.stats()};
        zassert_equal(on.delivered, off.delivered, "delivered with the rule, seed %u", static_cast<unsigned>(seed));
        zassert_equal(on.sof_mismatch, off.sof_mismatch, "sof_mismatch with the rule");
        zassert_equal(on.uart_error, off.uart_error, "uart_error with the rule");
        zassert_equal(on.foreign_downlink, off.foreign_downlink, "foreign_downlink with the rule");
        zassert_equal(on.unknown_sender, off.unknown_sender, "unknown_sender with the rule");
        zassert_true(on.crc_error <= off.crc_error, "crc_error with the rule, seed %u", static_cast<unsigned>(seed));
        zassert_true(on.timeout <= off.timeout, "timeout with the rule, seed %u", static_cast<unsigned>(seed));
    }
    dut.rx.expire(tick + 100);
    model_off.rx.expire(tick + 100);
    model_on.rx.expire(tick + 100);
    expect_same(dut.rx.stats(), model_on.rx.stats(), seed, 1500);
    for (size_t i{0}; i < dut.out_count; ++i) {
        zassert_true(same_uplink(dut.out[i], model_on.out[i]), "uplink %u, seed %u", static_cast<unsigned>(i),
                     static_cast<unsigned>(seed));
    }
}

/* The receiver equals the model with the nested rule on eviction, LEN < 4, LEN 4..11, classify and expiry. */
ZTEST(lcs_core_rx_model, test_lcs_core_model_with_nested_rule_equals_receiver_dense_random)
{
    for (const uint32_t seed : {0xA5A5A5A5U, 0x13579BDFU, 0x2468ACE0U, 0xDEADBEEFU}) {
        run_dense(seed);
    }
}
