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

ZTEST_SUITE(lcs_rx_action_shaped, NULL, NULL, NULL, NULL, NULL);

namespace {

constexpr std::array<uint8_t, 3> kActionCmds{0x48, 0x50, 0x68};
constexpr std::array<uint8_t, 3> kActionTargets{0, 1, 0xFF};

uint32_t d12()
{
    return oracle_deadline_ticks(12);
}

/* The valid uplink that follows an ACTION-shaped frame. */
void expect_aftermath(Rig &rig, const uint32_t tick, const Expect &before, const size_t out_before)
{
    rig.feed(frame_id1(), tick);
    Expect after{before};
    ++after.delivered;
    check_stats(rig.rx.stats(), after);
    rig.expect_last(uplink_id1(), out_before + 1);
}

}  // namespace

/* The own ACTION frame (direction bit 0) looks like a foreign downlink to the receiver. */
ZTEST(lcs_rx_action_shaped, test_sc03_01_action_frame_counts_foreign_downlink)
{
    for (const uint8_t cmd : kActionCmds) {
        for (const uint8_t target : kActionTargets) {
            for (const uint16_t param : {uint16_t{0x0000}, uint16_t{0xFFFF}}) {
                if (param == 0xFFFF && target != 0xFF) {
                    continue;
                }
                Rig rig;
                rig.feed(downlink_frame(target, cmd, param), 100);
                zassert_equal(rig.out_count, 0U);
                check_stats(rig.rx.stats(), Expect{.foreign_downlink = 1});
            }
        }
    }
}

ZTEST(lcs_rx_action_shaped, test_sc03_02_direction_bit_set_is_dropped_silently)
{
    for (const uint8_t cmd : {uint8_t{0xC8}, uint8_t{0xD0}, uint8_t{0xE8}}) {
        for (const uint8_t target : kActionTargets) {
            for (const uint16_t param : {uint16_t{0x0000}, uint16_t{0xFFFF}}) {
                if (param == 0xFFFF && target != 0xFF) {
                    continue;
                }
                Rig rig;
                rig.feed(downlink_frame(target, cmd, param), 100);
                zassert_equal(rig.out_count, 0U);
                check_stats(rig.rx.stats(), Expect{});
            }
        }
    }
}

ZTEST(lcs_rx_action_shaped, test_sc03_03_uplink_after_action_frame_is_delivered)
{
    for (const uint32_t gap : {uint32_t{0}, d12() - 1, d12(), d12() + 1}) {
        Rig rig;
        rig.feed(downlink_frame(0, 0x48, 0x0000), 100);
        expect_aftermath(rig, 100 + gap, Expect{.foreign_downlink = 1}, 0);
    }
}

ZTEST(lcs_rx_action_shaped, test_sc03_05_action_frame_with_bad_crc_counts_crc_error)
{
    Bytes bad{downlink_frame(1, 0x48, 0x0000)};
    bad.data[bad.size - 1] ^= 0x01U;
    Rig rig;
    rig.feed(bad, 100);
    zassert_equal(rig.out_count, 0U);
    check_stats(rig.rx.stats(), Expect{.crc_error = 1});
    rig.feed(frame_id0(), 100);
    rig.expect_last(uplink_id0(), 1);
    check_stats(rig.rx.stats(), Expect{.delivered = 1, .crc_error = 1});
}

/*
 * PINS THE CURRENT LOSS: an uplink whose SOF and LEN are the last two (CRC) bytes of an ACTION frame is
 * lost, because the valid ACTION frame discards the nested candidate. This case documents today's behaviour
 * of lcs_rx; it is to be changed together with lcs_rx if the loss proves harmful.
 */
ZTEST(lcs_rx_action_shaped, test_sc03_04_uplink_sof_inside_action_span_is_lost)
{
    Bytes action;
    bool found{false};
    for (const uint8_t target : kActionTargets) {
        for (const uint8_t cmd : kActionCmds) {
            for (uint32_t param{0}; param <= 0xFFFFU && !found; ++param) {
                Bytes body;
                body.push(0x4B);
                body.push(8);
                body.push(target);
                body.push(cmd);
                body.push(static_cast<uint8_t>(param >> 8));
                body.push(static_cast<uint8_t>(param & 0xFFU));
                if (ref_crc16(body, body.size) == 0x4B0C) {
                    action = body;
                    found = true;
                }
            }
            if (found) {
                break;
            }
        }
        if (found) {
            break;
        }
    }
    zassert_true(found, "no ACTION body with CRC 0x4B0C");

    /* The uplink starts at stream index 6; its bytes at stream indices 8..17 must not contain a SOF. */
    const Bytes uplink{uplink_frame(0, 0xC0, 0x010203, 0x050607)};
    zassert_equal(uplink.size, 12U);
    for (size_t i{2}; i < uplink.size; ++i) {
        zassert_not_equal(uplink.data[i], 0x4B, "SOF byte inside the uplink tail");
    }

    Bytes stream{action};
    stream.append(uplink, 0, uplink.size);
    zassert_equal(stream.size, 18U);
    zassert_equal(stream.data[6], 0x4B);
    zassert_equal(stream.data[7], 0x0C);

    Rig rig;
    rig.feed(stream, 100);
    zassert_equal(rig.out_count, 0U);
    check_stats_except_sof(rig.rx.stats(), Expect{.foreign_downlink = 1});

    rig.feed(frame_id1(), 100 + d12() + 1);
    rig.expect_last(uplink_id1(), 1);
    check_stats_except_sof(rig.rx.stats(), Expect{.delivered = 1, .foreign_downlink = 1});
}
