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

#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>

#include "lcs_request.hpp"

namespace lcs = lexxhard::lcs;

namespace {

constexpr std::array<uint8_t, 3> kValidCmds{0x48, 0x50, 0x68};
constexpr std::array<uint8_t, 3> kValidTargets{0x00, 0x01, 0xFF};

can_frame make_frame(uint8_t cmd, uint8_t target, uint16_t param)
{
    can_frame frame{};
    frame.id = lcs::kRequestId;
    frame.dlc = 8;
    frame.flags = 0;
    frame.data[0] = cmd;
    frame.data[1] = target;
    frame.data[2] = static_cast<uint8_t>(param >> 8);
    frame.data[3] = static_cast<uint8_t>(param & 0xFF);
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    return frame;
}

bool oracle_accepts(const can_frame &frame)
{
    bool cmd_ok = false;
    for (const uint8_t c : kValidCmds) {
        cmd_ok = cmd_ok || frame.data[0] == c;
    }
    bool target_ok = false;
    for (const uint8_t t : kValidTargets) {
        target_ok = target_ok || frame.data[1] == t;
    }
    return frame.id == 0x21E && frame.flags == 0 && frame.dlc == 8 && cmd_ok && target_ok;
}

uint32_t xorshift32(uint32_t &state)
{
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
}

lcs::RequestStats numbered_stats()
{
    lcs::RequestStats stats{};
    stats.received = 1;
    stats.queued = 2;
    stats.dropped_invalid = 3;
    stats.dropped_full = 4;
    stats.zero_reset = 10;
    stats.temp_drift_calib = 20;
    stats.abort_cmd = 30;
    return stats;
}

void expect_stats(const lcs::RequestStats &actual, const lcs::RequestStats &expected)
{
    zassert_equal(actual.received, expected.received, "received");
    zassert_equal(actual.queued, expected.queued, "queued");
    zassert_equal(actual.dropped_invalid, expected.dropped_invalid, "dropped_invalid");
    zassert_equal(actual.dropped_full, expected.dropped_full, "dropped_full");
    zassert_equal(actual.zero_reset, expected.zero_reset, "zero_reset");
    zassert_equal(actual.temp_drift_calib, expected.temp_drift_calib, "temp_drift_calib");
    zassert_equal(actual.abort_cmd, expected.abort_cmd, "abort_cmd");
}

} // namespace

ZTEST_SUITE(lcs_request, NULL, NULL, NULL, NULL, NULL);

// Catches a decoder that rejects part of the valid cmd/target product or mixes up the fields.
ZTEST(lcs_request, test_sc01_01_nine_valid_pairs)
{
    for (const uint8_t cmd : kValidCmds) {
        for (const uint8_t target : kValidTargets) {
            const can_frame frame = make_frame(cmd, target, 0);
            const std::optional<lcs::Request> req = lcs::decode_request(frame);
            zassert_true(req.has_value(), "cmd 0x%02x target 0x%02x rejected", cmd, target);
            zassert_equal(req->cmd, cmd);
            zassert_equal(req->target, target);
            zassert_equal(req->param, 0);
        }
    }
}

// Catches an unknown command code (notably SYNC 0x08) being relayed to the LCS.
ZTEST(lcs_request, test_sc01_02_invalid_cmd_dropped)
{
    constexpr std::array<uint8_t, 9> invalid_cmds{0x08, 0x00, 0x47, 0x49, 0x4F, 0x51, 0x67, 0x69, 0xFF};
    for (const uint8_t cmd : invalid_cmds) {
        for (const uint8_t target : kValidTargets) {
            const can_frame frame = make_frame(cmd, target, 0);
            zassert_false(lcs::decode_request(frame).has_value(), "cmd 0x%02x accepted", cmd);
        }
    }
}

// Catches a target check that is missing or too loose (range check instead of set check).
ZTEST(lcs_request, test_sc01_03_invalid_target_dropped)
{
    constexpr std::array<uint8_t, 4> invalid_targets{2, 3, 0x7F, 0xFE};
    for (const uint8_t cmd : kValidCmds) {
        for (const uint8_t target : invalid_targets) {
            const can_frame frame = make_frame(cmd, target, 0);
            zassert_false(lcs::decode_request(frame).has_value(), "target 0x%02x accepted", target);
        }
    }
    const can_frame both_bad = make_frame(0x00, 2, 0);
    zassert_false(lcs::decode_request(both_bad).has_value());
}

// Catches a decoder that reads data[] beyond the received length or ignores the DLC.
ZTEST(lcs_request, test_sc01_04_dlc)
{
    constexpr std::array<uint8_t, 9> bad_dlcs{0, 1, 2, 3, 4, 5, 6, 7, 9};
    for (const uint8_t dlc : bad_dlcs) {
        can_frame frame = make_frame(0x48, 0, 0);
        frame.dlc = dlc;
        zassert_false(lcs::decode_request(frame).has_value(), "dlc %u accepted", dlc);
    }
    can_frame frame15 = make_frame(0x48, 0, 0);
    frame15.dlc = 15;
    zassert_false(lcs::decode_request(frame15).has_value());

    can_frame frame8 = make_frame(0x48, 0, 0);
    frame8.dlc = 8;
    zassert_true(lcs::decode_request(frame8).has_value());
}

// Catches extended-ID, remote and FD frames with the same 11-bit ID being treated as requests.
ZTEST(lcs_request, test_sc01_05_flags_dropped)
{
    constexpr std::array<uint8_t, 3> bad_flags{CAN_FRAME_IDE, CAN_FRAME_RTR, CAN_FRAME_FDF};
    for (const uint8_t flags : bad_flags) {
        can_frame frame = make_frame(0x48, 0, 0);
        frame.flags = flags;
        zassert_false(lcs::decode_request(frame).has_value(), "flags 0x%02x accepted", flags);
    }
    can_frame ok = make_frame(0x48, 0, 0);
    ok.flags = 0;
    zassert_true(lcs::decode_request(ok).has_value());
}

// Catches an ID match that is off by one or only compares part of the identifier.
ZTEST(lcs_request, test_sc01_06_other_ids_dropped)
{
    constexpr std::array<uint16_t, 4> other_ids{0x21C, 0x21D, 0x21F, 0x21B};
    for (const uint16_t id : other_ids) {
        can_frame frame = make_frame(0x48, 0, 0);
        frame.id = id;
        zassert_false(lcs::decode_request(frame).has_value(), "id 0x%03x accepted", id);
    }
}

// Catches reserved byte 7 being ignored or carried, which would relay frames of an unknown format.
ZTEST(lcs_request, test_sc01_07_byte7_reserved)
{
    constexpr std::array<uint8_t, 4> values{0x01, 0x7F, 0x80, 0xFF};
    for (const uint8_t value : values) {
        can_frame frame = make_frame(0x50, 1, 0);
        frame.data[7] = value;
        zassert_false(lcs::decode_request(frame).has_value(), "byte 7 = 0x%02x accepted", value);
    }
}

// Catches wrong byte order or truncation of the 16-bit param.
ZTEST(lcs_request, test_sc01_08_param_carried)
{
    constexpr std::array<uint16_t, 4> params{0x0000, 0x0001, 0x1234, 0xFFFF};
    for (const uint16_t param : params) {
        const can_frame frame = make_frame(0x68, 0xFF, param);
        const std::optional<lcs::Request> req = lcs::decode_request(frame);
        zassert_true(req.has_value(), "param 0x%04x rejected", param);
        zassert_equal(req->param, param);
    }
}

// Catches reserved bytes 4-6 being ignored, which would relay frames of an unknown format.
ZTEST(lcs_request, test_sc01_08b_reserved_must_be_zero)
{
    constexpr std::array<uint8_t, 2> values{0x01, 0xFF};
    for (const uint8_t value : values) {
        for (std::size_t index = 4; index <= 6; ++index) {
            can_frame frame = make_frame(0x48, 0, 0);
            frame.data[index] = value;
            zassert_false(lcs::decode_request(frame).has_value(), "byte %u = 0x%02x accepted",
                          static_cast<unsigned>(index), value);
        }
    }
    can_frame all_set = make_frame(0x48, 0, 0);
    all_set.data[4] = 0x01;
    all_set.data[5] = 0x01;
    all_set.data[6] = 0x01;
    zassert_false(lcs::decode_request(all_set).has_value());

    const can_frame all_zero = make_frame(0x48, 0, 0);
    zassert_true(lcs::decode_request(all_zero).has_value());
}

// Catches any disagreement between the decoder and an independent statement of the accept rule
// on a mixed stream of valid and invalid frames.
ZTEST(lcs_request, test_sc01_09_seeded_random_against_oracle)
{
    uint32_t state = 0x1D2C3B4A;
    int accepted = 0;
    for (int i = 0; i < 10000; ++i) {
        can_frame frame{};
        frame.id = (xorshift32(state) % 4 != 0) ? 0x21E : (xorshift32(state) % 0x800);
        if (xorshift32(state) % 4 != 0) {
            frame.flags = 0;
        } else {
            const uint32_t pick = xorshift32(state) % 8;
            frame.flags = static_cast<uint8_t>(((pick & 1) ? CAN_FRAME_IDE : 0) |
                                               ((pick & 2) ? CAN_FRAME_RTR : 0) |
                                               ((pick & 4) ? CAN_FRAME_FDF : 0));
        }
        frame.dlc = (xorshift32(state) % 4 != 0) ? 8 : static_cast<uint8_t>(xorshift32(state) % 16);
        frame.data[0] = (xorshift32(state) % 4 != 0) ? kValidCmds[xorshift32(state) % kValidCmds.size()]
                                                     : static_cast<uint8_t>(xorshift32(state));
        frame.data[1] = (xorshift32(state) % 4 != 0)
                            ? kValidTargets[xorshift32(state) % kValidTargets.size()]
                            : static_cast<uint8_t>(xorshift32(state));
        frame.data[2] = 0;
        frame.data[3] = 0;
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 0;

        const bool expected = oracle_accepts(frame);
        const std::optional<lcs::Request> req = lcs::decode_request(frame);
        zassert_equal(req.has_value(), expected, "frame %d: decoder and oracle disagree", i);
        if (expected) {
            ++accepted;
            zassert_equal(req->cmd, frame.data[0]);
            zassert_equal(req->target, frame.data[1]);
            zassert_equal(req->param, 0);
        }
    }
    /* Each of the five fields is valid with probability 3/4 (a little more by chance): about 2540 of 10000. */
    zassert_true(accepted >= 1500 && accepted <= 4000, "accepted %d outside sanity range", accepted);
}

// 0x0001 sets only data[3] and 0x0100 only data[2], so a check of one byte is caught.
// The LCS ignores such an ACTION.
ZTEST(lcs_request, test_sc01_10_param_must_be_zero_for_zero_reset_and_temp_drift)
{
    constexpr std::array<uint8_t, 2> cmds{0x48, 0x50};
    constexpr std::array<uint16_t, 3> params{0x0001, 0x0100, 0xFFFF};
    for (const uint8_t cmd : cmds) {
        for (const uint8_t target : kValidTargets) {
            for (const uint16_t param : params) {
                const can_frame frame = make_frame(cmd, target, param);
                zassert_false(lcs::decode_request(frame).has_value(),
                              "cmd 0x%02x target 0x%02x param 0x%04x must be dropped", cmd, target, param);
            }
        }
    }
}

// Catches a decoder that applies the Param rule to the wrong cmd or only to some param bits.
ZTEST(lcs_request, test_sc01_10b_seeded_random_param_rule)
{
    uint32_t state = 0x2F6E2B1D;
    int accepted = 0;
    for (int i = 0; i < 10000; ++i) {
        const uint8_t cmd = kValidCmds[xorshift32(state) % kValidCmds.size()];
        const uint8_t target = kValidTargets[xorshift32(state) % kValidTargets.size()];
        const uint16_t param =
            (xorshift32(state) % 2 == 0) ? 0 : static_cast<uint16_t>(1 + xorshift32(state) % 0xFFFF);
        can_frame frame{};
        frame.id = 0x21E;
        frame.flags = 0;
        frame.dlc = 8;
        frame.data[0] = cmd;
        frame.data[1] = target;
        frame.data[2] = static_cast<uint8_t>(param >> 8);
        frame.data[3] = static_cast<uint8_t>(param & 0xFF);
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 0;

        const bool expected = (cmd == 0x68) || (param == 0);
        const std::optional<lcs::Request> req = lcs::decode_request(frame);
        zassert_equal(req.has_value(), expected, "frame %d: cmd 0x%02x param 0x%04x", i, cmd, param);
        if (expected) {
            ++accepted;
            zassert_equal(req->cmd, cmd);
            zassert_equal(req->target, target);
            zassert_equal(req->param, param);
        }
    }
    /* Accepted with probability 1/3 + 2/3 * 1/2 = 2/3 (about 6667, sigma 47); a plain uniform 16-bit param would
     * give only ~1/3. */
    zassert_true(accepted >= 6000 && accepted <= 7300, "accepted %d outside expected range", accepted);
}

// Catches swapped fields, a field never incremented, or several fields incremented for one cmd.
ZTEST(lcs_request, test_sc12_01_count_each_valid_cmd)
{
    {
        lcs::RequestStats stats = numbered_stats();
        lcs::count_request_cmd(stats, 0x48);
        lcs::RequestStats expected = numbered_stats();
        expected.zero_reset = 11;
        expect_stats(stats, expected);
    }
    {
        lcs::RequestStats stats = numbered_stats();
        lcs::count_request_cmd(stats, 0x50);
        lcs::RequestStats expected = numbered_stats();
        expected.temp_drift_calib = 21;
        expect_stats(stats, expected);
    }
    {
        lcs::RequestStats stats = numbered_stats();
        lcs::count_request_cmd(stats, 0x68);
        lcs::RequestStats expected = numbered_stats();
        expected.abort_cmd = 31;
        expect_stats(stats, expected);
    }
}

// Catches a default branch that counts an unknown cmd, notably SYNC 0x08, as one of the three.
ZTEST(lcs_request, test_sc12_02_other_cmd_values_count_nothing)
{
    constexpr std::array<uint8_t, 5> other_cmds{0x00, 0x08, 0x49, 0x67, 0xFF};
    for (const uint8_t cmd : other_cmds) {
        lcs::RequestStats stats = numbered_stats();
        lcs::count_request_cmd(stats, cmd);
        expect_stats(stats, numbered_stats());
    }
}

// Catches the cmd count being lost on the success path of the queue.
ZTEST(lcs_request, test_sc12_03_put_true_counts_cmd_and_queued)
{
    for (const uint8_t cmd : kValidCmds) {
        lcs::RequestStats stats{};
        lcs::enqueue_request([](const lcs::Request &) { return true; }, lcs::Request{cmd, 0, 0}, stats);
        zassert_equal(stats.zero_reset, cmd == 0x48 ? 1U : 0U, "cmd 0x%02x zero_reset", cmd);
        zassert_equal(stats.temp_drift_calib, cmd == 0x50 ? 1U : 0U, "cmd 0x%02x temp_drift_calib", cmd);
        zassert_equal(stats.abort_cmd, cmd == 0x68 ? 1U : 0U, "cmd 0x%02x abort_cmd", cmd);
        zassert_equal(stats.queued, 1U, "cmd 0x%02x queued", cmd);
        zassert_equal(stats.dropped_full, 0U, "cmd 0x%02x dropped_full", cmd);
    }
}

// Catches the cmd being counted only when the put succeeds.
ZTEST(lcs_request, test_sc12_04_put_false_counts_cmd_and_full)
{
    for (const uint8_t cmd : kValidCmds) {
        lcs::RequestStats stats{};
        lcs::enqueue_request([](const lcs::Request &) { return false; }, lcs::Request{cmd, 0, 0}, stats);
        zassert_equal(stats.zero_reset, cmd == 0x48 ? 1U : 0U, "cmd 0x%02x zero_reset", cmd);
        zassert_equal(stats.temp_drift_calib, cmd == 0x50 ? 1U : 0U, "cmd 0x%02x temp_drift_calib", cmd);
        zassert_equal(stats.abort_cmd, cmd == 0x68 ? 1U : 0U, "cmd 0x%02x abort_cmd", cmd);
        zassert_equal(stats.queued, 0U, "cmd 0x%02x queued", cmd);
        zassert_equal(stats.dropped_full, 1U, "cmd 0x%02x dropped_full", cmd);
    }
}

// Catches an increment of the wrong size or a counter interaction over a stream of mixed put results.
ZTEST(lcs_request, test_sc12_05_seeded_stream_matches_oracle)
{
    uint32_t state = 0x2F6E2B1u;
    lcs::RequestStats stats{};
    uint32_t oracle_zero_reset = 0;
    uint32_t oracle_temp_drift_calib = 0;
    uint32_t oracle_abort_cmd = 0;
    uint32_t oracle_queued = 0;
    uint32_t oracle_full = 0;
    for (int i = 0; i < 3000; ++i) {
        const uint8_t cmd = kValidCmds[xorshift32(state) % kValidCmds.size()];
        const bool accepted = (xorshift32(state) % 2) == 0;
        lcs::enqueue_request([accepted](const lcs::Request &) { return accepted; }, lcs::Request{cmd, 0, 0}, stats);

        if (cmd == 0x48) {
            ++oracle_zero_reset;
        } else if (cmd == 0x50) {
            ++oracle_temp_drift_calib;
        } else {
            ++oracle_abort_cmd;
        }
        if (accepted) {
            ++oracle_queued;
        } else {
            ++oracle_full;
        }

        zassert_equal(stats.zero_reset, oracle_zero_reset, "step %d zero_reset", i);
        zassert_equal(stats.temp_drift_calib, oracle_temp_drift_calib, "step %d temp_drift_calib", i);
        zassert_equal(stats.abort_cmd, oracle_abort_cmd, "step %d abort_cmd", i);
        zassert_equal(stats.queued, oracle_queued, "step %d queued", i);
        zassert_equal(stats.dropped_full, oracle_full, "step %d dropped_full", i);
        zassert_equal(stats.zero_reset + stats.temp_drift_calib + stats.abort_cmd, stats.queued + stats.dropped_full,
                      "step %d cmd sum differs from queued + dropped_full", i);
    }
}

// Catches a count that depends on target or param.
ZTEST(lcs_request, test_sc12_05b_nine_valid_pairs_count_per_cmd)
{
    lcs::RequestStats stats{};
    for (const uint8_t cmd : kValidCmds) {
        for (const uint8_t target : kValidTargets) {
            const uint16_t param = (cmd == 0x68) ? 0x1234 : 0;
            const can_frame frame = make_frame(cmd, target, param);
            const std::optional<lcs::Request> req = lcs::decode_request(frame);
            zassert_true(req.has_value(), "cmd 0x%02x target 0x%02x rejected", cmd, target);
            lcs::enqueue_request([](const lcs::Request &) { return true; }, *req, stats);
        }
    }
    zassert_equal(stats.zero_reset, 3U);
    zassert_equal(stats.temp_drift_calib, 3U);
    zassert_equal(stats.abort_cmd, 3U);
    zassert_equal(stats.queued, 9U);
}

// Catches overflow handling: only the counter of the arriving cmd wraps, the others stay.
ZTEST(lcs_request, test_sc12_07_cmd_counters_wrap)
{
    const auto max_stats = [] {
        lcs::RequestStats stats{};
        stats.received = 5;
        stats.queued = 5;
        stats.dropped_invalid = 5;
        stats.dropped_full = 5;
        stats.zero_reset = 0xFFFFFFFFu;
        stats.temp_drift_calib = 0xFFFFFFFFu;
        stats.abort_cmd = 0xFFFFFFFFu;
        return stats;
    };
    {
        lcs::RequestStats stats = max_stats();
        lcs::count_request_cmd(stats, 0x48);
        lcs::RequestStats expected = max_stats();
        expected.zero_reset = 0;
        expect_stats(stats, expected);
    }
    {
        lcs::RequestStats stats = max_stats();
        lcs::count_request_cmd(stats, 0x50);
        lcs::RequestStats expected = max_stats();
        expected.temp_drift_calib = 0;
        expect_stats(stats, expected);
    }
    {
        lcs::RequestStats stats = max_stats();
        lcs::count_request_cmd(stats, 0x68);
        lcs::RequestStats expected = max_stats();
        expected.abort_cmd = 0;
        expect_stats(stats, expected);
    }
}
