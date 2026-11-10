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

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <optional>
#include <vector>

#include "lcs_request.hpp"
#include "zcan_lcs.hpp"

namespace lexxhard::lcs_controller {
k_msgq msgq;
k_msgq request_msgq;
lexxhard::lcs::RequestStats g_request_stats;
}

namespace {

constexpr int kQueueDepth{16};
constexpr k_timeout_t kRxWait{K_MSEC(100)};
constexpr k_timeout_t kNothingWait{K_MSEC(20)};

alignas(4) std::array<char, kQueueDepth * sizeof(lexxhard::lcs_controller::msg)> lcs_storage;
alignas(4) std::array<char, kQueueDepth * sizeof(can_frame)> rx_storage;
alignas(4) std::array<char, lexxhard::lcs::kRequestQueueDepth * sizeof(lexxhard::lcs::Request)> request_storage;
k_msgq rx_msgq;
lexxhard::zcan_lcs::zcan_lcs request_adapter;

const device &can_dev()
{
    return *DEVICE_DT_GET(DT_NODELABEL(can2));
}

void *suite_setup()
{
    k_msgq_init(&lexxhard::lcs_controller::msgq, lcs_storage.data(), sizeof(lexxhard::lcs_controller::msg), kQueueDepth);
    k_msgq_init(&rx_msgq, rx_storage.data(), sizeof(can_frame), kQueueDepth);
    k_msgq_init(&lexxhard::lcs_controller::request_msgq, request_storage.data(), sizeof(lexxhard::lcs::Request),
                lexxhard::lcs::kRequestQueueDepth);

    /* The loopback driver delivers a sent frame to the filters only in loopback mode, and the mode
     * can be set only before can_start(). */
    zassert_true(device_is_ready(&can_dev()));
    zassert_equal(can_set_mode(&can_dev(), CAN_MODE_LOOPBACK), 0);
    const int started{can_start(&can_dev())};
    zassert_true(started == 0 || started == -EALREADY);

    /* The adapter registers first so that the delivery barrier below is the last filter to receive a frame. */
    request_adapter.init();

    /* Mask 0 accepts every standard ID. */
    const can_filter accept_all{.id = 0, .mask = 0, .flags = 0};
    zassert_true(can_add_rx_filter_msgq(&can_dev(), &rx_msgq, &accept_all) >= 0);
    /* A standard-ID filter never matches an extended frame, so the delivery barrier needs its own. */
    const can_filter accept_all_ext{.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};
    zassert_true(can_add_rx_filter_msgq(&can_dev(), &rx_msgq, &accept_all_ext) >= 0);
    return nullptr;
}

void before_each(void *)
{
    k_msgq_purge(&lexxhard::lcs_controller::msgq);
    k_msgq_purge(&rx_msgq);
    k_msgq_purge(&lexxhard::lcs_controller::request_msgq);
}

lexxhard::lcs_controller::msg make_frame(const std::uint8_t board, const std::uint8_t seed)
{
    lexxhard::lcs_controller::msg frame{board, {}};
    for (std::size_t i{0}; i < frame.data.size(); ++i)
        frame.data[i] = static_cast<std::uint8_t>(seed + i);
    return frame;
}

void queue(const lexxhard::lcs_controller::msg &frame)
{
    zassert_equal(k_msgq_put(&lexxhard::lcs_controller::msgq, &frame, K_NO_WAIT), 0);
}

void expect_received(const std::uint32_t expected_id, const lexxhard::lcs_controller::msg &expected)
{
    can_frame received{};
    zassert_equal(k_msgq_get(&rx_msgq, &received, kRxWait), 0, "no frame on can2");
    zassert_equal(received.id, expected_id);
    zassert_equal(received.dlc, 8);
    zassert_equal(received.flags, 0);
    for (std::size_t i{0}; i < expected.data.size(); ++i)
        zassert_equal(received.data[i], expected.data[i], "payload byte %zu", i);
}

namespace lcs = lexxhard::lcs;

constexpr std::uint32_t kRequestId{0x21E};

can_frame make_request_frame(const std::uint8_t cmd, const std::uint8_t target, const std::uint16_t param)
{
    can_frame frame{};
    frame.id = kRequestId;
    frame.dlc = 8;
    frame.data[0] = cmd;
    frame.data[1] = target;
    frame.data[2] = static_cast<std::uint8_t>(param >> 8);
    frame.data[3] = static_cast<std::uint8_t>(param & 0xFF);
    return frame;
}

can_frame valid_frame(const std::uint8_t seed)
{
    return make_request_frame(0x68, 0, static_cast<std::uint16_t>(0x1200 + seed));
}

/* Independent reference of the request rule; deliberately not the production decoder. */
std::optional<lcs::Request> oracle(const can_frame &frame)
{
    if (frame.id != kRequestId || frame.flags != 0 || frame.dlc != 8)
        return std::nullopt;
    const std::uint8_t cmd{frame.data[0]};
    const std::uint8_t target{frame.data[1]};
    if (!(cmd == 0x48 || cmd == 0x50 || cmd == 0x68) || !(target == 0 || target == 1 || target == 0xFF))
        return std::nullopt;
    if (frame.data[4] != 0 || frame.data[5] != 0 || frame.data[6] != 0 || frame.data[7] != 0)
        return std::nullopt;
    if ((cmd == 0x48 || cmd == 0x50) && (frame.data[2] != 0 || frame.data[3] != 0))
        return std::nullopt;
    return lcs::Request{cmd, target, static_cast<std::uint16_t>((frame.data[2] << 8) | frame.data[3])};
}

/* The accept-all filter on rx_msgq is the barrier: once the frame is there, delivery to every filter is done. */
void send_and_poll(const can_frame &frame)
{
    zassert_equal(can_send(&can_dev(), &frame, K_MSEC(100), nullptr, nullptr), 0);
    can_frame echoed{};
    zassert_equal(k_msgq_get(&rx_msgq, &echoed, kRxWait), 0, "frame not delivered on can2");
    request_adapter.poll();
}

lcs::RequestStats stats()
{
    return request_adapter.request_stats();
}

std::size_t queued_count()
{
    return k_msgq_num_used_get(&lexxhard::lcs_controller::request_msgq);
}

std::optional<lcs::Request> take_request()
{
    lcs::Request request{};
    if (k_msgq_get(&lexxhard::lcs_controller::request_msgq, &request, K_NO_WAIT) != 0)
        return std::nullopt;
    return request;
}

void expect_request(const std::optional<lcs::Request> &actual, const lcs::Request &expected)
{
    zassert_true(actual.has_value(), "no request in the queue");
    zassert_equal(actual->cmd, expected.cmd);
    zassert_equal(actual->target, expected.target);
    zassert_equal(actual->param, expected.param);
}

void expect_stats_delta(const lcs::RequestStats &before, const std::uint32_t received, const std::uint32_t queued,
                        const std::uint32_t dropped_invalid, const std::uint32_t dropped_full)
{
    const lcs::RequestStats now{stats()};
    zassert_equal(now.received - before.received, received, "received delta");
    zassert_equal(now.queued - before.queued, queued, "queued delta");
    zassert_equal(now.dropped_invalid - before.dropped_invalid, dropped_invalid, "dropped_invalid delta");
    zassert_equal(now.dropped_full - before.dropped_full, dropped_full, "dropped_full delta");
}

can_frame with_id(const std::uint32_t id)
{
    can_frame frame{valid_frame(1)};
    frame.id = id;
    return frame;
}

can_frame with_flags(const std::uint8_t flags)
{
    can_frame frame{valid_frame(1)};
    frame.flags = flags;
    return frame;
}

can_frame with_dlc(const std::uint8_t dlc)
{
    can_frame frame{valid_frame(1)};
    frame.dlc = dlc;
    return frame;
}

can_frame with_byte(const std::size_t index, const std::uint8_t value)
{
    can_frame frame{valid_frame(1)};
    frame.data[index] = value;
    return frame;
}

std::vector<can_frame> invalid_frames()
{
    return {with_byte(0, 0x00), with_byte(0, 0x49), with_byte(1, 2),    with_byte(1, 0xFE), with_byte(4, 1),
            with_byte(5, 1),    with_byte(6, 1),    with_dlc(7),        with_dlc(0)};
}

std::vector<can_frame> other_id_frames()
{
    std::vector<can_frame> frames{with_id(0x21F), with_id(0x21C), with_id(0x21D)};
    for (unsigned bit{0}; bit < 11; ++bit)
        frames.push_back(with_id(kRequestId ^ (1U << bit)));
    return frames;
}

std::uint32_t lcg_next(std::uint32_t &state)
{
    state = state * 1664525U + 1013904223U;
    return state >> 8;
}

}  // namespace

ZTEST_SUITE(zcan_lcs, nullptr, suite_setup, before_each, nullptr, nullptr);

// ZC-1: wrong id of board 0, flags set, payload changed
ZTEST(zcan_lcs, test_board0_frame_is_sent_unchanged)
{
    lexxhard::zcan_lcs::zcan_lcs adapter;
    adapter.init();
    const auto frame{make_frame(0, 0x10)};
    queue(frame);
    adapter.poll();
    expect_received(0x21C, frame);
}

// ZC-2: wrong id of board 1, flags set, payload changed
ZTEST(zcan_lcs, test_board1_frame_is_sent_unchanged)
{
    lexxhard::zcan_lcs::zcan_lcs adapter;
    adapter.init();
    const auto frame{make_frame(1, 0x20)};
    queue(frame);
    adapter.poll();
    expect_received(0x21D, frame);
}

// ZC-5: frames reordered, merged or sent more than one per poll
ZTEST(zcan_lcs, test_three_frames_leave_in_order_one_per_poll)
{
    lexxhard::zcan_lcs::zcan_lcs adapter;
    adapter.init();
    const auto first{make_frame(0, 0x30)};
    const auto second{make_frame(1, 0x40)};
    const auto third{make_frame(0, 0x50)};
    queue(first);
    queue(second);
    queue(third);

    adapter.poll();
    expect_received(0x21C, first);
    zassert_equal(k_msgq_num_used_get(&rx_msgq), 0U, "more than one frame in one poll");
    adapter.poll();
    expect_received(0x21D, second);
    zassert_equal(k_msgq_num_used_get(&rx_msgq), 0U, "more than one frame in one poll");
    adapter.poll();
    expect_received(0x21C, third);
}

// ZC-3: a silent renumbering of the wire contract shared with scbdriver
ZTEST(zcan_lcs, test_id_and_length_constants_are_the_wire_contract)
{
    namespace z = lexxhard::zcan_lcs;
    zassert_equal(z::LCS_ID0_DATA_ID, 0x21C);
    zassert_equal(z::LCS_ID1_DATA_ID, 0x21D);
    zassert_equal(z::LCS_REQUEST_ID, 0x21E);
    zassert_equal(z::LCS_LINK_STATE_ID, 0x21F);
    zassert_equal(z::LCS_DATA_LENGTH, 8);
}

// ZC-4: a read beyond the board-to-ID table; the next valid frame still leaves
ZTEST(zcan_lcs, test_board_index_outside_the_table_sends_nothing)
{
    lexxhard::zcan_lcs::zcan_lcs adapter;
    adapter.init();
    for (const std::uint8_t board : {std::uint8_t{2}, std::uint8_t{255}}) {
        queue(make_frame(board, 0x60));
        adapter.poll();
        can_frame received{};
        zassert_not_equal(k_msgq_get(&rx_msgq, &received, kNothingWait), 0, "a frame left for board %u", board);
    }
    const auto valid{make_frame(0, 0x70)};
    queue(valid);
    adapter.poll();
    expect_received(0x21C, valid);
}

// ZC-6: a frame invented on an empty queue
ZTEST(zcan_lcs, test_empty_queue_sends_nothing)
{
    lexxhard::zcan_lcs::zcan_lcs adapter;
    adapter.init();
    adapter.poll();
    can_frame received{};
    zassert_not_equal(k_msgq_get(&rx_msgq, &received, kNothingWait), 0, "a frame appeared from nothing");
}

// SC06-02: no CAN frame other than 0x21C/0x21D (never the link-state ID 0x21F) is sent for any board
ZTEST(zcan_lcs, test_only_board_ids_are_ever_sent)
{
    lexxhard::zcan_lcs::zcan_lcs adapter;
    adapter.init();
    for (const std::uint8_t board : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{255}}) {
        queue(make_frame(board, 0x80));
        adapter.poll();
    }
    std::size_t received_count{0};
    can_frame received{};
    while (k_msgq_get(&rx_msgq, &received, kNothingWait) == 0) {
        zassert_not_equal(received.id, 0x21F, "link-state ID sent");
        zassert_true(received.id == 0x21C || received.id == 0x21D, "unexpected ID 0x%x", received.id);
        ++received_count;
    }
    zassert_equal(received_count, 2U, "frames sent");
}

// SC02-01: a missing, wrong or doubled 0x21E filter, or a filter that matches neighbouring or extended IDs
ZTEST(zcan_lcs, test_request_filter_passes_only_standard_0x21E_once)
{
    const lcs::RequestStats before{stats()};
    send_and_poll(valid_frame(7));
    zassert_equal(queued_count(), 1U, "not exactly one request");
    expect_request(take_request(), *oracle(valid_frame(7)));
    zassert_equal(queued_count(), 0U, "double delivery");
    expect_stats_delta(before, 1, 1, 0, 0);

    for (const can_frame &frame : other_id_frames()) {
        const lcs::RequestStats ref{stats()};
        send_and_poll(frame);
        zassert_equal(queued_count(), 0U, "id 0x%x queued", frame.id);
        expect_stats_delta(ref, 0, 0, 0, 0);
    }

    send_and_poll(with_flags(CAN_FRAME_IDE));
    zassert_equal(queued_count(), 0U, "extended frame queued");
}

// SC02-02: requests lost, reordered or altered between the filter and the queue
ZTEST(zcan_lcs, test_valid_requests_are_queued_in_arrival_order)
{
    const std::array<can_frame, 3> frames{make_request_frame(0x48, 0, 0),
                                          make_request_frame(0x50, 1, 0),
                                          make_request_frame(0x68, 0xFF, 0xFFFF)};
    const lcs::RequestStats before{stats()};
    for (const can_frame &frame : frames)
        send_and_poll(frame);
    zassert_equal(queued_count(), 3U);
    for (const can_frame &frame : frames)
        expect_request(take_request(), *oracle(frame));
    zassert_equal(queued_count(), 0U);
    expect_stats_delta(before, 3, 3, 0, 0);
}

// SC02-03: a frame acted on before its ID, flags, length, command, target and reserved bytes are verified
ZTEST(zcan_lcs, test_invalid_frames_are_dropped_and_counted)
{
    for (const can_frame &frame : invalid_frames()) {
        const lcs::RequestStats before{stats()};
        send_and_poll(frame);
        zassert_equal(queued_count(), 0U, "invalid frame queued");
        expect_stats_delta(before, 1, 0, 1, 0);
    }
}

// SC02-03b: a request frame with a nonzero reserved byte 7 is dropped and counted
ZTEST(zcan_lcs, test_nonzero_byte7_is_dropped_and_counted)
{
    const lcs::RequestStats before{stats()};
    send_and_poll(with_byte(7, 0x01));
    zassert_equal(queued_count(), 0U, "frame with byte 7 set queued");
    expect_stats_delta(before, 1, 0, 1, 0);
}

// SC02-09: a nonzero Param for CMD 0x48 and 0x50 is dropped and counted on the adapter path
ZTEST(zcan_lcs, test_nonzero_param_for_zero_reset_and_temp_drift_is_dropped_and_counted)
{
    for (const std::uint8_t cmd : {0x48, 0x50}) {
        for (const std::uint16_t param : {0x0001, 0x0100}) {
            const lcs::RequestStats before{stats()};
            send_and_poll(make_request_frame(cmd, 0, param));
            zassert_equal(queued_count(), 0U, "request with a nonzero param queued");
            zassert_false(take_request().has_value(), "request with a nonzero param in the queue");
            expect_stats_delta(before, 1, 0, 1, 0);
        }
    }
    const lcs::RequestStats before{stats()};
    send_and_poll(valid_frame(5));
    expect_request(take_request(), *oracle(valid_frame(5)));
    expect_stats_delta(before, 1, 1, 0, 0);
}

// Catches: the mirror not written, written before the counters change, or written from the wrong object
ZTEST(zcan_lcs, test_request_stats_mirror_follows_the_adapter)
{
    lexxhard::lcs_controller::g_request_stats = lcs::RequestStats{};
    send_and_poll(valid_frame(1));
    send_and_poll(with_byte(7, 0x01));
    const lcs::RequestStats mirror{lexxhard::lcs_controller::g_request_stats};
    const lcs::RequestStats own{stats()};
    zassert_true(mirror.received >= 2U, "mirror not updated: received %u", mirror.received);
    zassert_equal(mirror.received, own.received, "mirror received differs");
    zassert_equal(mirror.queued, own.queued, "mirror queued differs");
    zassert_equal(mirror.dropped_invalid, own.dropped_invalid, "mirror dropped_invalid differs");
    zassert_equal(mirror.dropped_full, own.dropped_full, "mirror dropped_full differs");
}

// SC02-04: a rejected frame that leaves state behind and blocks the next valid request
ZTEST(zcan_lcs, test_valid_request_is_accepted_after_a_rejected_frame)
{
    const std::vector<can_frame> rejected{with_id(0x21F), with_id(0x21C), with_flags(CAN_FRAME_IDE), with_dlc(7),
                                          with_byte(0, 0x00)};
    std::uint8_t seed{20};
    for (const can_frame &frame : rejected) {
        send_and_poll(frame);
        const lcs::RequestStats before{stats()};
        const can_frame next{valid_frame(seed++)};
        send_and_poll(next);
        expect_stats_delta(before, 1, 1, 0, 0);
        expect_request(take_request(), *oracle(next));
        zassert_equal(queued_count(), 0U);
    }
}

// SC02-05: a full queue that overwrites, drops the oldest, or does not recover after a get
ZTEST(zcan_lcs, test_full_queue_keeps_the_first_requests_and_counts_the_rest)
{
    constexpr std::size_t depth{lcs::kRequestQueueDepth};
    for (const std::size_t extra : {std::size_t{1}, std::size_t{2}}) {
        const lcs::RequestStats before{stats()};
        for (std::size_t i{0}; i < depth + extra; ++i)
            send_and_poll(valid_frame(static_cast<std::uint8_t>(i)));
        zassert_equal(queued_count(), depth);
        expect_stats_delta(before, depth + extra, depth, 0, extra);

        for (std::size_t i{0}; i < depth; ++i)
            expect_request(take_request(), *oracle(valid_frame(static_cast<std::uint8_t>(i))));
        zassert_equal(queued_count(), 0U);
    }

    for (std::size_t i{0}; i < depth + 1; ++i)
        send_and_poll(valid_frame(static_cast<std::uint8_t>(i)));
    expect_request(take_request(), *oracle(valid_frame(0)));
    const lcs::RequestStats before{stats()};
    send_and_poll(valid_frame(100));
    expect_stats_delta(before, 1, 1, 0, 0);
    for (std::size_t i{1}; i < depth; ++i)
        expect_request(take_request(), *oracle(valid_frame(static_cast<std::uint8_t>(i))));
    expect_request(take_request(), *oracle(valid_frame(100)));
    zassert_equal(queued_count(), 0U);
}

// SC02-06: a request invented or lost on an interleaved stream of valid, invalid and foreign frames
ZTEST(zcan_lcs, test_random_stream_yields_exactly_the_oracle_requests)
{
    constexpr int kKinds{4};
    const std::vector<can_frame> invalid{invalid_frames()};
    const std::vector<can_frame> others{other_id_frames()};

    for (const std::uint32_t seed : {1U, 0xC0FFEEU, 0x5EEDU}) {
        std::uint32_t state{seed};
        const lcs::RequestStats before{stats()};
        std::vector<lcs::Request> expected;
        std::vector<lcs::Request> drained;

        for (int n{0}; n < 200; ++n) {
            const std::uint32_t pick{lcg_next(state)};
            can_frame frame{};
            switch (pick % kKinds) {
            case 0: {
                static constexpr std::array<std::uint8_t, 3> cmds{0x48, 0x50, 0x68};
                static constexpr std::array<std::uint8_t, 3> targets{0, 1, 0xFF};
                const std::uint32_t r{lcg_next(state)};
                frame = make_request_frame(cmds[r % 3], targets[(r >> 4) % 3], static_cast<std::uint16_t>(r >> 8));
                break;
            }
            case 1:
                frame = invalid[(pick >> 4) % invalid.size()];
                break;
            case 2:
                frame = others[(pick >> 4) % others.size()];
                break;
            default:
                frame = with_flags(CAN_FRAME_IDE);
                break;
            }
            if (const auto request{oracle(frame)}; request)
                expected.push_back(*request);

            send_and_poll(frame);
            if (n % 3 == 2) {
                while (const auto request{take_request()})
                    drained.push_back(*request);
            }
        }
        while (const auto request{take_request()})
            drained.push_back(*request);

        zassert_equal(drained.size(), expected.size(), "request count, seed %u", seed);
        for (std::size_t i{0}; i < expected.size(); ++i) {
            zassert_equal(drained[i].cmd, expected[i].cmd, "cmd %zu seed %u", i, seed);
            zassert_equal(drained[i].target, expected[i].target, "target %zu seed %u", i, seed);
            zassert_equal(drained[i].param, expected[i].param, "param %zu seed %u", i, seed);
        }

        const lcs::RequestStats now{stats()};
        const std::uint32_t received{now.received - before.received};
        const std::uint32_t queued{now.queued - before.queued};
        const std::uint32_t invalid_count{now.dropped_invalid - before.dropped_invalid};
        const std::uint32_t full{now.dropped_full - before.dropped_full};
        zassert_equal(received, queued + invalid_count + full, "counter invariant, seed %u", seed);
        zassert_equal(queued, expected.size(), "queued, seed %u", seed);
        zassert_equal(full, 0U, "dropped_full, seed %u", seed);
    }
}

// SC02-08: a failed 0x21E filter registration is retried by a later init(), and a successful one is made once
ZTEST(zcan_lcs, test_failed_filter_registration_is_retried_and_a_successful_one_is_made_once)
{
    bool done{false};
    int calls{0};
    const auto fail{[&] { ++calls; return -ENOSPC; }};
    const auto succeed{[&] { ++calls; return 0; }};

    done = lexxhard::zcan_lcs::register_once(done, fail);
    zassert_false(done, "a failed registration marked done");
    done = lexxhard::zcan_lcs::register_once(done, fail);
    zassert_false(done);
    zassert_equal(calls, 2, "a failed registration was not retried");

    done = lexxhard::zcan_lcs::register_once(done, succeed);
    zassert_true(done);
    zassert_equal(calls, 3);

    done = lexxhard::zcan_lcs::register_once(done, fail);
    zassert_true(done);
    zassert_equal(calls, 3, "a done registration was made again");
}

// vim: set expandtab shiftwidth=4:
