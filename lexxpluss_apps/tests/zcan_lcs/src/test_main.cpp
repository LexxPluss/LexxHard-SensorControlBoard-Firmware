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
#include <cstdint>

#include "zcan_lcs.hpp"

namespace lexxhard::lcs_controller {
k_msgq msgq;
}

namespace {

constexpr int kQueueDepth{16};
constexpr k_timeout_t kRxWait{K_MSEC(100)};
constexpr k_timeout_t kNothingWait{K_MSEC(20)};

alignas(4) std::array<char, kQueueDepth * sizeof(lexxhard::lcs_controller::msg)> lcs_storage;
alignas(4) std::array<char, kQueueDepth * sizeof(can_frame)> rx_storage;
k_msgq rx_msgq;

const device &can_dev()
{
    return *DEVICE_DT_GET(DT_NODELABEL(can2));
}

void *suite_setup()
{
    k_msgq_init(&lexxhard::lcs_controller::msgq, lcs_storage.data(), sizeof(lexxhard::lcs_controller::msg), kQueueDepth);
    k_msgq_init(&rx_msgq, rx_storage.data(), sizeof(can_frame), kQueueDepth);

    /* The loopback driver delivers a sent frame to the filters only in loopback mode, and the mode
     * can be set only before can_start(). */
    zassert_true(device_is_ready(&can_dev()));
    zassert_equal(can_set_mode(&can_dev(), CAN_MODE_LOOPBACK), 0);
    const int started{can_start(&can_dev())};
    zassert_true(started == 0 || started == -EALREADY);

    /* Mask 0 accepts every standard ID. */
    const can_filter accept_all{.id = 0, .mask = 0, .flags = 0};
    zassert_true(can_add_rx_filter_msgq(&can_dev(), &rx_msgq, &accept_all) >= 0);
    return nullptr;
}

void before_each(void *)
{
    k_msgq_purge(&lexxhard::lcs_controller::msgq);
    k_msgq_purge(&rx_msgq);
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

// vim: set expandtab shiftwidth=4:
