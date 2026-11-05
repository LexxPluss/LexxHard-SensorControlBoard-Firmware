/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <tuple>
#include "lcs_can_packer.hpp"
#include "lcs_controller.hpp"
#include "lcs_request.hpp"
#include "zcan_bounded_send.hpp"
#include "zcan_poll_budget.hpp"

namespace lexxhard::zcan_lcs {

LOG_MODULE_REGISTER(zcan_lcs);

inline constexpr uint16_t LCS_ID0_DATA_ID{0x21C};
inline constexpr uint16_t LCS_ID1_DATA_ID{0x21D};
inline constexpr uint16_t LCS_REQUEST_ID{0x21E};
inline constexpr uint16_t LCS_LINK_STATE_ID{0x21F};
inline constexpr uint8_t LCS_DATA_LENGTH{8};

/* The data ID of each board, indexed by the board number; the request and link state IDs stay reserved. */
inline constexpr std::array<uint16_t, lcs::kBoardCount> kBoardDataIds{LCS_ID0_DATA_ID, LCS_ID1_DATA_ID};

static_assert(kBoardDataIds.size() == lcs::kBoardCount, "one CAN ID per board");
static_assert(LCS_ID0_DATA_ID != LCS_ID1_DATA_ID, "CAN IDs must be distinct");
static_assert(LCS_ID0_DATA_ID >= 0x21C && LCS_LINK_STATE_ID <= 0x21F && LCS_ID1_DATA_ID < LCS_REQUEST_ID,
              "CAN IDs must stay inside the reserved range, data IDs below the reserved ones");
static_assert(LCS_DATA_LENGTH == std::tuple_size_v<decltype(lcs_controller::msg::data)>,
              "the data length must equal the payload size");

CAN_MSGQ_DEFINE(msgq_can_lcs_request, lcs::kRequestQueueDepth);

class zcan_lcs {
public:
    void init()
    {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)) {
            LOG_ERR("CAN_2 is not ready");
            return;
        }

        /* One registration per process: a second filter would deliver every frame twice. */
        static bool filter_added{false};
        if (!filter_added) {
            static const can_filter filter_request{.id = LCS_REQUEST_ID, .mask = CAN_STD_ID_MASK, .flags = 0};
            can_add_rx_filter_msgq(dev, &msgq_can_lcs_request, &filter_request);
            filter_added = true;
        }
    }

    void poll()
    {
        lcs_controller::msg message;
        for (int n{0}; n < zcan_poll_budget::kTxPerPass &&
                      k_msgq_get(&lcs_controller::msgq, &message, K_NO_WAIT) == 0; ++n)
        {
            if (message.board >= kBoardDataIds.size())
                continue;
            can_frame frame{
                .id = kBoardDataIds[message.board],
                .dlc = LCS_DATA_LENGTH
            };
            std::memcpy(frame.data, message.data.data(), message.data.size());
            zcan_bounded_send::send(dev, &frame, zcan_poll_budget::kMailboxWait);
        }

        can_frame rx_frame;
        for (int n{0}; n < zcan_poll_budget::kRxPerPass &&
                      k_msgq_get(&msgq_can_lcs_request, &rx_frame, K_NO_WAIT) == 0; ++n)
        {
            ++stats_.received;
            if (const auto request{lcs::decode_request(rx_frame)})
                lcs::enqueue_request([](const lcs::Request &r) {
                    return k_msgq_put(&lcs_controller::request_msgq, &r, K_NO_WAIT) == 0;
                }, *request, stats_);
            else
                ++stats_.dropped_invalid;
            lcs_controller::g_request_stats = stats_;
        }
    }

    const lcs::RequestStats &request_stats() const { return stats_; }

private:
    lcs::RequestStats stats_{};
    const device *dev{nullptr};
};

}

// vim: set expandtab shiftwidth=4:
