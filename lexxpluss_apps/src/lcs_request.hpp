/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Decoder of the host request frame that is relayed to the LCS boards. Pure: the received frame is passed in.
 * Only a frame of exactly the expected format is decoded, so that a frame of another format is never relayed.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include <zephyr/drivers/can.h>

#include "lcs_frame.hpp"

namespace lexxhard::lcs {

inline constexpr uint16_t kRequestId{0x21E};

inline constexpr uint8_t kCmdZeroReset{0x48};
inline constexpr uint8_t kCmdTempCalib{0x50};
inline constexpr uint8_t kCmdAbort{0x68};

inline constexpr std::size_t kRequestQueueDepth{8};

struct Request {
    uint8_t cmd;
    uint8_t target;
    uint16_t param;
    uint8_t counter;
};

/* received = queued + dropped_invalid + dropped_full. */
struct RequestStats {
    uint32_t received{0};
    uint32_t queued{0};
    uint32_t dropped_invalid{0};
    uint32_t dropped_full{0};
};

/* When the queue is full the new request is dropped and the queued ones are kept. */
template <typename Put>
void enqueue_request(Put &&put, const Request &request, RequestStats &stats)
{
    if (put(request))
        ++stats.queued;
    else
        ++stats.dropped_full;
}

/*
 * Flags must be 0: an extended-ID, remote or FD frame with the same ID is a different format and must not be relayed.
 * Bytes 4-6 must be 0: they are reserved, so a non-zero value means a frame of an unknown format.
 * ID, flags and DLC are decided before any payload byte is read.
 */
inline std::optional<Request> decode_request(const can_frame &frame)
{
    if (frame.id != kRequestId || frame.flags != 0 || frame.dlc != 8)
        return std::nullopt;

    const uint8_t cmd{frame.data[0]};
    const uint8_t target{frame.data[1]};
    const bool cmd_ok{cmd == kCmdZeroReset || cmd == kCmdTempCalib || cmd == kCmdAbort};
    const bool target_ok{target < kBoardCount || target == kBroadcastId};
    const bool reserved_ok{frame.data[4] == 0 && frame.data[5] == 0 && frame.data[6] == 0};
    if (!cmd_ok || !target_ok || !reserved_ok)
        return std::nullopt;

    return Request{cmd, target, static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]), frame.data[7]};
}

}
