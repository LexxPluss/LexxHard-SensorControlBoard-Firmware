/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * RS-485 frames between the SCB (host) and the load cell boards (LCS), WP5 layout.
 *
 *   Downlink 8 B: SOF | LEN | Target ID | CMD | Param (u16, big-endian) | CRC16 (big-endian)
 *   Uplink  12 B: SOF | LEN | Sender ID | Status | Ch1 (3 B) | Ch2 (3 B) | CRC16 (big-endian)
 *
 * LEN is the whole frame length including SOF, LEN and CRC. The CRC covers every byte before it,
 * LEN included. Ch1 and Ch2 are 24-bit two's complement values, most significant byte first, and
 * are kept as the raw bytes because the CAN frame carries them unchanged.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "lcs_crc16.hpp"

namespace lexxhard::lcs {

inline constexpr uint8_t kSof{0x4B};
inline constexpr uint8_t kBroadcastId{0xFF};
inline constexpr uint8_t kCmdLinkSync{0x08};

inline constexpr size_t kDownlinkSize{8};
inline constexpr size_t kUplinkSize{12};
inline constexpr size_t kMaxLen{kUplinkSize};

/* Boards on the bus: the sender IDs 0 and 1. */
inline constexpr size_t kBoardCount{2};

/* Bit 7 of the CMD byte (downlink) is 0 and of the Status byte (uplink) is 1. */
inline constexpr uint8_t kFrameDirBit{0x80};

using DownlinkFrame = std::array<uint8_t, kDownlinkSize>;
using UplinkFrame = std::array<uint8_t, kUplinkSize>;

struct Uplink {
    uint8_t sender_id;
    uint8_t status;
    std::array<uint8_t, 3> ch1;
    std::array<uint8_t, 3> ch2;
};

constexpr DownlinkFrame encode_downlink(const uint8_t target_id, const uint8_t cmd, const uint16_t param)
{
    DownlinkFrame frame{kSof, static_cast<uint8_t>(kDownlinkSize), target_id, cmd,
                        static_cast<uint8_t>(param >> 8), static_cast<uint8_t>(param & 0xFFU), 0, 0};
    const uint16_t crc{crc16(frame.data(), kDownlinkSize - 2)};
    frame[6] = static_cast<uint8_t>(crc >> 8);
    frame[7] = static_cast<uint8_t>(crc & 0xFFU);
    return frame;
}

constexpr UplinkFrame encode_uplink(const Uplink &uplink)
{
    UplinkFrame frame{kSof, static_cast<uint8_t>(kUplinkSize), uplink.sender_id, uplink.status,
                      uplink.ch1[0], uplink.ch1[1], uplink.ch1[2],
                      uplink.ch2[0], uplink.ch2[1], uplink.ch2[2], 0, 0};
    const uint16_t crc{crc16(frame.data(), kUplinkSize - 2)};
    frame[10] = static_cast<uint8_t>(crc >> 8);
    frame[11] = static_cast<uint8_t>(crc & 0xFFU);
    return frame;
}

}
