/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * CRC16-CCITT of the RS-485 frames to the load cell boards: polynomial 0x1021, initial value
 * 0xFFFF, no reflection, no final XOR. Computed one byte per step without a lookup table, so
 * there is no table in flash and no inner loop over the bits.
 * The CRC of the ASCII bytes "123456789" is 0x29B1.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace lexxhard::lcs {

/* One byte step: the eight shift-and-XOR rounds of polynomial 0x1021 folded into three XOR terms. */
constexpr uint16_t crc16_step(const uint16_t crc, const uint8_t byte)
{
    uint32_t x{((static_cast<uint32_t>(crc) >> 8) ^ byte) & 0xFFU};
    x ^= x >> 4;
    return static_cast<uint16_t>(((static_cast<uint32_t>(crc) << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFFU);
}

constexpr uint16_t crc16(const uint8_t *const data, const size_t length)
{
    uint16_t crc{0xFFFF};
    for (size_t i{0}; i < length; ++i) {
        crc = crc16_step(crc, data[i]);
    }
    return crc;
}

}
