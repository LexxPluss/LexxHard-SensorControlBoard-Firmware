/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * CRC16-CCITT of the RS-485 frames to the load cell boards: polynomial 0x1021, initial value
 * 0xFFFF, no reflection, no final XOR. Bitwise on purpose, so there is no lookup table in flash.
 * The CRC of the ASCII bytes "123456789" is 0x29B1.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace lexxhard::lcs {

constexpr uint16_t crc16(const uint8_t *const data, const size_t length)
{
    uint16_t crc{0xFFFF};
    for (size_t i{0}; i < length; ++i) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
        for (int bit{0}; bit < 8; ++bit) {
            crc = static_cast<uint16_t>((crc & 0x8000U) != 0U ? (crc << 1) ^ 0x1021U : crc << 1);
        }
    }
    return crc;
}

}
