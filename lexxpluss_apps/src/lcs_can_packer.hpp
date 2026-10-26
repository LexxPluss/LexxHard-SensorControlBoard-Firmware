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

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "lcs_frame.hpp"

namespace lexxhard::lcs {

struct CanFrame {
    uint32_t id;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
};

/* S14: the board-to-CAN-ID table; 0x222 and 0x223 stay reserved. */
inline constexpr std::array<uint32_t, kBoardCount> kCanIds{0x220, 0x221};

static_assert(kCanIds.size() == kBoardCount, "one CAN ID per board");
static_assert(kCanIds[0] != kCanIds[1], "CAN IDs must be distinct");
static_assert(kCanIds[0] >= 0x220 && kCanIds[0] <= 0x223 && kCanIds[1] >= 0x220 && kCanIds[1] <= 0x223,
              "CAN IDs must stay inside the reserved range");

class CanPacker {
public:
    /* Returns nothing for an unknown sender; no counter moves then. */
    std::optional<CanFrame> pack(const Uplink &uplink)
    {
        if (uplink.sender_id >= kBoardCount)
            return std::nullopt;
        uint8_t &counter{counters_[uplink.sender_id]};
        const CanFrame frame{
            kCanIds[uplink.sender_id],
            8,
            {uplink.status, uplink.ch1[0], uplink.ch1[1], uplink.ch1[2], uplink.ch2[0], uplink.ch2[1],
             uplink.ch2[2], counter}};
        /* The counter counts receptions: it moves here even if the frame is dropped later. */
        ++counter;
        return frame;
    }

    void reset() { counters_.fill(0); }

private:
    std::array<uint8_t, kBoardCount> counters_{};
};

}
