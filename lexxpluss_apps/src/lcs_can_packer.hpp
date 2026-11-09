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

/* The CAN ID and the length belong to the CAN adapter (zcan_lcs.hpp); the packer yields the board and the payload. */
struct msg {
    uint8_t board;
    std::array<uint8_t, 8> data;
};

class CanPacker {
public:
    /* Returns nothing for an unknown sender; no counter moves then. */
    std::optional<msg> pack(const Uplink &uplink)
    {
        if (uplink.sender_id >= kBoardCount)
            return std::nullopt;
        uint8_t &counter{counters_[uplink.sender_id]};
        const msg frame{
            uplink.sender_id,
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
