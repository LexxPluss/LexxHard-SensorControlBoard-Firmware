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

// CAN identifiers for the ToF grid transport (AMRSW-2322).
//
// Self-assigned integration allocation, authorized by the team (2026-08-06)
// and recorded in the wire contract (docs/can/tof_can_wire_contract.md,
// version 2026-08-02f): the SCB peripheral block 0x200-0x213 is contiguously
// occupied across both repositories and a live bus capture agrees, 0x214+
// extends that block, and all three values rank below every existing control
// and safety identifier in CAN arbitration. The adjacency of the data and
// health values carries no ordering meaning -- the contract guarantees no
// ordering between the two frame kinds.

#include <stdint.h>

namespace lexxhard::tof_can_ids {

inline constexpr uint16_t TOF_GRID_DATA_ID{0x214};
inline constexpr uint16_t TOF_GRID_HEALTH_ID{0x215};

// Reserved for the four-channel drop-sense frame. Its payload contract does
// not exist yet: no filter or handler may claim this value until it does.
inline constexpr uint16_t TOF_DROP_SENSE_RESERVED_ID{0x216};

}  // namespace lexxhard::tof_can_ids
