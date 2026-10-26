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

// ToF CAN allocations. Grid IDs follow the 0x200-0x213 SCB block (AMRSW-2322;
// wire contract 2026-08-02i). Data/health adjacency gives no ordering guarantee.

#include <stdint.h>

namespace lexxhard::tof_can_ids {

inline constexpr uint16_t TOF_GRID_DATA_ID{0x214};
inline constexpr uint16_t TOF_GRID_HEALTH_ID{0x215};

// Package IDs authorized 2026-10-26 (AMRSW-3681). They follow the cliff and
// commissioning pairs at 0x216-0x219. Keep these values here, not in build flags.
inline constexpr uint16_t PACKAGE_TOF_DATA_ID{0x21A};
inline constexpr uint16_t PACKAGE_TOF_HEALTH_ID{0x21B};

// Cliff IDs belong to the generated cliff contract, not this registry.

}  // namespace lexxhard::tof_can_ids
