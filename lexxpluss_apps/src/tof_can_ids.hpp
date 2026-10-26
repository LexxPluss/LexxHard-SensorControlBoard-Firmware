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

// CAN identifiers for the ToF transports: the grid (AMRSW-2322) and the
// package stream (AMRSW-3681).
//
// Self-assigned integration allocation, authorized by the team (2026-08-06)
// and recorded in the wire contract (docs/can/tof_can_wire_contract.md,
// version 2026-08-02i): the SCB peripheral block 0x200-0x213 is contiguously
// occupied across both repositories and a live bus capture agrees, 0x214+
// extends that block, and all three values rank below every existing control
// and safety identifier in CAN arbitration. The adjacency of the data and
// health values carries no ordering meaning -- the contract guarantees no
// ordering between the two frame kinds.

#include <stdint.h>

namespace lexxhard::tof_can_ids {

inline constexpr uint16_t TOF_GRID_DATA_ID{0x214};
inline constexpr uint16_t TOF_GRID_HEALTH_ID{0x215};

// Package ToF (AMRSW-3681), authorized by the CAN identifier owner on
// 2026-10-26. These two were the experimental candidates every earlier comment
// in this subsystem called unregistered; they are the allocation now, and this
// header is where that is recorded. The block argument is the same one 0x214
// rests on, continued: 0x216/0x217 are the cliff pair and 0x218/0x219 its
// commissioning channel, all four owned by the generated cliff contract, so
// 0x21A is the first free value above a contiguously occupied block and both
// values still arbitrate below every control and safety identifier.
//
// WHY THIS IS THE ONLY PLACE EITHER NUMBER IS WRITTEN IN THIS REPOSITORY. The
// transport takes its identifiers as configuration and refuses a zero, so for
// as long as they were unregistered the build flags carried them and no header
// could hold a default without becoming the allocation by being the only
// number anybody could find. Registration inverts that: a literal in a build
// target is now a duplicate of this declaration, free to drift from it while
// both keep compiling -- which is exactly why the cliff values are not copied
// here either.
inline constexpr uint16_t PACKAGE_TOF_DATA_ID{0x21A};
inline constexpr uint16_t PACKAGE_TOF_HEALTH_ID{0x21B};

// The cliff/drop-sense frames at 0x216 (measurement) and 0x217 (health) are
// deliberately NOT declared here. They are owned by the generated cliff wire
// contract -- docs/can/tof_cliff_contract.h, kMeasId and kHealthId -- and a
// second literal for a value another header already defines is exactly the
// kind of duplicate that drifts apart while both sides keep compiling.
//
// This header previously carried TOF_DROP_SENSE_RESERVED_ID{0x216} with a
// comment saying its payload contract did not exist and no handler could
// claim the value. That contract exists now and the publisher emits both
// frames, so the placeholder said the opposite of the truth to anyone reading
// this file for the allocation. It also left 0x217 unmentioned entirely,
// which is why the collision test below had never checked it. That test now
// pulls the cliff pair straight from the contract.

}  // namespace lexxhard::tof_can_ids
