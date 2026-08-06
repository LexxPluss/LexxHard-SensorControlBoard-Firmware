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

// The Dasher chain specification, pure so host tests can validate it.
//
// Chain order and types come from the connectivity diagram and were verified
// electrically on DS20001 (token walk, three independent runs): position 1
// and 2 are the hanging VL53L7CX boards, 3-6 the VL53L4CX drop-sense boards.
//
// Source mapping is CONTRACT-owned (docs/can, 2026-08-02f): source 0 is
// hanging_front_right, source 1 hanging_front_left; the connectivity diagram
// places PCB1 (position 1) on the right, PCB2 (position 2) on the left.
// Position-to-side is L0-owned knowledge and still awaits the frozen J29
// mapping document (Request C) -- if that document contradicts the diagram,
// THIS table is where the correction lands, nowhere else.
//
// The four L4 mounting roles are deliberately `unknown` until the same
// frozen mapping arrives: electrical enumeration proves the type sequence,
// never the mounting role of four identical boards.
//
// Target addresses 0x2A..0x2F are below every other device on this bus and
// were used throughout the DS20001 bring-up.

#include "tof_enumerator.hpp"

namespace lexxhard::tof_chain {

inline constexpr tof_enum::chain_spec dasher_spec()
{
    tof_enum::chain_spec s{};
    s.positions = 6;
    s.at[0] = {tof_enum::model::l7cx, 0x2A, 0, tof_enum::l4_role::unknown};
    s.at[1] = {tof_enum::model::l7cx, 0x2B, 1, tof_enum::l4_role::unknown};
    s.at[2] = {tof_enum::model::l4cx, 0x2C, -1, tof_enum::l4_role::unknown};
    s.at[3] = {tof_enum::model::l4cx, 0x2D, -1, tof_enum::l4_role::unknown};
    s.at[4] = {tof_enum::model::l4cx, 0x2E, -1, tof_enum::l4_role::unknown};
    s.at[5] = {tof_enum::model::l4cx, 0x2F, -1, tof_enum::l4_role::unknown};
    s.alloff_pulses = 8;
    s.watch_count = 0;
    s.require_all_sources = true;
    return s;
}

}  // namespace lexxhard::tof_chain
