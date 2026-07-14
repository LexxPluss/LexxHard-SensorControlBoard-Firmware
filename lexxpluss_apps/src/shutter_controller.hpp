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

#include <cstdint>
#include "shutter_limit_detector.hpp"

namespace lexxhard::shutter_controller {

using lexxhard::shutter_limit_detector::state;

// Logical direction, not the raw CAN ±1 encoding -- the physical
// RAISE/LOWER-to-open/close mapping is unverified on real hardware
// (TESTPLAN_shutter_controller_20260714.md sec.2.0), so that conversion is
// deliberately kept out of this pure logic and left to the caller once
// confirmed.
enum class request : uint8_t { stop, toward_open, toward_closed };

struct drive_command {
    request direction;
    uint8_t duty;
};

// Given the confirmed Limit Switch state and a requested drive command,
// decides the command that may actually be applied. Shutter has no encoder
// backup, so Limit Switch state is the only overtravel protection -- driving
// further toward an already-reached limit is always rejected.
drive_command decide_drive(state current_state, request requested_direction, uint8_t requested_duty);

}
