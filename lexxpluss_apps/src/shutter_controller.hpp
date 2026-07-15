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

// Logical direction, not the raw CAN +1/-1 encoding -- the physical
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

// Mirrors msg_control::UP(+1)/DOWN(-1)/STOP(0) (actuator_controller.hpp)
// without including that Zephyr-dependent header. Confirmed on real
// hardware: UP(+1) drives toward open, DOWN(-1) toward closed.
request request_from_raw_direction(int8_t raw_direction);

// TODO(placeholder, non-functional requirement unconfirmed, 2026-07-14): if
// the shutter has been continuously driven toward `driving_direction` for
// longer than ARRIVAL_TIMEOUT_MS without the Limit Switch reaching the state
// that direction should produce (toward_open -> state::open, toward_closed
// -> state::closed), it's presumed stuck (mechanical jam or similar) and
// must stop regardless of what decide_drive() would otherwise allow. This is
// NOT a communication/dead-host timeout -- CAN heartbeat liveness is handled
// by a separate frame/mechanism (board_controller's is_dead()/is_emergency())
// -- it's purely "did the expected physical transition happen in time."
// It's also a simpler, time-based alternative/complement to Current control
// (stuck shutter detection via current sensing, Phase2, out of scope) --
// not a replacement for it. 3 minutes is a provisional value pending
// separate confirmation of the shutter's actual full-travel time.
constexpr uint32_t ARRIVAL_TIMEOUT_MS{180000};

bool is_stalled(request driving_direction, state current_state, uint32_t elapsed_ms_in_direction);

// Mirrors actuator_controller's fail_checker retry-then-give-up pattern:
// retries with a fresh ARRIVAL_TIMEOUT_MS window up to MAX_RETRIES times,
// then latches to stop. Clears once decide_drive()'s output direction
// changes (target reached, or a different direction requested).
class stall_guard {
public:
    drive_command poll(drive_command cmd, state current_state, uint32_t now_ms);
    int retry_count() const { return retries; }
    bool is_latched() const { return retries > MAX_RETRIES; }
private:
    static constexpr int MAX_RETRIES{10};  // provisional, mirrors fail_max
    request active_direction{request::stop};
    uint32_t direction_start_ms{0};
    int retries{0};
};

// Shared with actuator_controller's ACTUATOR_COMMAND_TIMEOUT_MS -- a comms
// fail-safe against a stale CAN 0x208 stream, not overtravel protection
// (the Limit Switch alone owns that). 250ms is Left/Right's existing value.
constexpr uint32_t COMMAND_FRESHNESS_TIMEOUT_MS{250};

bool is_command_stale(uint32_t elapsed_ms_since_last_command);

}
