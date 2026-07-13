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

namespace lexxhard::shutter_limit_detector {

// EMX4-T12C truth table (REQUIREMENT_Belt_Conveyor_Feature.md sec.4):
// open_level=H,closed_level=L -> open ; open_level=L,closed_level=H -> closed
// open_level=L,closed_level=L -> between (in transit)
// open_level=H,closed_level=H is not a valid EMX4-T12C truth table entry
// (both channels indicating a target simultaneously); treated as unknown
// rather than silently picking one side.
enum class state { unknown, open, closed, between };

// EMX4-T12C datasheet: sensor output is unstable for up to ~100ms after
// power-on. EXTI for the limit switches must stay masked until this elapses,
// otherwise the functional prototype trips a spurious shutter fault on every
// power-up (DESIGN_Belt_Conveyor_Feature.md sec.1 "Power-on interrupt mask").
constexpr uint32_t power_on_mask_ms{100};

bool is_power_on_masked(uint32_t elapsed_ms);

const char *to_cstr(state s);

// Plain polling, no EXTI (see
// INVESTIGATION_shutter_limit_switch_exti_conflict_20260713.md -- the Open
// signal's EXTI line was already claimed by another sensor's interrupt, a
// hardware pin-allocation conflict, not fixable in this module). EMX4-T12C
// is a non-contact sensor with no mechanical bounce, so an unconditional
// level copy every poll() cycle needs no debounce/reconfirm step.
class detector {
public:
    void poll(bool open_level, bool closed_level);
    // Derived on demand from get_open_bit()/get_closed_bit() -- debug
    // display only (see `shutter_limit_switch info`), so it's not cached.
    state get_state() const;
    // The confirmed open/closed switch signals -- for CAN transmission
    // (bit7:6 of CAN_ID_GPIO_IN, see zcan_gpio.hpp).
    bool get_open_bit() const;
    bool get_closed_bit() const;
    static state decode(bool open_level, bool closed_level);
private:
    // (true,true) decodes to state::unknown -- not (false,false), which
    // would decode to between -- before the first poll().
    bool confirmed_open_bit{true};
    bool confirmed_closed_bit{true};
};

}
