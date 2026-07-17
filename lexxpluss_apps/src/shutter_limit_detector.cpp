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

#include "shutter_limit_detector.hpp"

namespace lexxhard::shutter_limit_detector {

bool is_power_on_masked(uint32_t elapsed_ms)
{
    return elapsed_ms < power_on_mask_ms;
}

void detector::poll(bool open_level, bool closed_level)
{
    confirmed_open_bit = open_level;
    confirmed_closed_bit = closed_level;
}

state detector::get_state() const
{
    return decode(confirmed_open_bit, confirmed_closed_bit);
}

bool detector::get_open_bit() const
{
    return confirmed_open_bit;
}

bool detector::get_closed_bit() const
{
    return confirmed_closed_bit;
}

state detector::decode(bool open_level, bool closed_level)
{
    if (open_level && !closed_level)
        return state::open;
    if (!open_level && closed_level)
        return state::closed;
    if (!open_level && !closed_level)
        return state::between;
    return state::unknown;
}

const char *to_cstr(state s)
{
    switch (s) {
    case state::open: return "open";
    case state::closed: return "closed";
    case state::between: return "between";
    default: return "unknown";
    }
}

}
