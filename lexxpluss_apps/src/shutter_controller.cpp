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

#include "shutter_controller.hpp"

namespace lexxhard::shutter_controller {

drive_command decide_drive(state current_state, request requested_direction, uint8_t requested_duty)
{
    if (requested_direction == request::stop || requested_duty == 0) {
        return {request::stop, 0};
    }

    switch (current_state) {
    case state::open:
        if (requested_direction == request::toward_open) {
            return {request::stop, 0};
        }
        break;
    case state::closed:
        if (requested_direction == request::toward_closed) {
            return {request::stop, 0};
        }
        break;
    case state::between:
        break;
    case state::unknown:
        return {request::stop, 0};
    }

    return {requested_direction, requested_duty};
}

request request_from_raw_direction(int8_t raw_direction)
{
    if (raw_direction > 0) {
        return request::toward_open;
    }
    if (raw_direction < 0) {
        return request::toward_closed;
    }
    return request::stop;
}

bool is_stalled(request driving_direction, state current_state, uint32_t elapsed_ms_in_direction)
{
    if (driving_direction == request::stop) {
        return false;
    }
    if (driving_direction == request::toward_open && current_state == state::open) {
        return false;
    }
    if (driving_direction == request::toward_closed && current_state == state::closed) {
        return false;
    }
    return elapsed_ms_in_direction >= ARRIVAL_TIMEOUT_MS;
}

drive_command stall_guard::poll(drive_command cmd, state current_state, uint32_t now_ms)
{
    if (cmd.direction != active_direction) {
        bool const reached{(active_direction == request::toward_open && current_state == state::open)
                            || (active_direction == request::toward_closed && current_state == state::closed)};
        if (cmd.direction != request::stop || reached) {
            // Target reached, or a genuinely different direction requested.
            active_direction = cmd.direction;
            direction_start_ms = now_ms;
            retries = 0;
            return cmd;
        }
        if (current_state == state::unknown) {
            // Limit Switch fault mid-drive: decide_drive() safely blocks the
            // motor every cycle, but this is a stall, not a transient
            // override -- let elapsed accumulate so it eventually surfaces
            // via retry_count()/is_latched() instead of going unnoticed
            // forever.
            if (is_latched()) {
                return {request::stop, 0};
            }
            uint32_t const elapsed{now_ms - direction_start_ms};
            if (elapsed >= ARRIVAL_TIMEOUT_MS) {
                ++retries;
                direction_start_ms = now_ms;
            }
            return {request::stop, 0};
        }
        // Transient safety-override stop -- preserve retry/latch history,
        // and slide the window so it doesn't count as elapsed once driving resumes.
        direction_start_ms = now_ms;
        return {request::stop, 0};
    }
    if (is_latched()) {
        return {request::stop, 0};
    }

    uint32_t const elapsed{now_ms - direction_start_ms};
    if (!is_stalled(cmd.direction, current_state, elapsed)) {
        return cmd;
    }

    // Stalled: retry with a fresh window, up to max_retries times, then give
    // up and latch (is_latched() above will keep returning stop until
    // cmd.direction changes).
    ++retries;
    direction_start_ms = now_ms;
    return {request::stop, 0};
}

}
