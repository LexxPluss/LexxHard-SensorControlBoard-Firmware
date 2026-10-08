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

namespace lexxhard {

enum class v_wheel_write_reason { ENTER, POLL };

enum class v_wheel_state { POST, STANDBY, NORMAL, SUSPEND, RESUME_WAIT, AUTO_CHARGE, MANUAL_CHARGE, LOCKDOWN,
                          OTHER, OFF };

struct v_wheel_inputs {
    bool wheel_poweroff;
    bool ksw_running;
    bool esw_asserted{false};
};

// Target level of the DECIDE mode. true means SUPPLIED.
// Standard: the ESW overrides only the ROS cut, the key switch must still be running.
struct standard_policy {
    static constexpr bool target(v_wheel_inputs inputs) {
        return inputs.ksw_running && !(inputs.wheel_poweroff && !inputs.esw_asserted);
    }
};

// Push: the ESW supplies the wheel regardless of the key switch and the ROS cut.
struct push_policy {
    static constexpr bool target(v_wheel_inputs inputs) {
        return inputs.esw_asserted || (!inputs.wheel_poweroff && inputs.ksw_running);
    }
};

// Single owner of the v_wheel pin writes. The target level is a function of the mode set by the state entry and of the
// inputs, so no write depends on the history of earlier writes.
// Writer::write(bool supplied, v_wheel_write_reason reason) returns false when the gpio is not ready.
template <typename Writer, typename Policy = standard_policy>
class v_wheel_output {
public:
    explicit v_wheel_output(Writer &writer) : writer_(writer) {}

    // Sets the mode of the entered state and applies its target at once.
    // Returns false when the writer failed and the caller may skip the rest of the state entry.
    bool enter(v_wheel_state state, v_wheel_inputs inputs) {
        mode_ = modeOf(state);
        return apply(inputs, v_wheel_write_reason::ENTER);
    }

    // Writes the target of the current mode once per call.
    void poll(v_wheel_inputs inputs) { apply(inputs, v_wheel_write_reason::POLL); }

private:
    enum class mode { FORCE_CUT, DECIDE, HOLD };

    static mode modeOf(v_wheel_state state) {
        switch (state) {
        case v_wheel_state::POST:
        case v_wheel_state::OFF:
        case v_wheel_state::MANUAL_CHARGE:
        case v_wheel_state::LOCKDOWN:
            return mode::FORCE_CUT;
        case v_wheel_state::STANDBY:
        case v_wheel_state::NORMAL:
        case v_wheel_state::SUSPEND:
        case v_wheel_state::RESUME_WAIT:
            return mode::DECIDE;
        case v_wheel_state::AUTO_CHARGE:
        case v_wheel_state::OTHER:
            break;
        }
        return mode::HOLD;
    }

    bool apply(v_wheel_inputs inputs, v_wheel_write_reason reason) {
        switch (mode_) {
        case mode::FORCE_CUT:
            return writer_.write(false, reason);
        case mode::DECIDE:
            return writer_.write(Policy::target(inputs), reason);
        case mode::HOLD:
            break;
        }
        return true;
    }

    Writer &writer_;
    mode mode_{mode::FORCE_CUT};
};

}  // namespace lexxhard
