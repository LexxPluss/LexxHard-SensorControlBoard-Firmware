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

// Test-only reference model of the pre-fix logic. It must stay a faithful copy of the old behavior.

#pragma once

#include "v_wheel_output.hpp"

// Tells the test writer what kind of write follows, so the test can classify old-logic writes.
enum class oracle_write_kind : unsigned char { ENTER, ENTER_DCDC, POLL_CHANGE, POLL_MAINTENANCE };

// PushBuild selects the ENABLE_PUSH_MODE variant, which has no per-cycle cut while the key switch is not running.
template <typename Writer, bool PushBuild>
class old_logic_oracle {
public:
    using state = lexxhard::v_wheel_state;
    using inputs = lexxhard::v_wheel_inputs;
    using reason = lexxhard::v_wheel_write_reason;

    explicit old_logic_oracle(Writer &writer) : writer_(writer) {}

    bool enter(state s, inputs in)
    {
        const bool bat_out_state = in.wheel_poweroff || in.ksw_running;
        if (s == state::STANDBY) {
            // dcdc_converter::set_enable(true) writes SUPPLIED before the state's own write
            writer_.noteNext(oracle_write_kind::ENTER_DCDC, in);
            if (!writer_.write(true, reason::ENTER)) {
                return false;
            }
        }
        switch (s) {
        case state::STANDBY:
        case state::NORMAL:
        case state::SUSPEND:
            writer_.noteNext(oracle_write_kind::ENTER, in);
            return writer_.write(bat_out_state, reason::ENTER);
        case state::POST:
        case state::MANUAL_CHARGE:
        case state::LOCKDOWN:
            writer_.noteNext(oracle_write_kind::ENTER, in);
            return writer_.write(false, reason::ENTER);
        case state::OFF:
            // dcdc_converter::set_enable(false)
            writer_.noteNext(oracle_write_kind::ENTER_DCDC, in);
            return writer_.write(false, reason::ENTER);
        case state::RESUME_WAIT:
        case state::AUTO_CHARGE:
        case state::OTHER:
            return true;
        }
        return true;
    }

    void poll(inputs in)
    {
        // The ROS cut is ignored while the ESW is asserted.
        const bool wheel_poweroff_eff = in.wheel_poweroff && !in.esw_asserted;
        if (last_wheel_poweroff_ != wheel_poweroff_eff) {
            last_wheel_poweroff_ = wheel_poweroff_eff;
            writer_.noteNext(oracle_write_kind::POLL_CHANGE, in);
            if (!writer_.write(!wheel_poweroff_eff, reason::POLL)) {
                return;
            }
        }
        if constexpr (!PushBuild) {
            if (!in.ksw_running) {
                writer_.noteNext(oracle_write_kind::POLL_MAINTENANCE, in);
                if (!writer_.write(false, reason::POLL)) {
                    return;
                }
            }
        }
    }

private:
    Writer &writer_;
    bool last_wheel_poweroff_{false};
};
