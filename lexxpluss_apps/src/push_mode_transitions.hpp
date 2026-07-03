// Copyright (c) 2024, LexxPluss Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once
#include "power_state.hpp"

namespace lexxhard::board_controller {

// Inputs for NORMAL state transition evaluation.
// Mirrors the if-else chain in board_controller.cpp NORMAL case (poll).
// IMPORTANT: Keep in sync with board_controller.cpp when editing transition conditions.
struct normal_state_inputs {
    bool should_turn_off{false};
    bool ksw_transition_to_running{false};
    bool should_lockdown{false};
    bool psw_pushed{false};
    bool power_off_from_ros{false};
    bool bmu_ok{true};
    bool dcdc_ok{true};
    bool esw_asserted{false};
    bool sl_asserted{false};
    bool emergency_stop_from_ros{false};
    bool is_dead{false};
    bool charge_guard_asserted{false};
    bool ac_docked{false};
    bool bmu_chargable{false};
    bool ac_charger_ready{false};
    bool should_manual_charge{false};
};

// Returns the next POWER_STATE for the NORMAL state poll.
// Pure function — no side effects (use_software_brake, can_skip_wait_sw are handled by caller).
inline POWER_STATE eval_normal_transitions(const normal_state_inputs& in) {
    if (in.should_turn_off)
        return POWER_STATE::OFF_WAIT;
    if (in.ksw_transition_to_running)
        return POWER_STATE::OFF_WAIT;
    if (in.should_lockdown)
        return POWER_STATE::LOCKDOWN;
#ifndef ENABLE_PUSH_MODE
    if (in.psw_pushed)
        return POWER_STATE::SUSPEND;
#endif
    if (in.power_off_from_ros)
        return POWER_STATE::SUSPEND;
    if (!in.bmu_ok)
        return POWER_STATE::SUSPEND;
    if (!in.dcdc_ok)
        return POWER_STATE::SUSPEND;
    if (in.esw_asserted)
        return POWER_STATE::SUSPEND;
#ifndef ENABLE_PUSH_MODE
    if (in.sl_asserted)
        return POWER_STATE::SUSPEND;
    if (in.emergency_stop_from_ros)
        return POWER_STATE::SUSPEND;
    if (in.is_dead)
        return POWER_STATE::SUSPEND;
#endif
    if (!in.charge_guard_asserted && in.ac_docked && in.bmu_chargable && in.ac_charger_ready)
        return POWER_STATE::AUTO_CHARGE;
    if (in.should_manual_charge)
        return POWER_STATE::MANUAL_CHARGE;
    return POWER_STATE::NORMAL;
}

}  // namespace lexxhard::board_controller
