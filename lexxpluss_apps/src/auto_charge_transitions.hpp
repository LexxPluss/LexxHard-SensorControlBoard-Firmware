// Copyright (c) 2026, LexxPluss Inc.
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

// Inputs for the AUTO_CHARGE state poll. Each field is a judged boolean, not a raw device state.
struct auto_charge_inputs {
    bool should_turn_off{false};
    bool ksw_transition_to_running{false};
    bool psw_pushed{false};
    bool power_off_from_ros{false};
    bool bmu_ok{true};
    bool dcdc_ok{true};
    bool esw_asserted{false};
    bool sl_asserted{false};
    bool emergency_stop_from_ros{false};
    bool is_dead{false};
    bool bmu_full_charge{false};
    bool ac_docked{true};
    bool mc_plugged{false};
    bool current_check_enable{false};
    bool bmu_charging{false};
};

// Why the AUTO_CHARGE poll decided to leave. The caller uses it to pick the log line.
enum class auto_charge_exit_reason {
    NONE,
    TURN_OFF,
    KSW_TO_RUNNING,
    PSW,
    POWER_OFF_FROM_ROS,
    BMU_FAILURE,
    DCDC_FAILURE,
    ESW,
    SAFETY_LIDAR,
    EMERGENCY_STOP_FROM_ROS,
    DEAD,
    FULL_CHARGE,
    UNDOCKED,
    MANUAL_CHARGER,
    NOT_CHARGING,
};

// Decision for one AUTO_CHARGE poll. The caller applies the side effects.
struct auto_charge_decision {
    POWER_STATE next{POWER_STATE::AUTO_CHARGE};
    bool can_skip_wait_sw{false};
    bool use_software_brake{false};
    auto_charge_exit_reason reason{auto_charge_exit_reason::NONE};
};

auto_charge_decision eval_auto_charge_transitions(const auto_charge_inputs& in);

enum class wheel_en_action { NONE, ENABLE, DISABLE_IMMEDIATE, DISABLE_DELAYED };

// Operations on leaving AUTO_CHARGE. The caller applies them.
struct leave_auto_charge_plan {
    bool stop_current_check_timer;
    bool force_stop_charger;
    wheel_en_action wheel_en;
};

leave_auto_charge_plan plan_leave_auto_charge(bool use_software_brake);

// wheel_en action on entering `s`. Only AUTO_CHARGE, NORMAL, STANDBY, SUSPEND and OFF_WAIT are handled.
wheel_en_action plan_enter_wheel_en(POWER_STATE s, bool ksw_maintenance);

}  // namespace lexxhard::board_controller
