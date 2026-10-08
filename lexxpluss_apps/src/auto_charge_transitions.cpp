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

#include "auto_charge_transitions.hpp"

namespace lexxhard::board_controller {

namespace {

using reason = auto_charge_exit_reason;

auto_charge_decision makeDecision(const POWER_STATE next, const reason why, const bool can_skip_wait_sw = false,
                                  const bool use_software_brake = false) {
    return auto_charge_decision{next, can_skip_wait_sw, use_software_brake, why};
}

}  // namespace

auto_charge_decision eval_auto_charge_transitions(const auto_charge_inputs& in) {
    if (in.should_turn_off) {
        return makeDecision(POWER_STATE::OFF_WAIT, reason::TURN_OFF);
    }
    if (in.ksw_transition_to_running) {
        return makeDecision(POWER_STATE::OFF_WAIT, reason::KSW_TO_RUNNING, true);
    }
#ifndef ENABLE_PUSH_MODE
    if (in.psw_pushed) {
        return makeDecision(POWER_STATE::STANDBY, reason::PSW);
    }
#endif
    if (in.power_off_from_ros) {
        return makeDecision(POWER_STATE::STANDBY, reason::POWER_OFF_FROM_ROS);
    }
    if (!in.bmu_ok) {
        return makeDecision(POWER_STATE::STANDBY, reason::BMU_FAILURE);
    }
    if (!in.dcdc_ok) {
        return makeDecision(POWER_STATE::STANDBY, reason::DCDC_FAILURE);
    }
    if (in.esw_asserted) {
        return makeDecision(POWER_STATE::SUSPEND, reason::ESW, false, true);
    }
    if (in.sl_asserted) {
        return makeDecision(POWER_STATE::SUSPEND, reason::SAFETY_LIDAR);
    }
    if (in.emergency_stop_from_ros) {
        return makeDecision(POWER_STATE::SUSPEND, reason::EMERGENCY_STOP_FROM_ROS, false, true);
    }
    if (in.is_dead) {
        return makeDecision(POWER_STATE::SUSPEND, reason::DEAD);
    }
    if (in.bmu_full_charge) {
        return makeDecision(POWER_STATE::NORMAL, reason::FULL_CHARGE);
    }
    if (!in.ac_docked) {
        return makeDecision(POWER_STATE::NORMAL, reason::UNDOCKED);
    }
    if (in.mc_plugged) {
        return makeDecision(POWER_STATE::NORMAL, reason::MANUAL_CHARGER);
    }
    if (in.current_check_enable && !in.bmu_charging) {
        return makeDecision(POWER_STATE::NORMAL, reason::NOT_CHARGING);
    }
    return auto_charge_decision{};
}

leave_auto_charge_plan plan_leave_auto_charge(const bool use_software_brake) {
#ifdef ENABLE_PUSH_MODE
    static_cast<void>(use_software_brake);
    return leave_auto_charge_plan{true, true, wheel_en_action::NONE};
#else
    return leave_auto_charge_plan{
        true, true, use_software_brake ? wheel_en_action::DISABLE_DELAYED : wheel_en_action::DISABLE_IMMEDIATE};
#endif
}

wheel_en_action plan_enter_wheel_en(const POWER_STATE s, const bool ksw_maintenance) {
    switch (s) {
    case POWER_STATE::AUTO_CHARGE:
    case POWER_STATE::NORMAL:
#ifdef ENABLE_PUSH_MODE
        static_cast<void>(ksw_maintenance);
        return wheel_en_action::ENABLE;
#else
        return ksw_maintenance ? wheel_en_action::DISABLE_IMMEDIATE : wheel_en_action::ENABLE;
#endif
    case POWER_STATE::STANDBY:
        return wheel_en_action::DISABLE_IMMEDIATE;
    default:
        return wheel_en_action::NONE;
    }
}

}  // namespace lexxhard::board_controller
