// Copyright (c) 2024, LexxPluss Inc.
// All rights reserved.
//
// Tests for eval_normal_transitions() in push_mode_transitions.hpp.
// Compiled twice:
//   test_normal_transitions_standard   — without ENABLE_PUSH_MODE
//   test_normal_transitions_push_mode  — with    ENABLE_PUSH_MODE=1

#include "push_mode_transitions.hpp"
#include <gtest/gtest.h>

using namespace lexxhard::board_controller;

// Baseline: healthy system in NORMAL state (all inputs at safe/neutral values).
static normal_state_inputs healthy() {
    normal_state_inputs in;
    // bmu_ok=true, dcdc_ok=true, nothing asserted — stay NORMAL
    return in;
}

// ============================================================
// Tests valid in BOTH compile modes
// ============================================================

TEST(CommonBehavior, HealthySystemStaysNormal) {
    EXPECT_EQ(eval_normal_transitions(healthy()), POWER_STATE::NORMAL);
}

TEST(CommonBehavior, ShouldTurnOffGoesToOffWait) {
    auto in = healthy();
    in.should_turn_off = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::OFF_WAIT);
}

TEST(CommonBehavior, KswTransitionToRunningGoesToOffWait) {
    auto in = healthy();
    in.ksw_transition_to_running = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::OFF_WAIT);
}

TEST(CommonBehavior, ShouldLockdownGoesToLockdown) {
    auto in = healthy();
    in.should_lockdown = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::LOCKDOWN);
}

TEST(CommonBehavior, LockdownTakesPriorityOverPswPushed) {
    auto in = healthy();
    in.should_lockdown = true;
    in.psw_pushed = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::LOCKDOWN);
}

TEST(CommonBehavior, PowerOffFromRosGoesToSuspend) {
    auto in = healthy();
    in.power_off_from_ros = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(CommonBehavior, BmuFailureGoesToSuspend) {
    auto in = healthy();
    in.bmu_ok = false;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(CommonBehavior, DcdcFailureGoesToSuspend) {
    auto in = healthy();
    in.dcdc_ok = false;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(CommonBehavior, EswAssertedGoesToSuspend) {
    // Hardware emergency switch always causes SUSPEND regardless of push mode
    auto in = healthy();
    in.esw_asserted = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(CommonBehavior, AutoChargeWhenDocked) {
    auto in = healthy();
    in.ac_docked = true;
    in.bmu_chargable = true;
    in.ac_charger_ready = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::AUTO_CHARGE);
}

TEST(CommonBehavior, ManualChargeWhenShouldCharge) {
    auto in = healthy();
    in.should_manual_charge = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::MANUAL_CHARGE);
}

// ============================================================
#ifdef ENABLE_PUSH_MODE
// Push mode: 4 scenarios must NOT cause SUSPEND
// ============================================================

TEST(PushMode, ManualSwitchPushedStaysNormal) {
    auto in = healthy();
    in.psw_pushed = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::NORMAL);
}

TEST(PushMode, SafetyLidarAssertedStaysNormal) {
    auto in = healthy();
    in.sl_asserted = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::NORMAL);
}

TEST(PushMode, EmergencyStopFromRosStaysNormal) {
    auto in = healthy();
    in.emergency_stop_from_ros = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::NORMAL);
}

TEST(PushMode, IsDeadAloneStaysNormal) {
    // is_dead alone (should_lockdown not set) → stays NORMAL in push mode
    auto in = healthy();
    in.is_dead = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::NORMAL);
}

TEST(PushMode, AllFourScenariosTogetherStayNormal) {
    auto in = healthy();
    in.psw_pushed = true;
    in.sl_asserted = true;
    in.emergency_stop_from_ros = true;
    in.is_dead = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::NORMAL);
}

TEST(PushMode, HeartbeatTimeoutCausesLockdown) {
    // should_lockdown = is_dead() && !esw → LOCKDOWN (failsafe always active)
    auto in = healthy();
    in.should_lockdown = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::LOCKDOWN);
}

TEST(PushMode, LockdownNoAutoRecovery) {
    // After lockdown: heartbeat recovers (is_dead=false) but should_lockdown remains true
    // → stays in LOCKDOWN (caller must not clear should_lockdown automatically)
    auto in = healthy();
    in.should_lockdown = true;
    in.is_dead = false;  // heartbeat "recovered"
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::LOCKDOWN);
}

// ============================================================
#else
// Standard mode: all 4 scenarios cause SUSPEND
// ============================================================

TEST(StandardMode, ManualSwitchPushedGoesToSuspend) {
    auto in = healthy();
    in.psw_pushed = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(StandardMode, SafetyLidarAssertedGoesToSuspend) {
    auto in = healthy();
    in.sl_asserted = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(StandardMode, EmergencyStopFromRosGoesToSuspend) {
    auto in = healthy();
    in.emergency_stop_from_ros = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(StandardMode, IsDeadGoesToSuspend) {
    auto in = healthy();
    in.is_dead = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::SUSPEND);
}

TEST(StandardMode, LockdownTakesPriorityOverIsDeadSuspend) {
    // should_lockdown takes priority even in standard mode
    auto in = healthy();
    in.should_lockdown = true;
    in.is_dead = true;
    EXPECT_EQ(eval_normal_transitions(in), POWER_STATE::LOCKDOWN);
}

#endif
