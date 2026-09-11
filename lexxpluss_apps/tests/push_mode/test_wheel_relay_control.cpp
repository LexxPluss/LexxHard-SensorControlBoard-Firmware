// Copyright (c) 2026, LexxPluss Inc.
// All rights reserved.
//
// Tests for eval_wheel_power_cut() in push_mode_transitions.hpp.
// This guard does not branch on ENABLE_PUSH_MODE (esw_asserted is a runtime
// flag from the ESW hardware switch), so a single build is sufficient.

#include "push_mode_transitions.hpp"
#include <gtest/gtest.h>

using namespace lexxhard::board_controller;

TEST(WheelRelayControl, WheelPoweroffWithoutEswCutsPower) {
    // Regression: existing ROS wheel_poweroff behavior must still cut v_wheel
    // when the ESW (Push Mode entry) is not asserted.
    EXPECT_TRUE(eval_wheel_power_cut(true, false));
}

TEST(WheelRelayControl, WheelPoweroffWithEswDoesNotCutPower) {
    // New behavior: while the ESW is asserted (Push Mode), suppress the cut
    // so the regenerative-braking current path is not removed while the
    // wheel can still be spun by external force.
    EXPECT_FALSE(eval_wheel_power_cut(true, true));
}

TEST(WheelRelayControl, EswAloneDoesNotCutPower) {
    // Unrelated case: esw_asserted alone (no wheel_poweroff request) never
    // cuts power in the first place.
    EXPECT_FALSE(eval_wheel_power_cut(false, true));
}

TEST(WheelRelayControl, NeitherFlagDoesNotCutPower) {
    EXPECT_FALSE(eval_wheel_power_cut(false, false));
}
