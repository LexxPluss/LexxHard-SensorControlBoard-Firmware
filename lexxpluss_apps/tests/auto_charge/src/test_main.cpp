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

#include <zephyr/ztest.h>

#include <cstddef>

#include "auto_charge_transitions.hpp"

namespace {

using lexxhard::board_controller::auto_charge_decision;
using lexxhard::board_controller::auto_charge_inputs;
using lexxhard::board_controller::eval_auto_charge_transitions;
using lexxhard::board_controller::leave_auto_charge_plan;
using lexxhard::board_controller::plan_enter_wheel_en;
using lexxhard::board_controller::plan_leave_auto_charge;
using lexxhard::board_controller::wheel_en_action;

using input_field = bool auto_charge_inputs::*;

auto_charge_inputs with(const input_field field, const bool value) {
    auto_charge_inputs in{};
    in.*field = value;
    return in;
}

// Final wheel_en result of leaving AUTO_CHARGE for `in`: the enter action wins when it acts, else the leave action.
struct exit_result {
    POWER_STATE next;
    wheel_en_action wheel_en;
};

exit_result composeExit(const auto_charge_inputs& in, const bool ksw_maintenance) {
    const auto_charge_decision decision = eval_auto_charge_transitions(in);
    const leave_auto_charge_plan leave = plan_leave_auto_charge(decision.use_software_brake);
    const wheel_en_action enter = plan_enter_wheel_en(decision.next, ksw_maintenance);
    return exit_result{decision.next, enter != wheel_en_action::NONE ? enter : leave.wheel_en};
}

}  // namespace

// L1: exit decision

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_no_trigger_stays)
{
    const auto_charge_decision d = eval_auto_charge_transitions(auto_charge_inputs{});
    zassert_equal(d.next, POWER_STATE::AUTO_CHARGE);
    zassert_false(d.can_skip_wait_sw);
    zassert_false(d.use_software_brake);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_should_turn_off_goes_off_wait)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::should_turn_off, true));
    zassert_equal(d.next, POWER_STATE::OFF_WAIT);
    zassert_false(d.can_skip_wait_sw);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_ksw_rising_goes_off_wait_and_skips_wait_sw)
{
    const auto_charge_decision d =
        eval_auto_charge_transitions(with(&auto_charge_inputs::ksw_transition_to_running, true));
    zassert_equal(d.next, POWER_STATE::OFF_WAIT);
    zassert_true(d.can_skip_wait_sw);
}

#ifdef ENABLE_PUSH_MODE
ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_psw_pushed_is_ignored_in_push)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::psw_pushed, true));
    zassert_equal(d.next, POWER_STATE::AUTO_CHARGE);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_psw_pushed_still_evaluates_later_conditions)
{
    auto_charge_inputs in = with(&auto_charge_inputs::psw_pushed, true);
    in.bmu_ok = false;
    zassert_equal(eval_auto_charge_transitions(in).next, POWER_STATE::STANDBY);
}
#else
ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_psw_pushed_goes_standby_in_standard)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::psw_pushed, true));
    zassert_equal(d.next, POWER_STATE::STANDBY);
}
#endif

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_power_off_from_ros_goes_standby)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::power_off_from_ros, true));
    zassert_equal(d.next, POWER_STATE::STANDBY);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_bmu_failure_goes_standby)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::bmu_ok, false));
    zassert_equal(d.next, POWER_STATE::STANDBY);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_dcdc_failure_goes_standby)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::dcdc_ok, false));
    zassert_equal(d.next, POWER_STATE::STANDBY);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_esw_goes_suspend_with_brake_latch)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::esw_asserted, true));
    zassert_equal(d.next, POWER_STATE::SUSPEND);
    zassert_true(d.use_software_brake);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_sl_goes_suspend_without_brake_latch)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::sl_asserted, true));
    zassert_equal(d.next, POWER_STATE::SUSPEND);
    zassert_false(d.use_software_brake);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_emergency_stop_goes_suspend_with_brake_latch)
{
    const auto_charge_decision d =
        eval_auto_charge_transitions(with(&auto_charge_inputs::emergency_stop_from_ros, true));
    zassert_equal(d.next, POWER_STATE::SUSPEND);
    zassert_true(d.use_software_brake);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_is_dead_goes_suspend_without_brake_latch)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::is_dead, true));
    zassert_equal(d.next, POWER_STATE::SUSPEND);
    zassert_false(d.use_software_brake);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_full_charge_goes_normal)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::bmu_full_charge, true));
    zassert_equal(d.next, POWER_STATE::NORMAL);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_undocked_goes_normal)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::ac_docked, false));
    zassert_equal(d.next, POWER_STATE::NORMAL);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_manual_charger_plugged_goes_normal)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::mc_plugged, true));
    zassert_equal(d.next, POWER_STATE::NORMAL);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_not_charging_after_check_goes_normal)
{
    const auto_charge_decision d = eval_auto_charge_transitions(with(&auto_charge_inputs::current_check_enable, true));
    zassert_equal(d.next, POWER_STATE::NORMAL);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_not_charging_before_check_stays)
{
    auto_charge_inputs in{};
    in.current_check_enable = false;
    in.bmu_charging = false;
    zassert_equal(eval_auto_charge_transitions(in).next, POWER_STATE::AUTO_CHARGE);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_charging_after_check_stays)
{
    auto_charge_inputs in{};
    in.current_check_enable = true;
    in.bmu_charging = true;
    zassert_equal(eval_auto_charge_transitions(in).next, POWER_STATE::AUTO_CHARGE);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_esw_beats_full_charge)
{
    auto_charge_inputs in = with(&auto_charge_inputs::esw_asserted, true);
    in.bmu_full_charge = true;
    zassert_equal(eval_auto_charge_transitions(in).next, POWER_STATE::SUSPEND);
}

ZTEST(power_control_auto_charge, test_eval_auto_charge_transitions_safety_triggers_go_suspend_in_both_builds)
{
    const input_field fields[] = {&auto_charge_inputs::sl_asserted, &auto_charge_inputs::emergency_stop_from_ros,
                                  &auto_charge_inputs::is_dead};
    for (const input_field field : fields) {
        zassert_equal(eval_auto_charge_transitions(with(field, true)).next, POWER_STATE::SUSPEND);
    }
}

// L2: leave operation plan

#ifdef ENABLE_PUSH_MODE
ZTEST(power_control_auto_charge, test_plan_leave_auto_charge_brake_latch_false_leaves_wheel_en_untouched_in_push)
{
    const leave_auto_charge_plan p = plan_leave_auto_charge(false);
    zassert_true(p.stop_current_check_timer);
    zassert_true(p.force_stop_charger);
    zassert_equal(p.wheel_en, wheel_en_action::NONE);
}

ZTEST(power_control_auto_charge, test_plan_leave_auto_charge_brake_latch_true_leaves_wheel_en_untouched_in_push)
{
    const leave_auto_charge_plan p = plan_leave_auto_charge(true);
    zassert_true(p.stop_current_check_timer);
    zassert_true(p.force_stop_charger);
    zassert_equal(p.wheel_en, wheel_en_action::NONE);
}
#else
ZTEST(power_control_auto_charge, test_plan_leave_auto_charge_brake_latch_false_disables_wheel_en_immediately)
{
    const leave_auto_charge_plan p = plan_leave_auto_charge(false);
    zassert_true(p.stop_current_check_timer);
    zassert_true(p.force_stop_charger);
    zassert_equal(p.wheel_en, wheel_en_action::DISABLE_IMMEDIATE);
}

ZTEST(power_control_auto_charge, test_plan_leave_auto_charge_brake_latch_true_disables_wheel_en_delayed)
{
    const leave_auto_charge_plan p = plan_leave_auto_charge(true);
    zassert_true(p.stop_current_check_timer);
    zassert_true(p.force_stop_charger);
    zassert_equal(p.wheel_en, wheel_en_action::DISABLE_DELAYED);
}
#endif

// L3: leave plan composed with the enter plan of the destination

ZTEST(power_control_auto_charge, test_plan_enter_wheel_en_destinations_match_current_behavior)
{
    zassert_equal(plan_enter_wheel_en(POWER_STATE::STANDBY, false), wheel_en_action::DISABLE_IMMEDIATE);
    zassert_equal(plan_enter_wheel_en(POWER_STATE::STANDBY, true), wheel_en_action::DISABLE_IMMEDIATE);
    zassert_equal(plan_enter_wheel_en(POWER_STATE::SUSPEND, false), wheel_en_action::NONE);
    zassert_equal(plan_enter_wheel_en(POWER_STATE::SUSPEND, true), wheel_en_action::NONE);
    zassert_equal(plan_enter_wheel_en(POWER_STATE::OFF_WAIT, false), wheel_en_action::NONE);
    zassert_equal(plan_enter_wheel_en(POWER_STATE::OFF_WAIT, true), wheel_en_action::NONE);
    zassert_equal(plan_enter_wheel_en(POWER_STATE::NORMAL, false), wheel_en_action::ENABLE);
#ifdef ENABLE_PUSH_MODE
    zassert_equal(plan_enter_wheel_en(POWER_STATE::NORMAL, true), wheel_en_action::ENABLE);
#else
    zassert_equal(plan_enter_wheel_en(POWER_STATE::NORMAL, true), wheel_en_action::DISABLE_IMMEDIATE);
#endif
}

ZTEST(power_control_auto_charge, test_compose_exit_bmu_failure_disables_wheel_en_immediately_in_both_builds)
{
    const exit_result r = composeExit(with(&auto_charge_inputs::bmu_ok, false), false);
    zassert_equal(r.next, POWER_STATE::STANDBY);
    zassert_equal(r.wheel_en, wheel_en_action::DISABLE_IMMEDIATE);
}

#ifdef ENABLE_PUSH_MODE
ZTEST(power_control_auto_charge, test_compose_exit_esw_suspend_never_disables_wheel_en_in_push)
{
    const exit_result r = composeExit(with(&auto_charge_inputs::esw_asserted, true), false);
    zassert_equal(r.next, POWER_STATE::SUSPEND);
    zassert_equal(r.wheel_en, wheel_en_action::NONE);
}

ZTEST(power_control_auto_charge, test_compose_exit_full_charge_enables_wheel_en_in_push)
{
    const exit_result r = composeExit(with(&auto_charge_inputs::bmu_full_charge, true), true);
    zassert_equal(r.next, POWER_STATE::NORMAL);
    zassert_equal(r.wheel_en, wheel_en_action::ENABLE);
}

ZTEST(power_control_auto_charge, test_compose_exit_off_wait_leaves_wheel_en_untouched_in_push)
{
    const exit_result r = composeExit(with(&auto_charge_inputs::should_turn_off, true), false);
    zassert_equal(r.next, POWER_STATE::OFF_WAIT);
    zassert_equal(r.wheel_en, wheel_en_action::NONE);
}

ZTEST(power_control_auto_charge, test_compose_exit_each_trigger_final_wheel_en_matches_table_in_push)
{
    struct row {
        input_field field;
        bool value;
        POWER_STATE next;
        wheel_en_action wheel_en;
    };
    const row rows[] = {
        {&auto_charge_inputs::should_turn_off, true, POWER_STATE::OFF_WAIT, wheel_en_action::NONE},
        {&auto_charge_inputs::ksw_transition_to_running, true, POWER_STATE::OFF_WAIT, wheel_en_action::NONE},
        {&auto_charge_inputs::power_off_from_ros, true, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::bmu_ok, false, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::dcdc_ok, false, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::esw_asserted, true, POWER_STATE::SUSPEND, wheel_en_action::NONE},
        {&auto_charge_inputs::sl_asserted, true, POWER_STATE::SUSPEND, wheel_en_action::NONE},
        {&auto_charge_inputs::emergency_stop_from_ros, true, POWER_STATE::SUSPEND, wheel_en_action::NONE},
        {&auto_charge_inputs::is_dead, true, POWER_STATE::SUSPEND, wheel_en_action::NONE},
    };
    for (const row& r : rows) {
        const exit_result result = composeExit(with(r.field, r.value), false);
        zassert_equal(result.next, r.next);
        zassert_equal(result.wheel_en, r.wheel_en);
    }
}
#else
ZTEST(power_control_auto_charge, test_compose_exit_esw_suspend_disables_wheel_en_delayed_in_standard)
{
    const exit_result r = composeExit(with(&auto_charge_inputs::esw_asserted, true), false);
    zassert_equal(r.next, POWER_STATE::SUSPEND);
    zassert_equal(r.wheel_en, wheel_en_action::DISABLE_DELAYED);
}

ZTEST(power_control_auto_charge, test_compose_exit_full_charge_follows_ksw_maintenance_in_standard)
{
    const auto_charge_inputs in = with(&auto_charge_inputs::bmu_full_charge, true);
    const exit_result maintenance = composeExit(in, true);
    zassert_equal(maintenance.next, POWER_STATE::NORMAL);
    zassert_equal(maintenance.wheel_en, wheel_en_action::DISABLE_IMMEDIATE);
    const exit_result running = composeExit(in, false);
    zassert_equal(running.next, POWER_STATE::NORMAL);
    zassert_equal(running.wheel_en, wheel_en_action::ENABLE);
}

ZTEST(power_control_auto_charge, test_compose_exit_off_wait_disables_wheel_en_immediately_in_standard)
{
    const exit_result r = composeExit(with(&auto_charge_inputs::should_turn_off, true), false);
    zassert_equal(r.next, POWER_STATE::OFF_WAIT);
    zassert_equal(r.wheel_en, wheel_en_action::DISABLE_IMMEDIATE);
}

ZTEST(power_control_auto_charge, test_compose_exit_each_trigger_final_wheel_en_matches_table_in_standard)
{
    struct row {
        input_field field;
        bool value;
        POWER_STATE next;
        wheel_en_action wheel_en;
    };
    const row rows[] = {
        {&auto_charge_inputs::should_turn_off, true, POWER_STATE::OFF_WAIT, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::ksw_transition_to_running, true, POWER_STATE::OFF_WAIT,
         wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::psw_pushed, true, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::power_off_from_ros, true, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::bmu_ok, false, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::dcdc_ok, false, POWER_STATE::STANDBY, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::esw_asserted, true, POWER_STATE::SUSPEND, wheel_en_action::DISABLE_DELAYED},
        {&auto_charge_inputs::sl_asserted, true, POWER_STATE::SUSPEND, wheel_en_action::DISABLE_IMMEDIATE},
        {&auto_charge_inputs::emergency_stop_from_ros, true, POWER_STATE::SUSPEND, wheel_en_action::DISABLE_DELAYED},
        {&auto_charge_inputs::is_dead, true, POWER_STATE::SUSPEND, wheel_en_action::DISABLE_IMMEDIATE},
    };
    for (const row& r : rows) {
        const exit_result result = composeExit(with(r.field, r.value), false);
        zassert_equal(result.next, r.next);
        zassert_equal(result.wheel_en, r.wheel_en);
    }
}
#endif

// L5: enter AUTO_CHARGE

#ifdef ENABLE_PUSH_MODE
ZTEST(power_control_auto_charge, test_plan_enter_wheel_en_auto_charge_maintenance_enables_in_push)
{
    zassert_equal(plan_enter_wheel_en(POWER_STATE::AUTO_CHARGE, true), wheel_en_action::ENABLE);
}

ZTEST(power_control_auto_charge, test_plan_enter_wheel_en_auto_charge_running_enables_in_push)
{
    zassert_equal(plan_enter_wheel_en(POWER_STATE::AUTO_CHARGE, false), wheel_en_action::ENABLE);
}
#else
ZTEST(power_control_auto_charge, test_plan_enter_wheel_en_auto_charge_maintenance_disables_in_standard)
{
    zassert_equal(plan_enter_wheel_en(POWER_STATE::AUTO_CHARGE, true), wheel_en_action::DISABLE_IMMEDIATE);
}

ZTEST(power_control_auto_charge, test_plan_enter_wheel_en_auto_charge_running_enables_in_standard)
{
    zassert_equal(plan_enter_wheel_en(POWER_STATE::AUTO_CHARGE, false), wheel_en_action::ENABLE);
}
#endif

ZTEST_SUITE(power_control_auto_charge, NULL, NULL, NULL, NULL, NULL);
