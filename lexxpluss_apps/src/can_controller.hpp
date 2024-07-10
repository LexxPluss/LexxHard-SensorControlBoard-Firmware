/*
 * Copyright (c) 2022, LexxPluss Inc.
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

#include <zephyr/kernel.h>

namespace lexxhard::can_controller {

struct msg_bmu {
    struct {
        uint16_t value;
        uint8_t id;
    } max_voltage, min_voltage, max_cell_voltage, min_cell_voltage;
    struct {
        int16_t value;
        uint8_t id;
    } max_temp, min_temp, max_current, min_current;
    int16_t fet_temp, pack_current;
    uint16_t charging_current, pack_voltage, design_capacity, full_charge_capacity, remain_capacity;
    uint16_t manufacturing, inspection, serial;
    uint8_t mod_status1, mod_status2, bmu_status, asoc, rsoc, soh;
    uint8_t bmu_fw_ver, mod_fw_ver, serial_config, parallel_config, bmu_alarm1, bmu_alarm2;
} __attribute__((aligned(4)));

struct msg_board {
    float wheel_motor_lr_loadsw_temp, peripheral_loadsw_temp, auto_charge_loadsw_temp, charge_connector_voltage;
    float wheel_motor_l_loadsw_cs, wheel_motor_r_loadsw_cs, peripheral_loadsw_cs;
    int16_t charge_connector_p_temp, charge_connector_n_temp;
    uint8_t fan_duty, shutdown_reason, state, charge_check_count, charge_heartbeat_delay;
    bool bumper_switch_asserted;
    bool emergency_switch_asserted;
    bool power_switch_state, wait_shutdown_state, auto_charging_status, manual_charging_status;
    bool c_fet, d_fet, p_dsg;
    bool is_activated_battery;
    bool v24_loadsw_on, v_wheel_motor_lr_loadsw_on, auto_charge_loadsw_on, v_peripheral_loadsw_on;
    bool v24_pgood, v_wheel_motor_l_pgood, v_wheel_motor_r_pgood, v_peripheral_pgood;
    bool c_act_pgood, l_act_pgood, r_act_pgood;
    bool wheel_enable;
    bool charge_temperature_good;
} __attribute__((aligned(4)));

struct msg_control {
    bool emergency_stop, power_off, wheel_power_off, heart_beat;
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
bool get_emergency_switch();
bool get_bumper_switch();
extern k_thread thread;
extern k_msgq /*msgq_bmu,*/ msgq_board, msgq_control;

}

// vim: set expandtab shiftwidth=4:
