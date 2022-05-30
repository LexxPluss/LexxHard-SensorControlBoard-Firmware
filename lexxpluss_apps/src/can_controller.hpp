#pragma once

#include <zephyr.h>

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
    float main_board_temp, actuator_board_temp[3], charge_connector_voltage;
    int16_t charge_connector_temp[2], power_board_temp;
    uint8_t fan_duty, shutdown_reason, state, charge_check_count, charge_heartbeat_delay;
    bool bumper_switch[2];
    bool emergency_switch[2];
    bool power_switch, wait_shutdown, auto_charging, manual_charging;
    bool c_fet, d_fet, p_dsg, v5_fail, v16_fail;
    bool wheel_disable[2];
    bool charge_temperature_error;
} __attribute__((aligned(4)));

struct msg_control {
    bool emergency_stop, power_off, wheel_power_off;
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
uint32_t get_rsoc();
bool is_emergency();
extern k_thread thread;
extern k_msgq msgq_bmu, msgq_board, msgq_control;

}

// vim: set expandtab shiftwidth=4:
