/*
 * Copyright (c) 2023, LexxPluss Inc.
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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "can_controller.hpp"
#include "board_controller.hpp"
#include "led_controller.hpp"
#include "power_state.hpp"


#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)

#ifndef VERSION
#  define VERSION v0.0.0-local-build  // This version must not be used for production. Only for local build.
#endif  // VERSION
#define VERSION_STR STR(VERSION)


namespace lexxhard::can_controller {

LOG_MODULE_REGISTER(can);

char __aligned(4) msgq_board_buffer[8 * sizeof (msg_board)];
char __aligned(4) msgq_control_buffer[8 * sizeof (msg_control)];

#define ROS2BOARD_TIMEOUT_MS 3000
class can_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_board, msgq_board_buffer, sizeof (msg_board), 8);
        k_msgq_init(&msgq_control, msgq_control_buffer, sizeof (msg_control), 8);

        ros2board.emergency_stop = true;

        return 0;
    }
    void run() {
        while (true) {
            bool handled{false};

            // board2ros board_controller->can_controller->ROS
            if (k_msgq_get(&board_controller::msgq_board_pb_tx, &board2ros, K_NO_WAIT) == 0) {
                if (k_msgq_put(&msgq_board, &board2ros, K_NO_WAIT) != 0){
                    k_msgq_purge(&msgq_board);
                }
                handled = true;
            }
            // ros2board ROS->can_controller->board_controller
            if ((k_msgq_get(&msgq_control, &ros2board, K_NO_WAIT) == 0) | heartbeat_timeout) {
                handler_to_pb();
                if (k_msgq_put(&board_controller::msgq_board_pb_rx, &msg_board_to_pb, K_NO_WAIT) != 0){
                    k_msgq_purge(&msgq_board);
                }
                prev_cycle_ros = k_cycle_get_32();
                handled = true;
            }

            // if the board_controller state is 0 (OFF), prev_cycle_ros is reset
            if (static_cast<POWER_STATE>(board2ros.state) == POWER_STATE::OFF) {
                prev_cycle_ros = 0;
                prev_cycle_send = 0;
                ros2board.emergency_stop = true;
                ros2board.heart_beat = false;
                ros2board.power_off = false;
                ros2board.wheel_power_off = false;
                heartbeat_timeout = false;
            }

            uint32_t now_cycle{k_cycle_get_32()};
            if (prev_cycle_ros != 0) {
                uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle_ros)};
                heartbeat_timeout = dt_ms > ROS2BOARD_TIMEOUT_MS;
            }

            if (!handled)
                k_msleep(1);
        }
    }
    bool get_emergency_switch() const {
        return board2ros.emergency_switch_asserted;
    }
    bool get_bumper_switch() const {
	return board2ros.bumper_switch_asserted;
    }
    void brd_emgoff() {
        ros2board.emergency_stop = false;
        heartbeat_timeout = false;

        // if changed send to board_controller
        handler_to_pb();
        if (k_msgq_put(&board_controller::msgq_board_pb_rx, &msg_board_to_pb, K_NO_WAIT) != 0){
            k_msgq_purge(&msgq_board);
        }
    }
    void brd_info(const shell *shell) const {
        shell_print(shell,
                    "Bumper:%d Emergency:%d SafetyLiDAR:%d Power:%d\n"
                    "Shutdown:%d Reason:%d AutoCharge:%d ManualCharge:%d\n"
                    "CFET:%d DFET:%d PDSG:%d\n"
                    "V24_PG:%d VPERIPH_PG:%d VWHEEL_L_PG:%d VWHEEL_R_PG:%d\n"
                    "STATE:%u WHEEL:%d\n"
                    "FAN:%u\n"
                    "ConnTemp:P %d N %d\n"
                    "Charge Connector Voltage:%f Count:%u Delay:%u ChargeTempGood:%d\n"
                    "Version:%s\n",
                    board2ros.bumper_switch_asserted, board2ros.emergency_switch_asserted, board2ros.safety_lidar_asserted, board2ros.power_switch_state,
                    board2ros.wait_shutdown_state, board2ros.shutdown_reason, board2ros.auto_charging_status, board2ros.manual_charging_status,
                    board2ros.c_fet, board2ros.d_fet, board2ros.p_dsg,
                    board2ros.v24_pgood, board2ros.v_peripheral_pgood, board2ros.v_wheel_motor_l_pgood, board2ros.v_wheel_motor_r_pgood,
                    board2ros.state, board2ros.wheel_enable,
                    board2ros.fan_duty,
                    board2ros.charge_connector_p_temp, board2ros.charge_connector_n_temp,
                    (double)board2ros.charge_connector_voltage, board2ros.charge_check_count, board2ros.charge_heartbeat_delay, board2ros.charge_temperature_good,
                    version);
    }
private:
    void handler_to_pb() {
        msg_board_to_pb.ros_emergency_stop = ros2board.emergency_stop;
        msg_board_to_pb.ros_power_off = ros2board.power_off;
        msg_board_to_pb.ros_heartbeat_timeout = heartbeat_timeout;
        msg_board_to_pb.ros_wheel_power_off = ros2board.wheel_power_off;
    }
    msg_board board2ros{0};
    msg_control ros2board{true, false, false, false};
    board_controller::msg_rcv_pb msg_board_to_pb{0};
    uint32_t prev_cycle_ros{0}, prev_cycle_send{0};
    bool heartbeat_timeout{false};
    static constexpr char version[]{VERSION_STR};
} impl;

int brd_emgoff(const shell *shell, size_t argc, char **argv)
{
    impl.brd_emgoff();
    return 0;
}

int brd_info(const shell *shell, size_t argc, char **argv)
{
    impl.brd_info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_brd,
    SHELL_CMD(emgoff, NULL, "ROS emergency stop off", brd_emgoff),
    SHELL_CMD(info, NULL, "Board information", brd_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(brd, &sub_brd, "Board commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

bool get_emergency_switch()
{
    return impl.get_emergency_switch();
}

bool get_bumper_switch()
{
    return impl.get_bumper_switch();
}

k_thread thread;
k_msgq msgq_board, msgq_control;

}

// vim: set expandtab shiftwidth=4:
