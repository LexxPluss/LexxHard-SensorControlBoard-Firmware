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

#include <cstdlib>
#include <tuple>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "shutter_motor_controller.hpp"
#include "shutter_controller.hpp"
#include "shutter_limit_detector.hpp"
#include "shutter_limit_switch.hpp"
#include "motor_driver.hpp"
#include "board_controller.hpp"

namespace lexxhard::shutter_motor_controller {

LOG_MODULE_REGISTER(shutter_motor_controller);

char __aligned(4) msgq_request_buffer[8 * sizeof (msg_request)];

// Thin translation between shutter_controller::request and motor_driver's
// int8_t direction -- Center owns its own motor_driver::driver instance,
// independent from the Major loop's (Left/Right).
class shutter {
public:
    int init() { return dev.init(motor_driver::axis::CENTER); }
    // toward_closed mirrors DOWN(-1), toward_open mirrors UP(+1) -- confirmed
    // on real hardware (see shutter_controller.hpp).
    void direct(shutter_controller::request req, uint8_t duty) {
        int8_t const dir{req == shutter_controller::request::toward_closed ? int8_t{-1} :
                          req == shutter_controller::request::toward_open ? int8_t{1} : int8_t{0}};
        dev.set_duty(dir, duty);
    }
    int32_t get_current() const { return dev.get_current(); }
    bool ready() const { return dev.ready(); }
    bool is_failed() const { return dev.is_failed(); }
    std::tuple<shutter_controller::request, uint8_t> get_duty() const {
        auto const [dir, duty]{dev.get_duty()};
        auto const req{dir < 0 ? shutter_controller::request::toward_closed :
                        dir > 0 ? shutter_controller::request::toward_open :
                        shutter_controller::request::stop};
        return {req, duty};
    }
private:
    motor_driver::driver dev;
};

class shutter_motor_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_request, msgq_request_buffer, sizeof (msg_request), 8);
        if (dev.init() != 0) {
            LOG_ERR("shutter pwm init failed.");
            return -1;
        }
        init_ok = true;
        return 0;
    }

    // ~1ms cadence: Limit Switch detection-to-stop latency directly equals
    // mechanical overtravel distance, since Shutter has no encoder backup.
    // Polls shutter_limit_switch itself so the confirmed bits stay fresh
    // at this cadence, not actuator_controller's ~10ms.
    void run() {
        // Mirrors every other HW-backed controller (actuator_controller,
        // imu_controller, etc.): don't drive the motor if init() failed.
        if (!init_ok) {
            LOG_ERR("shutter motor controller init failed, not running.");
            return;
        }
        while (true) {
            msg_request req;
            if (k_msgq_get(&msgq_request, &req, K_NO_WAIT) == 0) {
                last_request = req;
            }

            shutter_limit_switch::poll();
            auto const state{get_state()};

            // emergency/fail reset last_request so a frozen command can't
            // silently resume once cleared. CAN 0x208 freshness is not
            // checked here: Center's stop is Limit Switch/stall_guard, which
            // is independent of ROS host liveness, and the actual ROS design
            // is a single-shot publish per scene transition, not a stream.
            bool const override_stop{board_controller::is_emergency() || dev.is_failed()};
            if (override_stop) {
                last_request = msg_request{};
            }

            auto const requested_direction{shutter_controller::request_from_raw_direction(last_request.direction)};
            auto const decided{shutter_controller::decide_drive(state, requested_direction, last_request.power)};
            auto const cmd{stall.poll(decided, state, static_cast<uint32_t>(k_uptime_get()))};
            dev.direct(cmd.direction, cmd.duty);

            k_msleep(1);
        }
    }

    info get_info() const {
        return {0, dev.get_current(), dev.is_failed()};
    }

    void print_info(const shell *shell) const {
        auto const [direction, duty]{dev.get_duty()};
        shell_print(shell,
                    "init_ok:%s fail_gpio_ready:%s "
                    "state:%s requested_direction:%d requested_power:%u "
                    "emergency:%s fail:%d direction:%d duty:%u "
                    "stall_retries:%d stall_latched:%s current:%d",
                    init_ok ? "yes" : "no",
                    dev.ready() ? "yes" : "no",
                    shutter_limit_detector::to_cstr(get_state()),
                    last_request.direction, last_request.power,
                    board_controller::is_emergency() ? "yes" : "no",
                    dev.is_failed(),
                    static_cast<int>(direction), duty,
                    stall.retry_count(), stall.is_latched() ? "yes" : "no",
                    dev.get_current());
    }

private:
    // (true,true) on peek failure -> state::unknown, fail-safe.
    static shutter_limit_detector::state get_state() {
        shutter_limit_switch::msg m{true, true};
        k_msgq_peek(&shutter_limit_switch::msgq, &m);
        return shutter_limit_detector::detector::decode(m.open_bit, m.closed_bit);
    }

    shutter dev;
    shutter_controller::stall_guard stall;
    msg_request last_request{0, 0};
    bool init_ok{false};
} impl;

int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.print_info(shell);
    return 0;
}

// Debug-only: injects a request directly into msgq_request, the same queue
// handle_control() forwards CAN 0x208's Center slot into -- bypassing only
// actuator_controller's own CAN-forwarding/is_emergency gate (useful when
// ROS is asserting emergency_stop and CAN 0x208 never reaches here at all).
// Does NOT bypass this module's own safety checks: board_controller's
// is_emergency(), the Limit Switch/decide_drive() logic, dev.is_failed(),
// or stall_guard all still apply exactly as they would for a real CAN frame.
int cmd_drive(const shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_error(shell, "Usage: %s %s <direction:-1/0/1> <power:0-100>", argv[-1], argv[0]);
        return 1;
    }
    int const direction_int{atoi(argv[1])};
    int const power_int{atoi(argv[2])};
    if (direction_int != -1 && direction_int != 0 && direction_int != 1) {
        shell_error(shell, "direction must be -1, 0, or 1.");
        return 1;
    }
    if (power_int < 0 || power_int > 100) {
        shell_error(shell, "power must be 0-100.");
        return 1;
    }
    shell_print(shell, "[debug-only] bypassing CAN 0x208 / actuator_controller's emergency "
                        "gate for request delivery only -- this module's own "
                        "emergency/Limit-Switch/fail/stall checks still apply.");
    msg_request const req{static_cast<int8_t>(direction_int), static_cast<uint8_t>(power_int)};
    while (k_msgq_put(&msgq_request, &req, K_NO_WAIT) != 0)
        k_msgq_purge(&msgq_request);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_shutter_motor,
    SHELL_CMD(info, NULL, "Shutter motor information", cmd_info),
    SHELL_CMD(drive, NULL,
        "[debug only] Directly inject a drive request, bypassing CAN/actuator_controller's emergency gate",
        cmd_drive),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(shutter_motor, &sub_shutter_motor, "Shutter motor commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

info get_info()
{
    return impl.get_info();
}

k_thread thread;
k_msgq msgq_request;

}

// vim: set expandtab shiftwidth=4:
