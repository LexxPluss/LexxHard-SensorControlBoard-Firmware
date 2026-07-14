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

#include <algorithm>
#include <tuple>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "shutter_motor_controller.hpp"
#include "shutter_controller.hpp"
#include "shutter_limit_detector.hpp"
#include "board_controller.hpp"
#include "adc_reader.hpp"
#include "common.hpp"

namespace lexxhard::shutter_motor_controller {

LOG_MODULE_REGISTER(shutter_motor_controller);

char __aligned(4) msgq_request_buffer[8 * sizeof (msg_request)];

// Independent from actuator_controller's pwm_driver/fail_checker (act[]) --
// deliberately not shared, so the Major loop (Left/Right) and this Minor
// loop (Center) never write to the same object (see
// DESIGN_actuator_controller_20260714.md sec.7).
class shutter {
public:
    int init() {
        pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm5));
        if (!device_is_ready(pwm_dev))
            return -1;
        direct(shutter_controller::request::stop, 0);
        fail_dev = GET_GPIO(act_c_fail);
        if (gpio_is_ready_dt(&fail_dev))
            gpio_pin_configure_dt(&fail_dev, GPIO_INPUT);
        return 0;
    }
    // toward_closed -> pin 1 (mirrors DOWN in actuator_controller's
    // pwm_driver), toward_open -> pin 2 (mirrors UP). Physical rotation
    // direction is unconfirmed (see shutter_controller.hpp); this only needs
    // to be internally consistent with request_from_raw_direction().
    void direct(shutter_controller::request req, uint8_t duty) {
        uint32_t pulse_ns[2]{CONTROL_PERIOD_NS, CONTROL_PERIOD_NS};
        if (req != shutter_controller::request::stop && duty != 0) {
            uint32_t const duty_rev{std::clamp(100U - duty, 0U, 100U)};
            uint32_t const ns{duty_rev * CONTROL_PERIOD_NS / 100};
            pulse_ns[req == shutter_controller::request::toward_closed ? 0 : 1] = ns;
        }
        pwm_set(pwm_dev, 1, CONTROL_PERIOD_NS, pulse_ns[0], PWM_POLARITY_NORMAL);
        pwm_set(pwm_dev, 2, CONTROL_PERIOD_NS, pulse_ns[1], PWM_POLARITY_NORMAL);
        direction = req;
        this->duty = duty;
    }
    int32_t get_current() const {
        return calc_current(adc_reader::get(adc_reader::ACTUATOR_C));
    }
    bool is_failed() const {
        return gpio_is_ready_dt(&fail_dev) ? gpio_pin_get_dt(&fail_dev) == 0 : false;
    }
    std::tuple<shutter_controller::request, uint8_t> get_duty() const {
        return {direction, duty};
    }
private:
    static int32_t calc_current(int32_t adc_voltage_mv) {
        static constexpr float AMP_GAIN{50.0f}, VOLTAGE_DIVIDER{1.0f}, SHUNT_REGISTER{0.01f};
        float const current_a{adc_voltage_mv * 1e-3f / AMP_GAIN * VOLTAGE_DIVIDER / SHUNT_REGISTER};
        return static_cast<int32_t>(current_a * 1e+3f);
    }
    const device *pwm_dev{nullptr};
    gpio_dt_spec fail_dev{};
    shutter_controller::request direction{shutter_controller::request::stop};
    uint8_t duty{0};
    static constexpr uint32_t CONTROL_HZ{10000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
};

class shutter_motor_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_request, msgq_request_buffer, sizeof (msg_request), 8);
        if (dev.init() != 0) {
            LOG_ERR("shutter pwm init failed.");
            return -1;
        }
        if (!gpio_is_ready_dt(&open_dev) || !gpio_is_ready_dt(&closed_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed");
            return -1;
        }
        if (int const ret{gpio_pin_configure_dt(&open_dev, GPIO_INPUT)}; ret != 0)
            LOG_ERR("gpio_pin_configure_dt(open) failed: %d", ret);
        if (int const ret{gpio_pin_configure_dt(&closed_dev, GPIO_INPUT)}; ret != 0)
            LOG_ERR("gpio_pin_configure_dt(closed) failed: %d", ret);
        start_time = k_uptime_get();
        return 0;
    }

    // ~1ms cadence: Shutter has no encoder backup, so Limit Switch
    // detection-to-stop latency directly equals mechanical overtravel
    // distance -- this loop is intentionally independent of
    // actuator_controller's ~10ms major loop (see
    // DESIGN_actuator_controller_20260714.md sec.7).
    void run() {
        while (true) {
            msg_request req;
            if (k_msgq_get(&msgq_request, &req, K_NO_WAIT) == 0)
                last_request = req;

            auto const elapsed{static_cast<uint32_t>(k_uptime_get() - start_time)};
            if (!shutter_limit_detector::is_power_on_masked(elapsed))
                detector.poll(read(open_dev), read(closed_dev));

            auto const requested_direction{shutter_controller::request_from_raw_direction(last_request.direction)};
            auto const state{detector.get_state()};
            auto const decided{shutter_controller::decide_drive(state, requested_direction, last_request.power)};
            // Retries a stall with a fresh window up to a limited number of
            // times, then latches to stop -- mirrors actuator_controller's
            // fail_checker (fail_max) rather than retrying forever or
            // latching on the very first timeout (see shutter_controller.hpp
            // stall_guard -- provisional 3-minute window, NOT a CAN
            // communication timeout).
            auto const cmd{stall.poll(decided, state, static_cast<uint32_t>(k_uptime_get()))};

            // Independently checked here -- not shared with
            // actuator_controller's major loop -- so this loop's stop
            // guarantee doesn't depend on synchronizing with it.
            // dev.is_failed() is the motor driver IC's hardware fault pin
            // (INA240 current-sense feeding BD63150's RNF, same mechanism as
            // Left/Right's fail_checker) -- a real-time, hardware-detected
            // overcurrent/stall signal, much faster than the arrival_timeout
            // stall_guard above. Previously only reported over CAN, never
            // acted on here.
            if (board_controller::is_emergency() || dev.is_failed())
                dev.direct(shutter_controller::request::stop, 0);
            else
                dev.direct(cmd.direction, cmd.duty);

            k_msleep(1);
        }
    }

    info get_info() const {
        return {0, dev.get_current(), dev.is_failed()};
    }

    void print_info(const shell *shell) const {
        auto const elapsed{static_cast<uint32_t>(k_uptime_get() - start_time)};
        auto const [direction, duty]{dev.get_duty()};
        shell_print(shell,
                    "state:%s raw_open:%c raw_closed:%c mask:%s requested_direction:%d requested_power:%u "
                    "direction:%d duty:%u stall_retries:%d stall_latched:%s current:%d fail:%d",
                    shutter_limit_detector::to_cstr(detector.get_state()),
                    read(open_dev) ? 'H' : 'L',
                    read(closed_dev) ? 'H' : 'L',
                    shutter_limit_detector::is_power_on_masked(elapsed) ? "on" : "off",
                    last_request.direction, last_request.power,
                    static_cast<int>(direction), duty,
                    stall.retry_count(), stall.is_latched() ? "yes" : "no",
                    dev.get_current(), dev.is_failed());
    }

private:
    static bool read(const gpio_dt_spec &spec) { return gpio_pin_get_dt(&spec) > 0; }

    shutter dev;
    gpio_dt_spec open_dev = GET_GPIO(shutter_limit_open);
    gpio_dt_spec closed_dev = GET_GPIO(shutter_limit_closed);
    shutter_limit_detector::detector detector;
    shutter_controller::stall_guard stall;
    int64_t start_time{0};
    msg_request last_request{0, 0};
} impl;

int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.print_info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_shutter_motor,
    SHELL_CMD(info, NULL, "Shutter motor information", cmd_info),
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
