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

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
#include "board_controller.hpp"
#include "can_controller.hpp"
#include "bmu_controller.hpp"
#include "firmware_updater.hpp"
#include "imu_controller.hpp"
#include "led_controller.hpp"
#include "pgv_controller.hpp"
#include "zcan_main.hpp"
#include "runaway_detector.hpp"
#include "uss_controller.hpp"

namespace {

K_THREAD_STACK_DEFINE(actuator_controller_stack, 2048);
K_THREAD_STACK_DEFINE(adc_reader_stack, 2048);
K_THREAD_STACK_DEFINE(bmu_controller_stack, 2048);
K_THREAD_STACK_DEFINE(board_controller_stack, 2048);
K_THREAD_STACK_DEFINE(can_controller_stack, 2048);
K_THREAD_STACK_DEFINE(firmware_updater_stack, 2048);
K_THREAD_STACK_DEFINE(imu_controller_stack, 2048);
K_THREAD_STACK_DEFINE(led_controller_stack, 2048);
K_THREAD_STACK_DEFINE(pgv_controller_stack, 2048);
K_THREAD_STACK_DEFINE(runaway_detector_stack, 2048);
K_THREAD_STACK_DEFINE(uss_controller_stack, 2048);
K_THREAD_STACK_DEFINE(zcan_main_stack, 2048);

#define RUN(name, prio) \
    k_thread_create(&lexxhard::name::thread, name##_stack, K_THREAD_STACK_SIZEOF(name##_stack), \
                    lexxhard::name::run, nullptr, nullptr, nullptr, prio, K_FP_REGS, K_MSEC(2000));
}

void init_gpio() {
    gpio_dt_spec gpio_dev;

    // Output
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(ps_led_out), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(resume_led_out), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(bp_reset), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(v_autocharge), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(v24), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(v_wheel), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(v_peripheral), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(wheel_en), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(fan1), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(fan2), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(fan3), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(fan4), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(ps_lift_actuator), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(ipc_power_sw_fp), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    
    // Input
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(ps_sw_in), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(resume_sw_in), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(bp_left), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(es_left), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_LOW);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(es_right), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_LOW);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(mc_din), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(bmu_c_fet), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(bmu_d_fet), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(bmu_p_dsg), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(pgood_24v), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(pgood_peripheral), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(pgood_wheel_motor_left), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(pgood_wheel_motor_right), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
    
    // LED
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_led1), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_INACTIVE);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_led2), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_INACTIVE);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_led3), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_INACTIVE);
    gpio_dev = GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_led4), gpios);
    if (gpio_is_ready_dt(&gpio_dev))
        gpio_pin_configure_dt(&gpio_dev, GPIO_OUTPUT_INACTIVE);

    return;
}

void fan_on() {
    gpio_dt_spec gpio_fan1, gpio_fan2, gpio_fan3, gpio_fan4;

    gpio_fan1 = GPIO_DT_SPEC_GET(DT_NODELABEL(fan1), gpios);
    gpio_fan2 = GPIO_DT_SPEC_GET(DT_NODELABEL(fan2), gpios);
    gpio_fan3 = GPIO_DT_SPEC_GET(DT_NODELABEL(fan3), gpios);
    gpio_fan4 = GPIO_DT_SPEC_GET(DT_NODELABEL(fan4), gpios);

    // Fan1 and Fan2 ON
    if (gpio_is_ready_dt(&gpio_fan1))
        gpio_pin_configure_dt(&gpio_fan1, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
    if (gpio_is_ready_dt(&gpio_fan2))
        gpio_pin_configure_dt(&gpio_fan2, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);

    // Fan3 and Fan4 ON
    if (gpio_is_ready_dt(&gpio_fan3))
        gpio_pin_configure_dt(&gpio_fan3, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
    if (gpio_is_ready_dt(&gpio_fan4))
        gpio_pin_configure_dt(&gpio_fan4, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
}

void power_on() {
    gpio_dt_spec gpio_v24, gpio_wheel, gpio_peripheral;
    gpio_dt_spec gpio_wheel_en;

    gpio_v24 = GPIO_DT_SPEC_GET(DT_NODELABEL(v24), gpios);
    gpio_wheel = GPIO_DT_SPEC_GET(DT_NODELABEL(v_wheel), gpios);
    gpio_peripheral = GPIO_DT_SPEC_GET(DT_NODELABEL(v_peripheral), gpios);
    gpio_wheel_en = GPIO_DT_SPEC_GET(DT_NODELABEL(wheel_en), gpios);

    // Load switch ON
    if (gpio_is_ready_dt(&gpio_wheel)){
        gpio_pin_configure_dt(&gpio_wheel, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        k_msleep(3000);
    }
        
    if (gpio_is_ready_dt(&gpio_peripheral)){
        gpio_pin_configure_dt(&gpio_peripheral, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        k_msleep(3000);
    }
    
    if (gpio_is_ready_dt(&gpio_v24)){
        gpio_pin_configure_dt(&gpio_v24, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        k_msleep(3000);
    }

    if (gpio_is_ready_dt(&gpio_wheel_en)){
        gpio_pin_configure_dt(&gpio_wheel_en, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        k_msleep(3000);
    }
}

int main()
{
    init_gpio();
    k_msleep(3000);

    fan_on();

    lexxhard::actuator_controller::init();
    lexxhard::adc_reader::init();
    lexxhard::bmu_controller::init();
    lexxhard::board_controller::init();
    lexxhard::can_controller::init();
    lexxhard::firmware_updater::init();
    lexxhard::imu_controller::init();
    lexxhard::led_controller::init();
    lexxhard::pgv_controller::init();
    lexxhard::zcan_main::init();
    lexxhard::runaway_detector::init();
    lexxhard::uss_controller::init();

    RUN(actuator_controller, 2);
    RUN(adc_reader, 2);
    RUN(bmu_controller, 4);
    RUN(can_controller, 4);
    RUN(board_controller, 4); // board_controller must be started after bmu_controller (due to the CAN setting)
    RUN(firmware_updater, 7);
    RUN(imu_controller, 2);
    RUN(led_controller, 1);
    RUN(pgv_controller, 1);
    RUN(uss_controller, 2);
    RUN(runaway_detector, 4);
    RUN(zcan_main, 5); // zcan_main thread must be started at last.

    gpio_dt_spec heart_beat_led = GPIO_DT_SPEC_GET(DT_NODELABEL(dbg_led1), gpios);

    // Heartbeat LED
    while (true) {
        if(gpio_is_ready_dt(&heart_beat_led))
            gpio_pin_toggle_dt(&heart_beat_led);
        k_msleep(1000);
    }

    return 0;
}

// vim: set expandtab shiftwidth=4:
