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

#include <zephyr.h>
#include <drivers/gpio.h>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
// #include "can_controller.hpp"
// #include "bmu_controller.hpp"
// #include "firmware_updater.hpp"
#include "imu_controller.hpp"
#include "led_controller.hpp"
#include "pgv_controller.hpp"
#include "zcan_main.hpp"
#include "runaway_detector.hpp"
#include "uss_controller.hpp"

namespace {

K_THREAD_STACK_DEFINE(actuator_controller_stack, 2048);
K_THREAD_STACK_DEFINE(adc_reader_stack, 2048);
// K_THREAD_STACK_DEFINE(bmu_controller_stack, 2048);
// K_THREAD_STACK_DEFINE(can_controller_stack, 2048);
// K_THREAD_STACK_DEFINE(firmware_updater_stack, 2048);
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

#define PIN_NUM_LOADSW_V24_EN_GPIOC 15
#define PIN_NUM_LOADSW_AUTOCHARGE_EN_GPIOD 0
#define PIN_NUM_LOADSW_WHEEL_EN_GPIOD 1
#define PIN_NUM_LOADSW_PERIPHERAL_EN_GPIOD 2
#define PIN_NUM_FAN1_EN_GPIOC 10
#define PIN_NUM_FAN2_EN_GPIOC 11
#define PIN_NUM_FAN3_EN_GPIOB 14
#define PIN_NUM_FAN4_EN_GPIOB 15
#define PIN_NUM_WHEEL_EN_GPIOK 3

void init_io() {
    if (const device *gpioc{device_get_binding("GPIOC")}; device_is_ready(gpioc)) {
        // Load switch for V24 OFF
        gpio_pin_configure(gpioc, PIN_NUM_LOADSW_V24_EN_GPIOC, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpioc, PIN_NUM_LOADSW_V24_EN_GPIOC, 1);

        // Fan1 and Fan2 ON
        gpio_pin_configure(gpioc, PIN_NUM_FAN1_EN_GPIOC, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpioc, PIN_NUM_FAN1_EN_GPIOC, 1);
        gpio_pin_configure(gpioc, PIN_NUM_FAN2_EN_GPIOC, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpioc, PIN_NUM_FAN2_EN_GPIOC, 1);
    }

    if (const device *gpiob{device_get_binding("GPIOB")}; device_is_ready(gpiob)) {
        // Fan3 and Fan4 ON
        gpio_pin_configure(gpiob, PIN_NUM_FAN3_EN_GPIOB, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiob, PIN_NUM_FAN3_EN_GPIOB, 1);
        gpio_pin_configure(gpiob, PIN_NUM_FAN3_EN_GPIOB, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiob, PIN_NUM_FAN3_EN_GPIOB, 1);
    }

    if (const device *gpiod{device_get_binding("GPIOD")}; device_is_ready(gpiod)) {
        // Load switch for Autocharge, Wheel and Peripheral OFF
        gpio_pin_configure(gpiod, PIN_NUM_LOADSW_AUTOCHARGE_EN_GPIOD, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiod, PIN_NUM_LOADSW_AUTOCHARGE_EN_GPIOD, 0);
        gpio_pin_configure(gpiod, PIN_NUM_LOADSW_WHEEL_EN_GPIOD, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiod, PIN_NUM_LOADSW_WHEEL_EN_GPIOD, 0);
        gpio_pin_configure(gpiod, PIN_NUM_LOADSW_PERIPHERAL_EN_GPIOD, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiod, PIN_NUM_LOADSW_PERIPHERAL_EN_GPIOD, 0);
    }

    if (const device *gpiok{device_get_binding("GPIOK")}; device_is_ready(gpiok)) {
        // Wheel Disable
        gpio_pin_configure(gpiok, PIN_NUM_WHEEL_EN_GPIOK, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiok, PIN_NUM_WHEEL_EN_GPIOK, 0);
    }
    k_msleep(3000);
}

void power_on() {
    if (const device *gpiod{device_get_binding("GPIOD")}; device_is_ready(gpiod)) {
        // Load switch for Wheel and Peripheral ON
        gpio_pin_configure(gpiod, PIN_NUM_LOADSW_WHEEL_EN_GPIOD, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiod, PIN_NUM_LOADSW_WHEEL_EN_GPIOD, 1);
        k_msleep(1000);
        gpio_pin_configure(gpiod, PIN_NUM_LOADSW_PERIPHERAL_EN_GPIOD, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiod, PIN_NUM_LOADSW_PERIPHERAL_EN_GPIOD, 1);
        k_msleep(3000);
    }

    if (const device *gpioc{device_get_binding("GPIOC")}; device_is_ready(gpioc)) {
        // Load switch for V24 ON
        gpio_pin_configure(gpioc, PIN_NUM_LOADSW_V24_EN_GPIOC, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpioc, PIN_NUM_LOADSW_V24_EN_GPIOC, 0);
        k_msleep(3000);
    }

    if (const device *gpiok{device_get_binding("GPIOK")}; device_is_ready(gpiok)) {
        // Wheel Disable
        gpio_pin_configure(gpiok, PIN_NUM_WHEEL_EN_GPIOK, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpiok, PIN_NUM_WHEEL_EN_GPIOK, 1);
    }
}

void main()
{
    /*** Function Only for Zephyr V2 ***/
    init_io();
    power_on();
    /***********************************/

    lexxhard::actuator_controller::init();
    lexxhard::adc_reader::init();
    // lexxhard::can_controller::init();
    // lexxhard::bmu_controller::init();
    // lexxhard::firmware_updater::init();
    lexxhard::imu_controller::init();
    lexxhard::led_controller::init();
    lexxhard::pgv_controller::init();
    lexxhard::zcan_main::init();
    lexxhard::runaway_detector::init();
    lexxhard::uss_controller::init();

    RUN(actuator_controller, 2);
    RUN(adc_reader, 2);
    // RUN(can_controller, 4);
    // RUN(bmu_controller, 4);
    // RUN(firmware_updater, 7);
    RUN(imu_controller, 2);
    RUN(led_controller, 1);
    RUN(pgv_controller, 1);
    RUN(uss_controller, 2);
    RUN(runaway_detector, 4);
    RUN(zcan_main, 5); // The rosserial thread will be started last.

    printk("--- SensorControlBoard V0.9 ---\n");

    // Heartbeat LED
    const device *gpiog{device_get_binding("GPIOG")};
    if (gpiog != nullptr)
        gpio_pin_configure(gpiog, 7, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    int heartbeat_led{1};
    while (true) {
        if (gpiog != nullptr) {
            gpio_pin_set(gpiog, 7, heartbeat_led);
            heartbeat_led = !heartbeat_led;
        }
        k_msleep(1000);
    }
}

// vim: set expandtab shiftwidth=4:
