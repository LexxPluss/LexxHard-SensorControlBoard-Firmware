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
#include "can_controller.hpp"
#include "firmware_updater.hpp"
#include "imu_controller.hpp"
#include "interlock_controller.hpp"
#include "led_controller.hpp"
#include "misc_controller.hpp"
#include "pgv_controller.hpp"
#include "rosserial.hpp"
#include "rosserial_service.hpp"
#include "runaway_detector.hpp"
#include "tof_controller.hpp"
#include "uss_controller.hpp"
#include "towing_unit_controller.hpp"

namespace {

K_THREAD_STACK_DEFINE(actuator_controller_stack, 2048);
K_THREAD_STACK_DEFINE(adc_reader_stack, 2048);
K_THREAD_STACK_DEFINE(can_controller_stack, 2048);
K_THREAD_STACK_DEFINE(firmware_updater_stack, 2048);
K_THREAD_STACK_DEFINE(imu_controller_stack, 2048);
K_THREAD_STACK_DEFINE(interlock_controller_stack, 2048);
K_THREAD_STACK_DEFINE(led_controller_stack, 2048);
K_THREAD_STACK_DEFINE(misc_controller_stack, 2048);
K_THREAD_STACK_DEFINE(pgv_controller_stack, 2048);
K_THREAD_STACK_DEFINE(rosserial_stack, 2048);
K_THREAD_STACK_DEFINE(rosserial_service_stack, 2048);
K_THREAD_STACK_DEFINE(runaway_detector_stack, 2048);
K_THREAD_STACK_DEFINE(tof_controller_stack, 2048);
K_THREAD_STACK_DEFINE(uss_controller_stack, 2048);
K_THREAD_STACK_DEFINE(towing_unit_controller_stack, 2048);

#define RUN(name, prio) \
    k_thread_create(&lexxhard::name::thread, name##_stack, K_THREAD_STACK_SIZEOF(name##_stack), \
                    lexxhard::name::run, nullptr, nullptr, nullptr, prio, K_FP_REGS, K_MSEC(2000));

void reset_usb_hub()
{
    if (const device *gpioj{device_get_binding("GPIOJ")}; device_is_ready(gpioj)) {
        gpio_pin_configure(gpioj, 13, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        k_msleep(1);
        gpio_pin_set(gpioj, 13, 0);
        k_msleep(1);
        gpio_pin_set(gpioj, 13, 1);
    }
}

uint8_t get_board_setting()
{
    uint8_t brd_setting{0}, temp{0};

    const device *gpiof{device_get_binding("GPIOF")};

    if (device_is_ready(gpiof)) {
        gpio_pin_configure(gpiof, 12, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);   //SPRGPI09
        gpio_pin_configure(gpiof, 13, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);   //SPRGPI10

        brd_setting = uint8_t(gpio_pin_get(gpiof, 12));
        temp = uint8_t(gpio_pin_get(gpiof, 13));
        temp = temp << 1;
        brd_setting = brd_setting | temp;
    }

    return brd_setting;
}

}

void main()
{
    reset_usb_hub();
    lexxhard::actuator_controller::init();
    lexxhard::adc_reader::init();
    lexxhard::can_controller::init();
    lexxhard::firmware_updater::init();
    lexxhard::imu_controller::init();
    lexxhard::interlock_controller::init();
    lexxhard::led_controller::init();
    lexxhard::misc_controller::init();
    lexxhard::pgv_controller::init();
    lexxhard::rosserial::init();
    lexxhard::rosserial_service::init();
    lexxhard::runaway_detector::init();
    lexxhard::tof_controller::init();
    lexxhard::uss_controller::init();
    
    RUN(actuator_controller, 2);
    RUN(adc_reader, 2);
    RUN(can_controller, 4);
    RUN(firmware_updater, 7);
    RUN(imu_controller, 2);
    RUN(interlock_controller, 5);
    RUN(led_controller, 1);
    RUN(misc_controller, 2);
    RUN(pgv_controller, 1);
    RUN(tof_controller, 2);
    RUN(uss_controller, 2);
    RUN(runaway_detector, 4);

    switch (get_board_setting()) {
        case 0: //Wani Unit
            lexxhard::towing_unit_controller::init();
            RUN(towing_unit_controller, 2);
            break;
        case 1: //Reserved
            break;
        case 2: //Reserved
            break;
        default:
            break;
    }

    RUN(rosserial, 5); // The rosserial thread will be started last.
    RUN(rosserial_service, 6); // The rosserial thread will be started last.
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
