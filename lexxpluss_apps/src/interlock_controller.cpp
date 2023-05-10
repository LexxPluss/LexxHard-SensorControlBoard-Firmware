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
#include <device.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include "can_controller.hpp"
#include "interlock_controller.hpp"

namespace lexxhard::interlock_controller {

LOG_MODULE_REGISTER(interlock);

char __aligned(4) msgq_connected_robot_status_buffer[8 * sizeof (msg_connected_robot_status)];
char __aligned(4) msgq_amr_status_buffer[8 * sizeof (msg_amr_status)];
char __aligned(4) msgq_can_interlock_buffer[8 * sizeof (msg_can_interlock)];

class interlock_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_connected_robot_status, msgq_connected_robot_status_buffer, sizeof (msg_connected_robot_status), 8);
        k_msgq_init(&msgq_amr_status, msgq_amr_status_buffer, sizeof (msg_amr_status), 8);
        k_msgq_init(&msgq_can_interlock, msgq_can_interlock_buffer, sizeof (msg_can_interlock), 8);
        is_emergency_stop_at_amr = false;
        is_emergency_stop_at_connected_robot = false;
        return 0;
    }
    void run() {
        const device *gpioc{device_get_binding("GPIOC")};
        if (device_is_ready(gpioc)) {
            gpio_pin_configure(gpioc, 4, GPIO_INPUT      | GPIO_ACTIVE_HIGH);
            gpio_pin_configure(gpioc, 5, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        }

        while (true) {
#ifdef ENABLE_INTERLOCK
            if (device_is_ready(gpioc)) {
                is_emergency_stop_at_connected_robot = (gpio_pin_get(gpioc, 4) == 0);
            } else {
                is_emergency_stop_at_connected_robot = true;
            }

            msg_connected_robot_status message_connected_robot_status;
            message_connected_robot_status.is_emergency_stop = is_emergency_stop_at_connected_robot;
            while (k_msgq_put(&msgq_connected_robot_status, &message_connected_robot_status, K_NO_WAIT) != 0) {
                k_msgq_purge(&msgq_connected_robot_status);
            }

            msg_amr_status message_amr_status;
            if (k_msgq_get(&msgq_amr_status, &message_amr_status, K_NO_WAIT) == 0) {
                is_emergency_stop_at_amr = message_amr_status.is_emergency_stop   ||
                                           can_controller::get_emergency_switch() ||
					   can_controller::get_bumper_switch();
            } else {
                is_emergency_stop_at_amr = true;
            }

            if (device_is_ready(gpioc)) {
                if (is_emergency_stop_at_amr) {
                    gpio_pin_set(gpioc, 5, 0);
                } else {
                    gpio_pin_set(gpioc, 5, 1);
                }
            }
#else
            msg_connected_robot_status message_connected_robot_status;
            message_connected_robot_status.is_emergency_stop = false;
            while (k_msgq_put(&msgq_connected_robot_status, &message_connected_robot_status, K_NO_WAIT) != 0) {
                k_msgq_purge(&msgq_connected_robot_status);
            }
            msg_can_interlock message_can_interlock;
            message_can_interlock.is_emergency_stop = false;
            while (k_msgq_put(&msgq_can_interlock, &message_can_interlock, K_NO_WAIT) != 0) {
                k_msgq_purge(&msgq_can_interlock);
            }
#endif  // ENABLE_INTERLOCK

            k_msleep(200);
        }
    }
private:
    bool is_emergency_stop_at_amr;
    bool is_emergency_stop_at_connected_robot;
} impl;

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
k_msgq msgq_amr_status;
k_msgq msgq_connected_robot_status;
k_msgq msgq_can_interlock;

}  // namespace lexxhard::interlock_controller

// vim: set expandtab shiftwidth=4:
