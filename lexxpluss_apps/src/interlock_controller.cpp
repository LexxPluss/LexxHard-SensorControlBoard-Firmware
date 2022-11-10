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
#include "interlock_controller.hpp"

namespace lexxhard::interlock_controller {

LOG_MODULE_REGISTER(interlock);

char __aligned(4) msgq_enable_amr_buffer[8 * sizeof (msg_enable)];
char __aligned(4) msgq_done_amr_buffer[8 * sizeof (msg_done)];

class interlock_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_enable_amr, msgq_enable_amr_buffer, sizeof (msg_enable), 8);
        k_msgq_init(&msgq_done_amr, msgq_done_amr_buffer, sizeof (msg_done), 8);
	amr_is_operational = false;
        connected_robot_is_operational = false;
        return 0;
    }
    void run() {
        const device *gpioc{device_get_binding("GPIOC")};
        if (device_is_ready(gpioc)) {
            gpio_pin_configure(gpioc, 4, GPIO_INPUT      | GPIO_ACTIVE_HIGH);
            gpio_pin_configure(gpioc, 5, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        }

        while (true) {
            if (device_is_ready(gpioc) && gpio_pin_get(gpioc, 4) == 1) {
                connected_robot_is_operational = false;
 
                if (!amr_is_operational) {
                    amr_is_operational = true;
                    msg_enable message;
		    message.enable = true;
		    while (k_msgq_put(&msgq_enable_amr, &message, K_NO_WAIT) != 0) {
                        k_msgq_purge(&msgq_enable_amr);
                    }
                }
	    }
       
	    msg_done message;
            if (k_msgq_get(&msgq_done_amr, &message, K_NO_WAIT) == 0 && message.done) {
                amr_is_operational = false;

                if (!connected_robot_is_operational) {
                    connected_robot_is_operational = true;
                }
            }

            if (device_is_ready(gpioc)) {
                if (connected_robot_is_operational) {
                    gpio_pin_set(gpioc, 5, 1);
		} else {
                    gpio_pin_set(gpioc, 5, 0);
                }
            }

            k_msleep(200);
        }
    }
private:
    bool amr_is_operational;
    bool connected_robot_is_operational;
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
k_msgq msgq_enable_amr, msgq_done_amr;

}  // namespace lexxhard::interlock_controller

// vim: set expandtab shiftwidth=4:
