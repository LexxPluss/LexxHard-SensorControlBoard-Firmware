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
#include "towing_unit_controller.hpp"

namespace lexxhard::towing_unit_controller {

LOG_MODULE_REGISTER(towing_unit);

char __aligned(4) msgq_towing_unit_status_buffer[8 * sizeof (msg_towing_unit_status)];

class towing_unit_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_towing_unit_status, msgq_towing_unit_status_buffer, sizeof (msg_towing_unit_status), 8);
        is_towing_unit_power_on = V12_ON;
        return 0;
    }
    void run() {
        const device *gpioj{device_get_binding("GPIOJ")};
        if (device_is_ready(gpioj)) {
            gpio_pin_configure(gpioj, 1, GPIO_OUTPUT_LOW    | GPIO_ACTIVE_LOW);   //Power ON Output SPRGPIO4
            gpio_pin_configure(gpioj, 2, GPIO_INPUT         | GPIO_ACTIVE_LOW);   //Switch 1 SPRGPIO5
            gpio_pin_configure(gpioj, 3, GPIO_INPUT         | GPIO_ACTIVE_LOW);   //Switch 2 SPRGPIO6
            gpio_pin_configure(gpioj, 4, GPIO_INPUT         | GPIO_ACTIVE_LOW);   //PowerGood +12V SPRGPIO7
        }

        while (true) {
            //Get Switch & Power Good Status
            if (device_is_ready(gpioj)) {
                /* SW_L */ 
                if(gpio_pin_get(gpioj, 1) == 0){
                    is_towing_unit_sw_l_loading = LOADED;  //Loading
                }else{
                    is_towing_unit_sw_l_loading = UNLOADED; //Not Loading
                }
                /* SW_R */ 
                if(gpio_pin_get(gpioj, 2) == 0){
                    is_towing_unit_sw_r_loading = LOADED;  //Loading
                }else{
                    is_towing_unit_sw_r_loading = UNLOADED; //Not Loading
                }
                /* Power Good */ 
                if(gpio_pin_get(gpioj, 3) == 0){
                    is_towing_unit_power_good = V12_OK;  //+12V is on
                }else{
                    is_towing_unit_power_good = V12_NG; //+12V is off
                } 
            } else{
                is_towing_unit_sw_l_loading = DEVICE_NOT_READY
                is_towing_unit_sw_r_loading = DEVICE_NOT_READY
                is_towing_unit_power_good = DEVICE_NOT_READY
            }

            //Get Power ON Output Status
            if (k_msgq_get(&message_towing_status, &message_towing_status_rx, K_NO_WAIT) == 0) {
                is_towing_unit_power_on = message_towing_status_rx.power_on;
            } 
            /* Power ON */
            if (device_is_ready(gpioj)) {
                if (is_towing_unit_power_on == V12_ON) {
                    gpio_pin_set(gpioj, 4, 0);  //Set 12V Power ON(active low)
                } else {
                    gpio_pin_set(gpioj, 4, 1);  //Set 12V Power OFF
                }
            } 

            /* Set Status to PUB message */
            message_towing_status_tx.left_sw = is_towing_unit_sw_l_loading;
            message_towing_status_tx.right_sw = is_towing_unit_sw_r_loading;
            message_towing_status_tx.power_good = is_towing_unit_power_good;
            message_towing_status_tx.power_on = is_towing_unit_power_on;

            /* Send PUB message */
            while (k_msgq_put(&msgq_towing_unit_status, &message_towing_status_tx, K_NO_WAIT) != 0) {
                k_msgq_purge(&msgq_towing_unit_status);
            }

            k_msleep(200);
        }
    }
private:
    msg_towing_unit_status message_towing_status_rx, message_towing_status_tx;
    uint8_t is_towing_unit_power_on;
    uint8_t is_towing_unit_power_good;
    uint8_t is_towing_unit_sw_l_loading;
    uint8_t is_towing_unit_sw_r_loading;
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
k_msgq msgq_towing_unit_status;

}  // namespace lexxhard::towing_unit_controller

// vim: set expandtab shiftwidth=4:
