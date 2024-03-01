/*
 * Copyright (c) 2022-2024, LexxPluss Inc.
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

namespace {

void reset_usb_hub()
{
    if (const gpio_dt_spec reset = GPIO_DT_SPEC_GET(DT_NODELABEL(hubreset), gpios); gpio_is_ready_dt(&reset)) {
        gpio_pin_configure_dt(&reset, GPIO_OUTPUT_ACTIVE);
        k_msleep(1);
        gpio_pin_set_dt(&reset, 0);
        k_msleep(1);
        gpio_pin_set_dt(&reset, 1);
    }
}

}

int main()
{
    reset_usb_hub();
    const gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(led1), gpios);
    gpio_is_ready_dt(&led) && gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    while (true) {
        gpio_is_ready_dt(&led) && gpio_pin_toggle_dt(&led);
        k_msleep(1000);
    }
    return 0;
}

// vim: set expandtab shiftwidth=4:
