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

#include <zephyr/drivers/pwm.h>
#include "motor_driver.hpp"
#include "motor_driver_calc.hpp"
#include "adc_reader.hpp"
#include "common.hpp"

namespace lexxhard::motor_driver {

int driver::init(axis a)
{
    switch (a) {
    case axis::CENTER:
        dev[0] = DEVICE_DT_GET(DT_NODELABEL(pwm5));
        dev[1] = dev[0];
        pin[0] = 1;
        pin[1] = 2;
        fail_dev = GET_GPIO(act_c_fail);
        current_adc = adc_reader::ACTUATOR_C;
        break;
    case axis::LEFT:
        dev[0] = DEVICE_DT_GET(DT_NODELABEL(pwm8));
        dev[1] = dev[0];
        pin[0] = 1;
        pin[1] = 2;
        fail_dev = GET_GPIO(act_l_fail);
        current_adc = adc_reader::ACTUATOR_L;
        break;
    case axis::RIGHT:
        dev[0] = DEVICE_DT_GET(DT_NODELABEL(pwm2));
        dev[1] = dev[0];
        pin[0] = 3;
        pin[1] = 4;
        fail_dev = GET_GPIO(act_r_fail);
        current_adc = adc_reader::ACTUATOR_R;
        break;
    }
    if (!device_is_ready(dev[0]) || !device_is_ready(dev[1]))
        return -1;
    if (!ready())
        return -1;
    gpio_pin_configure_dt(&fail_dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    set_duty(0);
    return 0;
}

void driver::set_duty(int8_t dir, uint8_t d)
{
    uint32_t pulse_ns[2];
    motor_driver_calc::calc_pulse_ns(dir, d, CONTROL_PERIOD_NS, pulse_ns);
    pwm_set(dev[0], pin[0], CONTROL_PERIOD_NS, pulse_ns[0], PWM_POLARITY_NORMAL);
    pwm_set(dev[1], pin[1], CONTROL_PERIOD_NS, pulse_ns[1], PWM_POLARITY_NORMAL);
    direction = dir;
    duty = d;
}

std::tuple<int8_t, uint8_t> driver::get_duty() const
{
    return {direction, duty};
}

bool driver::ready() const
{
    return gpio_is_ready_dt(&fail_dev);
}

bool driver::is_failed() const
{
    return ready() ? gpio_pin_get_dt(&fail_dev) == 0 : false;
}

int32_t driver::get_current() const
{
    return motor_driver_calc::calc_current_ma(current_adc >= 0 ? adc_reader::get(current_adc) : 0);
}

}
