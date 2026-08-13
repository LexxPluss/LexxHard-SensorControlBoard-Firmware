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

#include <zephyr/ztest.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include "gpio_fault_detector.hpp"

using namespace lexxhard::motor_driver;

ZTEST_SUITE(gpio_fault_detector, NULL, NULL, NULL, NULL, NULL);

// Real gpio_dt_spec backed by native_sim's built-in gpio0 (zephyr,gpio-emul)
// -- exercises the actual Zephyr GPIO API (gpio_is_ready_dt/gpio_pin_get_dt),
// not a restated assertion. Built directly from the device, bypassing the
// devicetree "gpios" phandle-array property expansion entirely (no overlay
// needed -- gpio0 is already native_sim's own node).
static const gpio_dt_spec emulated_spec{
    DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    0,
    GPIO_ACTIVE_HIGH,
};

ZTEST(gpio_fault_detector, test_ready_and_high_not_failed)
{
    gpio_fault_detector det;
    det.bind(emulated_spec);
    det.configure_input();
    gpio_emul_input_set(emulated_spec.port, emulated_spec.pin, 1);

    zassert_true(det.ready());
    zassert_false(det.is_failed());
}

ZTEST(gpio_fault_detector, test_ready_and_low_is_failed)
{
    gpio_fault_detector det;
    det.bind(emulated_spec);
    det.configure_input();
    gpio_emul_input_set(emulated_spec.port, emulated_spec.pin, 0);

    zassert_true(det.ready());
    zassert_true(det.is_failed());
}

ZTEST(gpio_fault_detector, test_not_ready_gpio)
{
    gpio_fault_detector det;
    gpio_dt_spec const unready_spec{nullptr, 0, 0};
    det.bind(unready_spec);

    zassert_false(det.ready());
    zassert_false(det.is_failed());
}
