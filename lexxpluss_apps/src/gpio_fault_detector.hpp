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

#pragma once

#include <zephyr/drivers/gpio.h>

namespace lexxhard::motor_driver {

// Extracted from motor_driver::driver so the GPIO-read branching logic can
// be ztest'd via gpio_emul without pulling in PWM devicetree nodes (see
// TESTPLAN_shutter_controller_20260714.md section 4.2).
class gpio_fault_detector {
public:
    void bind(const gpio_dt_spec &d) { dev = d; }
    bool ready() const { return gpio_is_ready_dt(&dev); }
    // ACTIVE_HIGH config: pin reads LOW when the fault line is asserted.
    bool is_failed() const { return ready() ? gpio_pin_get_dt(&dev) == 0 : false; }
    void configure_input() { gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_ACTIVE_HIGH); }
private:
    gpio_dt_spec dev{};
};

}
