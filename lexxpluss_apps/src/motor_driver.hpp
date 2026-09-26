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

#include <cstdint>
#include <tuple>
#include <zephyr/device.h>
#include "gpio_fault_detector.hpp"

namespace lexxhard::motor_driver {

enum class axis { CENTER, LEFT, RIGHT };

// PWM 2-pin H-bridge drive + ADC current sense + GPIO fault pin, shared by
// every axis (Center/Left/Right). Each axis owns its own independent
// instance -- Major loop (Left/Right) and Minor loop (Center) never share
// one, so no cross-loop write conflict. See motor_driver_calc.hpp for the
// stateless conversion formulas this delegates to.
class driver {
public:
    int init(axis a);
    void set_duty(int8_t direction, uint8_t duty = 0);
    std::tuple<int8_t, uint8_t> get_duty() const;
    bool ready() const;
    bool is_failed() const;
    int32_t get_current() const;
private:
    uint32_t pin[2]{0, 0};
    int8_t direction{0};
    uint8_t duty{0};
    const device *dev[2]{nullptr, nullptr};
    gpio_fault_detector fail_gpio{};
    int32_t current_adc{-1};
    static constexpr uint32_t CONTROL_HZ{10000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
};

}
