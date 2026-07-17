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

#include "motor_driver_calc.hpp"

namespace lexxhard::motor_driver_calc {

void calc_pulse_ns(int8_t direction, uint8_t duty, uint32_t period_ns, uint32_t (&pulse_ns)[2])
{
    pulse_ns[0] = period_ns;
    pulse_ns[1] = period_ns;
    if (direction != 0 && duty != 0) {
        uint32_t const duty_clamped{duty > 100U ? 100U : static_cast<uint32_t>(duty)};
        uint32_t const duty_rev{100U - duty_clamped};
        uint32_t const ns{duty_rev * period_ns / 100};
        pulse_ns[direction < 0 ? 0 : 1] = ns;
    }
}

int32_t calc_current_ma(int32_t adc_voltage_mv)
{
    static constexpr float AMP_GAIN{50.0f}, VOLTAGE_DIVIDER{1.0f}, SHUNT_REGISTER{0.01f};
    float const current_a{adc_voltage_mv * 1e-3f / AMP_GAIN * VOLTAGE_DIVIDER / SHUNT_REGISTER};
    return static_cast<int32_t>(current_a * 1e+3f);
}

}
