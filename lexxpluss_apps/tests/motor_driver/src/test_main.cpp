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
#include "motor_driver_calc.hpp"

using namespace lexxhard::motor_driver_calc;

ZTEST_SUITE(motor_driver_calc, NULL, NULL, NULL, NULL, NULL);

// direction mirrors msg_control::DOWN(-1)/STOP(0)/UP(1) (actuator_controller.hpp).
constexpr int8_t DOWN{-1}, STOP{0}, UP{1};
constexpr uint32_t PERIOD_NS{100000};

ZTEST(motor_driver_calc, test_stop_direction_no_output_regardless_of_duty)
{
    uint32_t pulse_ns[2];
    calc_pulse_ns(STOP, 50, PERIOD_NS, pulse_ns);
    zassert_equal(pulse_ns[0], PERIOD_NS);
    zassert_equal(pulse_ns[1], PERIOD_NS);
}

ZTEST(motor_driver_calc, test_zero_duty_no_output_regardless_of_direction)
{
    uint32_t pulse_ns[2];
    calc_pulse_ns(UP, 0, PERIOD_NS, pulse_ns);
    zassert_equal(pulse_ns[0], PERIOD_NS);
    zassert_equal(pulse_ns[1], PERIOD_NS);
}

ZTEST(motor_driver_calc, test_up_full_duty)
{
    uint32_t pulse_ns[2];
    calc_pulse_ns(UP, 100, PERIOD_NS, pulse_ns);
    zassert_equal(pulse_ns[1], 0);
    zassert_equal(pulse_ns[0], PERIOD_NS);
}

ZTEST(motor_driver_calc, test_up_half_duty)
{
    uint32_t pulse_ns[2];
    calc_pulse_ns(UP, 50, PERIOD_NS, pulse_ns);
    zassert_equal(pulse_ns[1], PERIOD_NS / 2);
    zassert_equal(pulse_ns[0], PERIOD_NS);
}

ZTEST(motor_driver_calc, test_down_full_duty)
{
    uint32_t pulse_ns[2];
    calc_pulse_ns(DOWN, 100, PERIOD_NS, pulse_ns);
    zassert_equal(pulse_ns[0], 0);
    zassert_equal(pulse_ns[1], PERIOD_NS);
}

ZTEST(motor_driver_calc, test_down_half_duty)
{
    uint32_t pulse_ns[2];
    calc_pulse_ns(DOWN, 50, PERIOD_NS, pulse_ns);
    zassert_equal(pulse_ns[0], PERIOD_NS / 2);
    zassert_equal(pulse_ns[1], PERIOD_NS);
}

ZTEST(motor_driver_calc, test_current_zero_voltage)
{
    zassert_equal(calc_current_ma(0), 0);
}

ZTEST(motor_driver_calc, test_current_known_voltage)
{
    // 1000mV * 1e-3 / 50 * 1 / 0.01 = 2.0A = 2000mA.
    zassert_equal(calc_current_ma(1000), 2000);
}
