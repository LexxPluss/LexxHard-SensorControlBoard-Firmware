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
#include "shutter_limit_detector.hpp"

using namespace lexxhard::shutter_limit_detector;

ZTEST_SUITE(shutter_limit_detector, NULL, NULL, NULL, NULL, NULL);

// decode()/to_cstr() are pure-function correctness checks for the debug
// display path (`shutter_limit_switch info`); they don't touch the
// reconfirm-gating mechanism, so they're tested standalone.
ZTEST(shutter_limit_detector, test_decode_truth_table)
{
    zassert_equal(detector::decode(true, false), state::open);
    zassert_equal(detector::decode(false, true), state::closed);
    zassert_equal(detector::decode(false, false), state::between);
    zassert_equal(detector::decode(true, true), state::unknown);
}

ZTEST(shutter_limit_detector, test_to_cstr)
{
    zassert_mem_equal(to_cstr(state::unknown), "unknown", 7);
    zassert_mem_equal(to_cstr(state::open), "open", 4);
    zassert_mem_equal(to_cstr(state::closed), "closed", 6);
    zassert_mem_equal(to_cstr(state::between), "between", 7);
}

// EMX4-T12C is a non-contact sensor (no mechanical bounce), so poll() is a
// plain, unconditional level-copy every cycle -- no EXTI, no edge gating.
// (EXTI was dropped: STM32 EXTI lines are shared by pin NUMBER across GPIO
// ports, and the Open signal's EXTI line was already claimed by another
// sensor's interrupt -- see
// INVESTIGATION_shutter_limit_switch_exti_conflict_20260713.md.)

ZTEST(shutter_limit_detector, test_initial_bits)
{
    // (true,true) so decode() reports unknown, not between, before the
    // first poll().
    detector d;
    zassert_true(d.get_open_bit());
    zassert_true(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_poll_copies_levels_unconditionally)
{
    detector d;
    d.poll(true, false);
    zassert_true(d.get_open_bit());
    zassert_false(d.get_closed_bit());

    d.poll(false, true);
    zassert_false(d.get_open_bit());
    zassert_true(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_get_state_derives_from_confirmed_bits)
{
    // Confirms get_state() is wired to decode(get_open_bit(),
    // get_closed_bit()) -- decode()'s own truth table is already fully
    // covered by test_decode_truth_table, so this only checks the wiring,
    // not every input combination.
    detector d;
    d.poll(true, false);
    zassert_equal(d.get_state(), state::open);
}

ZTEST(shutter_limit_detector, test_power_on_mask_boundary)
{
    zassert_true(is_power_on_masked(0));
    zassert_true(is_power_on_masked(99));
    zassert_false(is_power_on_masked(100));
    zassert_false(is_power_on_masked(101));
}
