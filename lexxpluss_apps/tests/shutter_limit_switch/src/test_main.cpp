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

// The reconfirm-gating mechanism (on_edge_isr() + poll()) updates
// get_open_bit()/get_closed_bit() -- what's actually sent over CAN -- and
// get_state() -- debug display only -- from the exact same code path, so
// each case below asserts both together instead of duplicating the gating
// logic in separate tests per output.

ZTEST(shutter_limit_detector, test_initial_state)
{
    // Before the first reconfirm, get_open_bit()/get_closed_bit() must
    // decode to state::unknown (i.e. both true, per decode()'s truth table)
    // -- not (false,false), which decodes to state::between and would
    // misreport "not yet confirmed" as a definite shutter position.
    detector d;
    zassert_equal(d.get_state(), state::unknown);
    zassert_true(d.get_open_bit());
    zassert_true(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_poll_without_edge_ignores_level_change)
{
    detector d;
    // No on_edge_isr() call: a level presented to poll() must not update
    // anything, since the ISR is the only trigger to reconfirm.
    d.poll(true, false);
    zassert_equal(d.get_state(), state::unknown);
    zassert_true(d.get_open_bit());
    zassert_true(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_edge_then_poll_reconfirms)
{
    detector d;
    d.on_edge_isr();
    d.poll(true, false);
    zassert_equal(d.get_state(), state::open);
    zassert_true(d.get_open_bit());
    zassert_false(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_pending_flag_clears_after_poll)
{
    detector d;
    d.on_edge_isr();
    d.poll(true, false);
    // Level change without a new edge must not move anything further.
    d.poll(false, true);
    zassert_equal(d.get_state(), state::open);
    zassert_true(d.get_open_bit());
    zassert_false(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_transient_noise_still_reconfirms_current_level)
{
    detector d;
    d.on_edge_isr();
    d.poll(false, false);
    zassert_equal(d.get_state(), state::between);
    zassert_false(d.get_open_bit());
    zassert_false(d.get_closed_bit());

    d.on_edge_isr();
    d.poll(true, false);
    zassert_equal(d.get_state(), state::open);
    zassert_true(d.get_open_bit());
    zassert_false(d.get_closed_bit());
}

ZTEST(shutter_limit_detector, test_power_on_mask_boundary)
{
    zassert_true(is_power_on_masked(0));
    zassert_true(is_power_on_masked(99));
    zassert_false(is_power_on_masked(100));
    zassert_false(is_power_on_masked(101));
}
