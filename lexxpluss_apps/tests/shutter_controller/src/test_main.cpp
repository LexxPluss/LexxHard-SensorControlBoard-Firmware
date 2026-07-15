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
#include "shutter_controller.hpp"

using namespace lexxhard::shutter_controller;

ZTEST_SUITE(shutter_controller, NULL, NULL, NULL, NULL, NULL);

ZTEST(shutter_controller, test_stop_request_always_stops)
{
    auto const cmd{decide_drive(state::between, request::stop, 50)};
    zassert_equal(cmd.direction, request::stop);
    zassert_equal(cmd.duty, 0);
}

ZTEST(shutter_controller, test_zero_duty_always_stops)
{
    auto const cmd{decide_drive(state::between, request::toward_open, 0)};
    zassert_equal(cmd.direction, request::stop);
    zassert_equal(cmd.duty, 0);
}

ZTEST(shutter_controller, test_open_blocks_further_toward_open)
{
    auto const cmd{decide_drive(state::open, request::toward_open, 50)};
    zassert_equal(cmd.direction, request::stop);
    zassert_equal(cmd.duty, 0);
}

ZTEST(shutter_controller, test_open_allows_toward_closed)
{
    auto const cmd{decide_drive(state::open, request::toward_closed, 50)};
    zassert_equal(cmd.direction, request::toward_closed);
    zassert_equal(cmd.duty, 50);
}

ZTEST(shutter_controller, test_closed_blocks_further_toward_closed)
{
    auto const cmd{decide_drive(state::closed, request::toward_closed, 50)};
    zassert_equal(cmd.direction, request::stop);
    zassert_equal(cmd.duty, 0);
}

ZTEST(shutter_controller, test_closed_allows_toward_open)
{
    auto const cmd{decide_drive(state::closed, request::toward_open, 50)};
    zassert_equal(cmd.direction, request::toward_open);
    zassert_equal(cmd.duty, 50);
}

ZTEST(shutter_controller, test_between_allows_both_directions)
{
    auto const open_cmd{decide_drive(state::between, request::toward_open, 40)};
    zassert_equal(open_cmd.direction, request::toward_open);
    zassert_equal(open_cmd.duty, 40);

    auto const closed_cmd{decide_drive(state::between, request::toward_closed, 40)};
    zassert_equal(closed_cmd.direction, request::toward_closed);
    zassert_equal(closed_cmd.duty, 40);
}

// Also covers power-on mask: the caller feeds state::unknown while masked
// (shutter_limit_detector::is_power_on_masked()), so no separate startup
// case is needed here.
ZTEST(shutter_controller, test_unknown_state_always_stops)
{
    auto const open_cmd{decide_drive(state::unknown, request::toward_open, 50)};
    zassert_equal(open_cmd.direction, request::stop);
    zassert_equal(open_cmd.duty, 0);

    auto const closed_cmd{decide_drive(state::unknown, request::toward_closed, 50)};
    zassert_equal(closed_cmd.direction, request::stop);
    zassert_equal(closed_cmd.duty, 0);
}

// TODO(direction mapping unconfirmed, see shutter_controller.hpp): this only
// checks the mapping is wired up and self-consistent, not that it matches
// the real motor's rotation direction.
ZTEST(shutter_controller, test_request_from_raw_direction)
{
    zassert_equal(request_from_raw_direction(1), request::toward_open);
    zassert_equal(request_from_raw_direction(-1), request::toward_closed);
    zassert_equal(request_from_raw_direction(0), request::stop);
    // Boundary generalization beyond the exact ±1 wire values -- the
    // implementation is sign-based (>0/<0), not an exact match against 1/-1.
    zassert_equal(request_from_raw_direction(2), request::toward_open);
    zassert_equal(request_from_raw_direction(-5), request::toward_closed);
}

// TODO(placeholder timeout, see shutter_controller.hpp): 180000ms is
// provisional pending separate confirmation of the shutter's actual
// full-travel time.
ZTEST(shutter_controller, test_stalled_boundary)
{
    // Driving toward open, never reaching it -- stalled once the timeout elapses.
    zassert_false(is_stalled(request::toward_open, state::between, 179999));
    zassert_true(is_stalled(request::toward_open, state::between, 180000));
}

ZTEST(shutter_controller, test_stalled_not_when_target_reached)
{
    // Reached the state the direction should produce -- never stalled, even
    // well past the timeout (this is normal: driving stops once decide_drive()
    // blocks further motion at the limit).
    zassert_false(is_stalled(request::toward_open, state::open, 999999));
    zassert_false(is_stalled(request::toward_closed, state::closed, 999999));
}

ZTEST(shutter_controller, test_stalled_wrong_direction_still_counts)
{
    // Reaching the *other* limit doesn't satisfy "toward_open" -- still stalled.
    zassert_true(is_stalled(request::toward_open, state::closed, 180000));
    zassert_true(is_stalled(request::toward_closed, state::open, 180000));
}

ZTEST(shutter_controller, test_stalled_stop_direction_never_stalls)
{
    zassert_false(is_stalled(request::stop, state::between, 999999));
}

ZTEST(shutter_controller, test_stall_guard_passes_through_when_ok)
{
    stall_guard guard;
    auto const cmd{guard.poll({request::toward_open, 30}, state::between, 0)};
    zassert_equal(cmd.direction, request::toward_open);
    zassert_equal(cmd.duty, 30);
    zassert_equal(guard.retry_count(), 0);
    zassert_false(guard.is_latched());
}

// Mirrors actuator_controller's fail_checker (fail_max) -- retries with a
// fresh window each time, then latches once retries exceed max_retries(10).
ZTEST(shutter_controller, test_stall_guard_retries_then_latches)
{
    stall_guard guard;
    drive_command const requested{request::toward_open, 30};
    guard.poll(requested, state::between, 0);
    for (int i = 1; i <= 11; ++i) {
        auto const cmd{guard.poll(requested, state::between, i * ARRIVAL_TIMEOUT_MS)};
        zassert_equal(cmd.direction, request::stop);
        zassert_equal(guard.retry_count(), i);
    }
    zassert_true(guard.is_latched());
    // Once latched, it stays stopped even with a short elapsed time --
    // it no longer grants a fresh retry window at all.
    auto const cmd{guard.poll(requested, state::between, 11 * ARRIVAL_TIMEOUT_MS + 1)};
    zassert_equal(cmd.direction, request::stop);
}

ZTEST(shutter_controller, test_stall_guard_clears_when_target_reached)
{
    stall_guard guard;
    drive_command const requested{request::toward_open, 30};
    guard.poll(requested, state::between, 0);
    guard.poll(requested, state::between, ARRIVAL_TIMEOUT_MS);
    zassert_equal(guard.retry_count(), 1);
    // decide_drive() itself now returns stop, since state::open blocks
    // further toward_open -- the target was reached after all.
    auto const cmd{guard.poll({request::stop, 0}, state::open, ARRIVAL_TIMEOUT_MS + 1)};
    zassert_equal(cmd.direction, request::stop);
    zassert_equal(guard.retry_count(), 0);
    zassert_false(guard.is_latched());
}

ZTEST(shutter_controller, test_stall_guard_clears_on_direction_change)
{
    stall_guard guard;
    drive_command const requested{request::toward_open, 30};
    guard.poll(requested, state::between, 0);
    guard.poll(requested, state::between, ARRIVAL_TIMEOUT_MS);
    zassert_equal(guard.retry_count(), 1);
    auto const cmd{guard.poll({request::toward_closed, 30}, state::between, ARRIVAL_TIMEOUT_MS + 1)};
    zassert_equal(cmd.direction, request::toward_closed);
    zassert_equal(guard.retry_count(), 0);
}

// TODO(placeholder threshold, see shutter_controller.hpp).
ZTEST(shutter_controller, test_command_stale_boundary)
{
    zassert_false(is_command_stale(0));
    zassert_false(is_command_stale(249));
    zassert_true(is_command_stale(250));
    zassert_true(is_command_stale(251));
}
