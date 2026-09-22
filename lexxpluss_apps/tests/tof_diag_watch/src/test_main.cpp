/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The rule under test: feed while everything that has begun keeps finishing; stop feeding once
 * something has begun and not finished for longer than its bound, or the zcan loop stops going
 * round. A false stop costs a revert to E3; a missed one costs a silent board. Both directions are
 * tested.
 */

#include <zephyr/ztest.h>

#include "tof_diag_hang.hpp"

namespace d = lexxhard::tof_diag;

namespace {

/* A healthy board at time t: everything balanced, the zcan loop moving. */
d::watch_input healthy(uint32_t t)
{
    return d::watch_input{t, t / 20, t / 20, t / 5, t / 5, t / 200, t / 200, t};
}

} // namespace

ZTEST_SUITE(tof_diag_watch, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_diag_watch, test_a_healthy_board_is_always_fed)
{
    d::watch_state st{};
    for (uint32_t t{0}; t < 600'000; t += 1000)
        zassert_equal(d::evaluate(st, healthy(t)), 0u, "a healthy board was refused at %u ms", t);
}

ZTEST(tof_diag_watch, test_nothing_is_refused_during_the_grace_period)
{
    d::watch_state st{};
    d::watch_input in{healthy(0)};
    in.send_begin = in.send_end + 1; // stuck from the start
    for (uint32_t t{0}; t < d::kGraceMs; t += 1000) {
        in.now_ms = t;
        zassert_equal(d::evaluate(st, in), 0u, "refused inside the grace period at %u ms", t);
    }
}

ZTEST(tof_diag_watch, test_an_idle_acquisition_is_not_a_stuck_one)
{
    /* Commissioning holds the chain for seconds: no cycle begins, none ends. Not stuck. */
    d::watch_state st{};
    d::watch_input in{healthy(100'000)};
    for (uint32_t t{100'000}; t < 200'000; t += 1000) {
        in.now_ms = t;
        in.zcan_loops = t;
        zassert_equal(d::evaluate(st, in), 0u, "an idle but balanced acquisition was refused");
    }
}

ZTEST(tof_diag_watch, test_a_send_that_never_returns_is_refused_after_its_bound)
{
    d::watch_state st{};
    d::watch_input in{healthy(100'000)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.send_begin = in.send_end + 1; // one send in flight, and it stays there
    uint32_t refused_at{0};
    for (uint32_t t{101'000}; t < 110'000; t += 1000) {
        in.now_ms = t;
        in.zcan_loops = t;
        if (d::evaluate(st, in) & d::stuck_send) {
            refused_at = t;
            break;
        }
    }
    zassert_true(refused_at != 0, "a send stuck for 9 s was never refused");
    zassert_true(refused_at - 100'000 > d::kSendBoundMs, "refused before its bound");
    zassert_true(refused_at - 100'000 <= d::kSendBoundMs + 1000, "refused later than one tick past its bound");
}

ZTEST(tof_diag_watch, test_a_send_in_flight_that_keeps_completing_others_is_fine)
{
    /* Concurrent senders: begin is often end + 1 at the instant of sampling, but ends keep moving. */
    d::watch_state st{};
    for (uint32_t t{100'000}; t < 200'000; t += 1000) {
        d::watch_input in{healthy(t)};
        in.send_begin = in.send_end + 1;
        zassert_equal(d::evaluate(st, in), 0u, "a busy but moving send path was refused");
    }
}

ZTEST(tof_diag_watch, test_a_cycle_holding_both_uld_downloads_is_not_stuck)
{
    d::watch_state st{};
    d::watch_input in{healthy(100'000)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.acq_begin = in.acq_end + 1;
    for (uint32_t t{101'000}; t <= 100'000 + d::kCycleBoundMs; t += 1000) {
        in.now_ms = t;
        in.zcan_loops = t;
        zassert_equal(d::evaluate(st, in) & d::stuck_cycle, 0u, "a long cycle was refused at %u", t);
    }
    in.now_ms = 100'000 + d::kCycleBoundMs + 1000;
    in.zcan_loops = in.now_ms;
    zassert_true(d::evaluate(st, in) & d::stuck_cycle, "a cycle stuck past its bound was not refused");
}

ZTEST(tof_diag_watch, test_a_stuck_health_item_is_refused)
{
    d::watch_state st{};
    d::watch_input in{healthy(100'000)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.health_begin = in.health_end + 1;
    in.now_ms = 100'000 + d::kHealthBoundMs + 1000;
    in.zcan_loops = in.now_ms;
    zassert_true(d::evaluate(st, in) & d::stuck_health);
}

ZTEST(tof_diag_watch, test_a_stopped_zcan_loop_is_refused_even_with_nothing_in_flight)
{
    d::watch_state st{};
    d::watch_input in{healthy(100'000)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.now_ms = 100'000 + d::kZcanBoundMs; // exactly at the bound: still fed
    zassert_equal(d::evaluate(st, in) & d::stuck_zcan, 0u);
    in.now_ms = 100'000 + d::kZcanBoundMs + 1000;
    zassert_true(d::evaluate(st, in) & d::stuck_zcan, "a zcan loop stopped for 6 s was not refused");
}

ZTEST(tof_diag_watch, test_the_hang_seen_on_dasher2_is_refused)
{
    /* Every thread silent: sends begun and not returned, zcan loop still, acquisition inside a cycle. */
    d::watch_state st{};
    d::watch_input in{healthy(900'000)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.send_begin = in.send_end + 1;
    in.acq_begin = in.acq_end + 1;
    in.now_ms = 900'000 + 30'000;
    const uint32_t why{d::evaluate(st, in)};
    zassert_true(why & d::stuck_send);
    zassert_true(why & d::stuck_cycle);
    zassert_true(why & d::stuck_zcan);
}

ZTEST(tof_diag_watch, test_uptime_wrap_of_the_32_bit_millisecond_counter_is_harmless)
{
    d::watch_state st{};
    const uint32_t t0{0xFFFFFFFFU - 3000};
    d::watch_input in{healthy(100'000)};
    in.now_ms = t0;
    zassert_equal(d::evaluate(st, in), 0u);
    for (uint32_t k{1}; k <= 10; ++k) {
        in.now_ms = t0 + k * 1000; // wraps past zero
        in.zcan_loops += 1;
        in.send_begin += 1;
        in.send_end += 1;
        zassert_equal(d::evaluate(st, in), 0u, "refused across the wrap at step %u", k);
    }
}
