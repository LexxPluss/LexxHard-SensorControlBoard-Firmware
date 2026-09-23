/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The rule under test: while the image is UNARMED nothing about threads is judged -- after a CAN DFU
 * the SCB boots about 90 s before the robot PC and every synchronous send blocks until the PC is on
 * the bus, which once reset a healthy board -- only the unarmed revert deadline applies. `arm`
 * refuses unless every watched activity has completed on this boot and nothing is in flight, and
 * from the moment it succeeds everything is judged, including the first L7 open and the first grid
 * send. A false stop costs a revert to E3; a missed one costs a silent board. Both are tested.
 */

#include <zephyr/ztest.h>

#include "tof_diag_hang.hpp"
#include "tof_diag_i2c.hpp"

namespace d = lexxhard::tof_diag;

namespace {

/* A healthy ARMED board at time t: everything balanced, the zcan loop moving, armed at 100 s. */
constexpr uint32_t kArmedAt{100'000};
/* The first instant at which the bounds apply. */
constexpr uint32_t kWatched{kArmedAt + d::kGraceMs};

d::watch_input healthy(uint32_t t)
{
    d::watch_input in{};
    in.now_ms = t;
    in.acq_begin = in.acq_end = t / 20;
    in.send_begin = in.send_end = t / 5;
    in.health_begin = in.health_end = t / 200;
    in.zcan_loops = t;
    in.armed = true;
    in.armed_ms = kArmedAt;
    return in;
}

d::arm_input arm_ready()
{
    d::arm_input in{};
    in.acq_begin = in.acq_end = 500;
    in.send_begin = in.send_end = 3000;
    in.health_begin = in.health_end = 40;
    in.zcan_loops = 900'000;
    in.slot_active[0] = in.slot_active[1] = false;
    return in;
}

} // namespace

ZTEST_SUITE(tof_diag_watch, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_diag_watch, test_a_healthy_board_is_always_fed)
{
    d::watch_state st{};
    for (uint32_t t{kArmedAt}; t < 900'000; t += 1000)
        zassert_equal(d::evaluate(st, healthy(t)), 0u, "a healthy board was refused at %u ms", t);
}

ZTEST(tof_diag_watch, test_nothing_is_refused_during_the_grace_period_after_arming)
{
    d::watch_state st{};
    d::watch_input in{healthy(kArmedAt)};
    in.send_begin = in.send_end + 1; // stuck from the moment it was armed
    for (uint32_t t{kArmedAt}; t < kArmedAt + d::kGraceMs; t += 1000) {
        in.now_ms = t;
        zassert_equal(d::evaluate(st, in), 0u, "refused inside the grace period at %u ms", t);
    }
}

/* --- unarmed: the boot is not judged, but it does not last forever --- */

ZTEST(tof_diag_watch, test_an_unarmed_image_is_never_judged_on_threads)
{
    /* Exactly the first GATE 0 boot: the PC is not back, so the first heartbeat send never returns,
     * the zcan loop has not gone round once and no cycle has run. It must still be fed. */
    d::watch_state st{};
    d::watch_input in{};
    in.armed = false;
    in.send_begin = 1;
    in.send_end = 0;
    in.health_begin = 1;
    in.health_end = 0;
    in.zcan_loops = 0;
    for (uint32_t t{0}; t < d::kUnarmedRevertMs; t += 1000) {
        in.now_ms = t;
        zassert_equal(d::evaluate(st, in), 0u, "an unarmed image was judged at %u ms", t);
    }
}

ZTEST(tof_diag_watch, test_an_image_nobody_armed_gives_the_board_back)
{
    d::watch_state st{};
    d::watch_input in{};
    in.armed = false;
    in.now_ms = d::kUnarmedRevertMs - 1000;
    zassert_equal(d::evaluate(st, in), 0u);
    in.now_ms = d::kUnarmedRevertMs;
    zassert_equal(d::evaluate(st, in), static_cast<uint32_t>(d::unarmed_timeout),
                  "an unarmed image was not handed back at the deadline");
}

/* --- arm refuses until the baseline is healthy --- */

ZTEST(tof_diag_watch, test_arm_is_allowed_only_from_a_healthy_baseline)
{
    zassert_equal(d::arm_blockers(arm_ready()), 0u, "a healthy baseline was refused");
}

ZTEST(tof_diag_watch, test_arm_refuses_the_post_dfu_boot)
{
    /* The GATE 0 state: one heartbeat begun and never returned, nothing else has run. */
    d::arm_input in{};
    in.send_begin = 1;
    in.health_begin = 1;
    in.slot_active[1] = true;
    const uint32_t why{d::arm_blockers(in)};
    zassert_true(why & d::arm_no_send);
    zassert_true(why & d::arm_no_health);
    zassert_true(why & d::arm_no_zcan);
    zassert_true(why & d::arm_no_cycle);
    zassert_true(why & d::arm_send_in_flight);
    zassert_true(why & d::arm_health_in_flight);
    zassert_true(why & d::arm_slot_active);
}

ZTEST(tof_diag_watch, test_arm_refuses_each_missing_piece_on_its_own)
{
    struct {
        const char *what;
        d::arm_input in;
        uint32_t bit;
    } cases[]{
        {"no send", arm_ready(), d::arm_no_send},
        {"no health", arm_ready(), d::arm_no_health},
        {"no zcan", arm_ready(), d::arm_no_zcan},
        {"no cycle", arm_ready(), d::arm_no_cycle},
        {"send in flight", arm_ready(), d::arm_send_in_flight},
        {"health in flight", arm_ready(), d::arm_health_in_flight},
        {"cycle in flight", arm_ready(), d::arm_cycle_in_flight},
        {"slot active", arm_ready(), d::arm_slot_active},
    };
    cases[0].in.send_end = 0;
    cases[0].in.send_begin = 0;
    cases[1].in.health_end = 0;
    cases[1].in.health_begin = 0;
    cases[2].in.zcan_loops = 0;
    cases[3].in.acq_end = 0;
    cases[3].in.acq_begin = 0;
    cases[4].in.send_begin += 1;
    cases[5].in.health_begin += 1;
    cases[6].in.acq_begin += 1;
    cases[7].in.slot_active[0] = true;
    for (const auto &c : cases)
        zassert_true(d::arm_blockers(c.in) & c.bit, "%s was not refused", c.what);
}

ZTEST(tof_diag_watch, test_everything_is_watched_immediately_after_the_grace)
{
    /* The first L7 open or the first static-grid send stuck right after arming is caught: nothing
     * has to succeed once more after `arm`. */
    d::watch_state st{};
    d::watch_input in{healthy(kArmedAt)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.send_begin = in.send_end + 1;
    in.acq_begin = in.acq_end + 1;
    in.now_ms = kArmedAt + d::kGraceMs + d::kCycleBoundMs + 1000;
    in.zcan_loops = in.now_ms;
    const uint32_t why{d::evaluate(st, in)};
    zassert_true(why & d::stuck_send);
    zassert_true(why & d::stuck_cycle);
}

ZTEST(tof_diag_watch, test_an_idle_acquisition_is_not_a_stuck_one)
{
    /* Commissioning holds the chain for seconds: no cycle begins, none ends. Not stuck. */
    d::watch_state st{};
    d::watch_input in{healthy(100'000)};
    for (uint32_t t{kWatched}; t < kWatched + 100'000; t += 1000) {
        in.now_ms = t;
        in.zcan_loops = t;
        zassert_equal(d::evaluate(st, in), 0u, "an idle but balanced acquisition was refused");
    }
}

ZTEST(tof_diag_watch, test_a_send_that_never_returns_is_refused_after_its_bound)
{
    d::watch_state st{};
    d::watch_input in{healthy(kWatched)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.send_begin = in.send_end + 1; // one send in flight, and it stays there
    uint32_t refused_at{0};
    for (uint32_t t{kWatched + 1000}; t < kWatched + 10'000; t += 1000) {
        in.now_ms = t;
        in.zcan_loops = t;
        if (d::evaluate(st, in) & d::stuck_send) {
            refused_at = t;
            break;
        }
    }
    zassert_true(refused_at != 0, "a send stuck for 9 s was never refused");
    zassert_true(refused_at - kWatched > d::kSendBoundMs, "refused before its bound");
    zassert_true(refused_at - kWatched <= d::kSendBoundMs + 1000, "refused later than one tick past its bound");
}

ZTEST(tof_diag_watch, test_a_send_in_flight_that_keeps_completing_others_is_fine)
{
    /* Concurrent senders: begin is often end + 1 at the instant of sampling, but ends keep moving. */
    d::watch_state st{};
    for (uint32_t t{kWatched}; t < kWatched + 100'000; t += 1000) {
        d::watch_input in{healthy(t)};
        in.send_begin = in.send_end + 1;
        zassert_equal(d::evaluate(st, in), 0u, "a busy but moving send path was refused");
    }
}

ZTEST(tof_diag_watch, test_a_cycle_holding_both_uld_downloads_is_not_stuck)
{
    d::watch_state st{};
    d::watch_input in{healthy(kWatched)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.acq_begin = in.acq_end + 1;
    for (uint32_t t{kWatched + 1000}; t <= kWatched + d::kCycleBoundMs; t += 1000) {
        in.now_ms = t;
        in.zcan_loops = t;
        zassert_equal(d::evaluate(st, in) & d::stuck_cycle, 0u, "a long cycle was refused at %u", t);
    }
    in.now_ms = kWatched + d::kCycleBoundMs + 1000;
    in.zcan_loops = in.now_ms;
    zassert_true(d::evaluate(st, in) & d::stuck_cycle, "a cycle stuck past its bound was not refused");
}

ZTEST(tof_diag_watch, test_a_stuck_health_item_is_refused)
{
    d::watch_state st{};
    d::watch_input in{healthy(kWatched)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.health_begin = in.health_end + 1;
    in.now_ms = kWatched + d::kHealthBoundMs + 1000;
    in.zcan_loops = in.now_ms;
    zassert_true(d::evaluate(st, in) & d::stuck_health);
}

ZTEST(tof_diag_watch, test_a_stopped_zcan_loop_is_refused_even_with_nothing_in_flight)
{
    d::watch_state st{};
    d::watch_input in{healthy(kWatched)};
    zassert_equal(d::evaluate(st, in), 0u);
    in.now_ms = kWatched + d::kZcanBoundMs; // exactly at the bound: still fed
    zassert_equal(d::evaluate(st, in) & d::stuck_zcan, 0u);
    in.now_ms = kWatched + d::kZcanBoundMs + 1000;
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
    in.armed_ms = t0 - 100'000; // armed just before the wrap
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

/* The other half of the diagnosis: what one i2c2 event-interrupt entry means.
 *
 * The Zephyr 3.6 STM32 I2C v2 event ISR advances current.buf and current.len whenever
 * current.len != 0, without checking that this entry carried RXNE or TXIS. These cases are the two
 * ends of the desync that follows -- the entry that consumes a byte which never arrived, and the
 * byte that then finds current.len == 0 and is never read, leaving RXNE asserted for ever. The
 * classification is pure so this test and the interrupt agree by construction. */
namespace i2c = lexxhard::tof_diag_i2c;

ZTEST(tof_diag_i2c_entry, test_ordinary_transfers_are_not_signatures)
{
    /* A byte arriving with room for it, and a byte going out with more to send: the normal case,
     * which must never be counted as a fault or the measurement is worthless. */
    zassert_equal(i2c::classify_event(i2c::bit_rxne, 5), 0U);
    zassert_equal(i2c::classify_event(i2c::bit_txis, 5), 0U);
    /* The last byte of a chunk arrives together with the reload boundary. */
    zassert_equal(i2c::classify_event(i2c::bit_rxne | i2c::bit_tcr, 1), 0U);
}

ZTEST(tof_diag_i2c_entry, test_reload_boundary_with_nothing_left_is_not_a_phantom)
{
    /* TCR or TC after the chunk is fully transferred is how a 255 + 73 read legitimately ends:
     * len is already 0, so nothing is advanced and nothing is stuck. */
    zassert_equal(i2c::classify_event(i2c::bit_tcr, 0), 0U);
    zassert_equal(i2c::classify_event(i2c::bit_tc, 0), 0U);
    zassert_equal(i2c::classify_event(i2c::bit_stopf, 0), 0U);
}

ZTEST(tof_diag_i2c_entry, test_entry_without_data_while_bytes_remain_is_the_phantom_byte)
{
    /* Bytes still expected, but this entry carried no data flag: the driver advances anyway. */
    zassert_equal(i2c::classify_event(i2c::bit_tcr, 3), i2c::entry_advance_no_data);
    zassert_equal(i2c::classify_event(i2c::bit_nackf, 3), i2c::entry_advance_no_data);
    zassert_equal(i2c::classify_event(i2c::bit_stopf, 1), i2c::entry_advance_no_data);
    /* ADDR is not a controller-transfer flag, so it counts as no flag at all AND advances. */
    zassert_equal(i2c::classify_event(i2c::bit_addr, 2),
                  i2c::entry_advance_no_data | i2c::entry_no_flag);
}

ZTEST(tof_diag_i2c_entry, test_rxne_with_nothing_left_but_a_flag_to_act_on_is_ordinary)
{
    /* Measured on the board: this state is reached about once per read, 13.8 million times in the
     * run that hung, and left again within microseconds -- because the same entry carries a
     * completion or an abort that the handler does act on. It must NOT be reported as the fault,
     * or the fault indicator is worthless. */
    const uint32_t serviceable[]{i2c::bit_tc, i2c::bit_tcr, i2c::bit_stopf, i2c::bit_nackf};
    for (uint32_t flag : serviceable)
        zassert_equal(i2c::classify_event(i2c::bit_rxne | flag, 0), i2c::entry_rxne_with_len0,
                      "RXNE with 0x%x must not be the fault", flag);
    zassert_equal(i2c::classify_event(i2c::bit_txis, 0), i2c::entry_txis_with_len0);
}

ZTEST(tof_diag_i2c_entry, test_rxne_with_nothing_left_and_nothing_to_act_on_is_the_fault)
{
    /* The wedge itself, as the board recorded it: ISR 0x8005 -- RXNE, TXE, BUSY -- with len 0 and
     * no TC, TCR, STOPF or NACKF. The handler can neither consume the byte nor mask the line. */
    const uint32_t wedged{i2c::bit_rxne | 1U /* TXE */ | (1U << 15) /* BUSY */};
    zassert_equal(i2c::classify_event(wedged, 0),
                  i2c::entry_rxne_with_len0 | i2c::entry_orphan_rxne);
    /* And it keeps being reported, entry after entry -- the count is the storm. */
    for (int i{0}; i < 4; ++i)
        zassert_equal(i2c::classify_event(i2c::bit_rxne, 0),
                      i2c::entry_rxne_with_len0 | i2c::entry_orphan_rxne);
}

ZTEST(tof_diag_i2c_entry, test_flags_already_gone_when_the_isr_read_them)
{
    /* The shape the later upstream rework of this driver exists for. With bytes outstanding it is
     * also a phantom byte; with none it is only a spurious entry. */
    zassert_equal(i2c::classify_event(0, 0), i2c::entry_no_flag);
    zassert_equal(i2c::classify_event(0, 7), i2c::entry_no_flag | i2c::entry_advance_no_data);
}

ZTEST(tof_diag_i2c_entry, test_every_error_source_errie_can_raise_has_a_bucket)
{
    /* PEC, SMBus timeout and SMBAlert are not configured on this bus. That is a reason to give them
     * their own counters, not to leave them falling into "an error we did not classify": an error
     * source nobody clears is the shortest path to an interrupt that re-enters for ever, and it
     * would be the most valuable thing this run could find. */
    const uint32_t every[]{i2c::bit_berr,   i2c::bit_arlo,    i2c::bit_ovr,
                           i2c::bit_pecerr, i2c::bit_timeout, i2c::bit_alert};
    for (uint32_t bit : every)
        zassert_true((i2c::kAnyErrorFlag & bit) != 0U, "an error source with no bucket: 0x%x", bit);
}

ZTEST(tof_diag_i2c_entry, test_error_flags_are_not_counted_as_a_flagless_entry)
{
    /* An event entry carrying only an error flag has carried something, so it must not inflate the
     * "flags had already gone" count, which is the one that would point at a different fault. */
    const uint32_t errors[]{i2c::bit_pecerr, i2c::bit_timeout, i2c::bit_alert, i2c::bit_ovr};
    for (uint32_t bit : errors)
        zassert_equal(i2c::classify_event(bit, 0), 0U, "bit 0x%x counted as flagless", bit);
    /* An error flag is not something the handler's completion paths act on, so an RXNE that
     * arrives with only an error beside it is still the orphan. */
    zassert_equal(i2c::classify_event(i2c::bit_rxne | i2c::bit_ovr, 0),
                  i2c::entry_rxne_with_len0 | i2c::entry_orphan_rxne);
}

ZTEST_SUITE(tof_diag_i2c_entry, NULL, NULL, NULL, NULL, NULL);
