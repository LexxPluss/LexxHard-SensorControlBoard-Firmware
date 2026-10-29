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

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <zephyr/ztest.h>

#include "lcs_sync_schedule.hpp"

using lexxhard::lcs::DownlinkFrame;
using lexxhard::lcs::SyncOutcome;
using lexxhard::lcs::SyncPoll;
using lexxhard::lcs::SyncScheduler;

ZTEST_SUITE(lcs_sync_schedule, NULL, NULL, NULL, NULL, NULL);

namespace {

/* 10 ms at 100 us per tick. */
constexpr uint32_t kPeriod{100};

/* Table-driven CRC16-CCITT (0x1021, initial 0xFFFF), independent of the bitwise code under test. */
uint16_t ref_crc16(const uint8_t *data, const size_t count)
{
    static const std::array<uint16_t, 256> table = [] {
        std::array<uint16_t, 256> t{};
        for (uint32_t i{0}; i < 256U; ++i) {
            uint32_t r{i << 8};
            for (int bit{0}; bit < 8; ++bit) {
                r = (r & 0x8000U) != 0U ? ((r << 1) ^ 0x1021U) & 0xFFFFU : (r << 1) & 0xFFFFU;
            }
            t[i] = static_cast<uint16_t>(r);
        }
        return t;
    }();
    uint32_t crc{0xFFFFU};
    for (size_t i{0}; i < count; ++i) {
        crc = ((crc << 8) & 0xFFFFU) ^ table[((crc >> 8) ^ data[i]) & 0xFFU];
    }
    return static_cast<uint16_t>(crc);
}

/* Reference model of the schedule on an unwrapped 64-bit time line, plain signed integers. */
struct Model {
    bool started{false};
    int64_t due{0};

    struct Result {
        SyncOutcome outcome;
        bool late;
    };

    Result poll(const int64_t now, const bool accept)
    {
        if (!started) {
            started = true;
            due = now;
        }
        const int64_t behind{now - due};
        if (behind < 0) {
            return {SyncOutcome::NotDue, false};
        }
        const bool late{behind >= static_cast<int64_t>(kPeriod)};
        due = late ? now + kPeriod : due + kPeriod;
        return {accept ? SyncOutcome::Sent : SyncOutcome::Skipped, late};
    }

    /* Ticks until the next SYNC is due, clamped to cap; 0 when not started or due. */
    uint32_t ticks_until_due(const int64_t now, const uint32_t cap) const
    {
        if (!started || now >= due) {
            return 0;
        }
        const int64_t remaining{due - now};
        return remaining > static_cast<int64_t>(cap) ? cap : static_cast<uint32_t>(remaining);
    }
};

/* Records the frames handed to the transmitter and answers with a fixed acceptance. */
struct Sink {
    bool accept{true};
    std::vector<DownlinkFrame> frames;

    bool operator()(const DownlinkFrame &frame)
    {
        frames.push_back(frame);
        return accept;
    }
};

struct Step {
    uint32_t dt; /* ticks since the first poll */
    bool accept;
};

uint32_t wrapped(const uint64_t base, const uint64_t t)
{
    return static_cast<uint32_t>((base + t) & 0xFFFFFFFFULL);
}

/* Compares ticks_until_due of the scheduler with the model for several caps at one tick. */
void check_remaining(const SyncScheduler &scheduler, const Model &model, const uint64_t base, const uint32_t dt,
                     const size_t i)
{
    const std::array<uint32_t, 4> caps{1U, 10U, kPeriod, 0xFFFFFFFFU};
    for (const uint32_t cap : caps) {
        zassert_equal(scheduler.ticks_until_due(wrapped(base, dt), cap), model.ticks_until_due(dt, cap),
                      "ticks_until_due at step %zu cap %u", i, cap);
    }
}

/* Replays steps on both the scheduler (wrapped ticks) and the model (unwrapped time). */
void replay(const uint64_t base, const std::vector<Step> &steps)
{
    SyncScheduler scheduler;
    Model model;
    for (size_t i{0}; i < steps.size(); ++i) {
        Sink sink;
        sink.accept = steps[i].accept;
        check_remaining(scheduler, model, base, steps[i].dt, i);
        const bool was_due{scheduler.due(wrapped(base, steps[i].dt))};
        const SyncPoll got{scheduler.poll(wrapped(base, steps[i].dt), sink)};
        const Model::Result want{model.poll(static_cast<int64_t>(steps[i].dt), steps[i].accept)};
        zassert_equal(static_cast<int>(got.outcome), static_cast<int>(want.outcome), "outcome at step %zu", i);
        zassert_equal(got.late, want.late, "late at step %zu", i);
        zassert_equal(was_due, want.outcome != SyncOutcome::NotDue, "due() at step %zu", i);
        zassert_equal(sink.frames.size(), want.outcome == SyncOutcome::NotDue ? 0U : 1U, "sends at step %zu", i);
        zassert_equal(scheduler.next_due(), wrapped(base, static_cast<uint64_t>(model.due)), "next_due at step %zu", i);
        check_remaining(scheduler, model, base, steps[i].dt, i);
    }
}

}

ZTEST(lcs_sync_schedule, test_lcs_sync_first_poll_sends_at_once)
{
    SyncScheduler scheduler;
    Sink sink;
    const SyncPoll result{scheduler.poll(1234, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 1U);
    zassert_equal(scheduler.next_due(), 1234U + 100U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_poll_before_due_is_not_due)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    zassert_false(scheduler.due(1099));
    const SyncPoll result{scheduler.poll(1099, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 1U);
    zassert_equal(scheduler.next_due(), 1100U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_poll_at_due_tick_sends)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    zassert_true(scheduler.due(1100));
    const SyncPoll result{scheduler.poll(1100, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 2U);
    zassert_equal(scheduler.next_due(), 1200U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_late_by_period_minus_one_keeps_grid)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    const SyncPoll result{scheduler.poll(1199, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 2U);
    zassert_equal(scheduler.next_due(), 1200U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_late_by_one_period_realigns)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    const SyncPoll result{scheduler.poll(1200, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_true(result.late);
    zassert_equal(sink.frames.size(), 2U);
    zassert_equal(scheduler.next_due(), 1300U);
    /* One SYNC only: no catch-up burst. */
    const SyncPoll again{scheduler.poll(1200, sink)};
    zassert_equal(static_cast<int>(again.outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_equal(sink.frames.size(), 2U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_rejected_send_is_skipped_not_retried)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    sink.accept = false;
    const SyncPoll result{scheduler.poll(1100, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Skipped));
    zassert_false(result.late);
    zassert_equal(scheduler.next_due(), 1200U);
    const SyncPoll retry{scheduler.poll(1101, sink)};
    zassert_equal(static_cast<int>(retry.outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_equal(sink.frames.size(), 2U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_rejected_first_poll_advances_due)
{
    SyncScheduler scheduler;
    Sink sink;
    sink.accept = false;
    const SyncPoll result{scheduler.poll(500, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Skipped));
    zassert_equal(scheduler.next_due(), 600U);
    zassert_false(scheduler.due(599));
}

ZTEST(lcs_sync_schedule, test_lcs_sync_wrap_cases_repeat_across_counter_overflow)
{
    /* First poll at 0xFFFFFFFF - 110 = 4294967185; due 4294967285, then 4294967385 = 89 after the wrap. */
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(4294967185U, sink);
    zassert_equal(scheduler.next_due(), 4294967285U);

    /* SY-2 across the wrap: before the due tick. */
    zassert_equal(static_cast<int>(scheduler.poll(4294967284U, sink).outcome), static_cast<int>(SyncOutcome::NotDue));
    /* SY-3: exactly the due tick; the next due is 4294967385 mod 2^32 = 89. */
    zassert_equal(static_cast<int>(scheduler.poll(4294967285U, sink).outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_equal(scheduler.next_due(), 89U);
    /* Polls after the wrap, still before 89. */
    zassert_equal(static_cast<int>(scheduler.poll(88U, sink).outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_false(scheduler.due(88U));
    /* SY-4: period - 1 late, grid kept. */
    const SyncPoll late_less{scheduler.poll(188U, sink)};
    zassert_equal(static_cast<int>(late_less.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(late_less.late);
    zassert_equal(scheduler.next_due(), 189U);
    /* SY-5: exactly one period late, realigned to now + period. */
    const SyncPoll late_one{scheduler.poll(289U, sink)};
    zassert_equal(static_cast<int>(late_one.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_true(late_one.late);
    zassert_equal(scheduler.next_due(), 389U);
    zassert_equal(sink.frames.size(), 4U);

    /* SY-5 where the late poll itself is the one that passes the wrap. */
    SyncScheduler second;
    (void)second.poll(4294967290U, sink);
    const SyncPoll across{second.poll(94U, sink)};
    zassert_equal(static_cast<int>(across.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(across.late);
    zassert_equal(second.next_due(), 194U);
    const SyncPoll late_across{second.poll(294U, sink)};
    zassert_true(late_across.late);
    zassert_equal(second.next_due(), 394U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_poll_tables_match_the_model)
{
    const std::vector<Step> steps{
        {0, true},    {0, true},    {99, true},   {100, true},  {101, true},  {199, true},   {200, false}, {200, true},
        {299, true},  {300, true},  {500, true},  {500, true},  {600, false}, {601, true},   {699, true},  {700, true},
        {899, true},  {900, true},  {1000, true}, {1099, true}, {1199, true}, {1200, false}, {2000, true}, {2099, true},
        {2100, true}, {2200, true}};
    const std::array<uint64_t, 7> bases{0ULL,           1ULL,           0x7FFFFFFFULL - 100U, 0x80000000ULL - 60U,
                                        0xFFFFFFFFULL - 120U, 0xFFFFFFFFULL - 99U, 0xFFFFFFFFULL};
    for (const uint64_t base : bases) {
        replay(base, steps);
    }
}

ZTEST(lcs_sync_schedule, test_lcs_sync_seeded_random_polls_match_the_model)
{
    uint32_t state{0x2545F491U};
    const auto next = [&state]() {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    };
    const std::array<uint64_t, 3> bases{0ULL, 0xFFFFFFFFULL - 5000U, 0x7FFFFFFFULL - 5000U};
    for (const uint64_t base : bases) {
        std::vector<Step> steps;
        uint32_t t{0};
        for (size_t i{0}; i < 4000; ++i) {
            t += next() % 260U;
            steps.push_back({t, next() % 4U != 0U});
        }
        replay(base, steps);
    }
}

ZTEST(lcs_sync_schedule, test_lcs_sync_sent_frame_is_the_golden_sync)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(0, sink);
    zassert_equal(sink.frames.size(), 1U);
    const std::array<uint8_t, 6> body{0x4B, 0x08, 0xFF, 0x08, 0x00, 0x00};
    const uint16_t crc{ref_crc16(body.data(), body.size())};
    const DownlinkFrame &frame{sink.frames[0]};
    zassert_equal(frame.size(), 8U);
    for (size_t i{0}; i < body.size(); ++i) {
        zassert_equal(frame[i], body[i], "byte %zu", i);
    }
    zassert_equal(frame[6], static_cast<uint8_t>(crc >> 8));
    zassert_equal(frame[7], static_cast<uint8_t>(crc & 0xFFU));
}

ZTEST(lcs_sync_schedule, test_lcs_sync_period_constant_is_10ms_in_100_ticks)
{
    zassert_equal(lexxhard::lcs::kSyncPeriodUs, 10000U);
    zassert_equal(lexxhard::lcs::sync_period_ticks(lexxhard::lcs::kSyncPeriodUs), kPeriod);
    zassert_equal(SyncScheduler::kPeriodTicks, kPeriod);
}

namespace {

constexpr uint32_t kCap{10};

/* The wrap bases of the suite: no wrap, due tick exactly at the wrap, due tick at the signed midpoint. */
const std::array<uint32_t, 3> kRemainBases{0U, 0xFFFFFFFFU - kPeriod, 0x7FFFFFFFU - kPeriod};

/* Starts a scheduler with a first poll at base; the next due tick is base + kPeriod. */
SyncScheduler started_at(const uint32_t base)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(base, sink);
    return scheduler;
}

}

/* SY-T1: not started means the first poll is due at once. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_not_started_is_zero)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler;
        zassert_equal(scheduler.ticks_until_due(base, kCap), 0U);
        zassert_equal(scheduler.ticks_until_due(base + 5U, kCap), 0U);
        zassert_equal(scheduler.ticks_until_due(base, 0xFFFFFFFFU), 0U);
    }
}

/* SY-T2: exactly the due tick. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_at_due_tick_is_zero)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler{started_at(base)};
        zassert_equal(scheduler.ticks_until_due(base + kPeriod, kCap), 0U);
    }
}

/* SY-T3: overdue, where next_due() - tick wraps to a huge unsigned value. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_overdue_is_zero)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler{started_at(base)};
        const uint32_t due{base + kPeriod};
        zassert_equal(scheduler.ticks_until_due(due + 1U, kCap), 0U);
        zassert_equal(scheduler.ticks_until_due(due + kPeriod, kCap), 0U);
        zassert_equal(scheduler.ticks_until_due(due + 0x7FFFFFFFU, 0xFFFFFFFFU), 0U);
    }
}

/* SY-T4: one tick before the due tick. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_remaining_one_is_one)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler{started_at(base)};
        zassert_equal(scheduler.ticks_until_due(base + kPeriod - 1U, kCap), 1U);
    }
}

/* SY-T5: the remaining time equals the cap. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_remaining_equal_to_cap_is_cap)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler{started_at(base)};
        zassert_equal(scheduler.ticks_until_due(base + kPeriod - kCap, kCap), kCap);
    }
}

/* SY-T6: the remaining time is cap + 1. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_remaining_cap_plus_one_is_cap)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler{started_at(base)};
        zassert_equal(scheduler.ticks_until_due(base + kPeriod - kCap - 1U, kCap), kCap);
    }
}

/* SY-T7: the remaining time is far larger than the cap. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_remaining_far_above_cap_is_cap)
{
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler scheduler{started_at(base)};
        zassert_equal(scheduler.ticks_until_due(base, kCap), kCap);
        zassert_equal(scheduler.ticks_until_due(base, 1U), 1U);
        zassert_equal(scheduler.ticks_until_due(base - 1000U, kCap), kCap);
    }
}

/* SY-T8: the remaining time is computed across the counter wrap, not only at the three bases. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_is_correct_across_the_wrap)
{
    /* First poll at 0xFFFFFFFF - 29 = 4294967266; due 4294967366 = 70 after the wrap. */
    const SyncScheduler scheduler{started_at(0xFFFFFFFFU - 29U)};
    zassert_equal(scheduler.next_due(), 70U);
    /* Tick 4294967286: remaining 80, the span crosses the wrap. */
    zassert_equal(scheduler.ticks_until_due(0xFFFFFFFFU - 9U, 0xFFFFFFFFU), 80U);
    /* Tick 0xFFFFFFFF: remaining 71, the counter has not yet wrapped. */
    zassert_equal(scheduler.ticks_until_due(0xFFFFFFFFU, 0xFFFFFFFFU), 71U);
    /* Tick 0: wrapped, remaining 70. */
    zassert_equal(scheduler.ticks_until_due(0U, 0xFFFFFFFFU), 70U);
    zassert_equal(scheduler.ticks_until_due(69U, 0xFFFFFFFFU), 1U);
    zassert_equal(scheduler.ticks_until_due(70U, 0xFFFFFFFFU), 0U);
    zassert_equal(scheduler.ticks_until_due(0U, kCap), kCap);
}

/* SY-T9: zero if and only if due(), over tabulated states and the half-range state. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_is_zero_iff_due)
{
    const std::array<uint32_t, 2> caps{kCap, 0xFFFFFFFFU};
    for (const uint32_t base : kRemainBases) {
        const SyncScheduler started{started_at(base)};
        const SyncScheduler fresh;
        const uint32_t due{base + kPeriod};
        const std::array<uint32_t, 12> ticks{due,          due + 1U,    due + kPeriod, due + 0x7FFFFFFFU,
                                             due - 1U,     due - kCap,  due - kCap - 1U, due - kPeriod,
                                             due + 0x80000000U, due + 0x80000001U, due - 0x7FFFFFFFU, base};
        for (const uint32_t cap : caps) {
            for (const uint32_t tick : ticks) {
                zassert_equal(started.ticks_until_due(tick, cap) == 0U, started.due(tick), "started tick %u", tick);
                zassert_equal(fresh.ticks_until_due(tick, cap) == 0U, fresh.due(tick), "fresh tick %u", tick);
            }
            /* tick - next_due == 0x80000000: due() is false and the remaining time is 0x80000000. */
            const uint32_t half{due + 0x80000000U};
            zassert_false(started.due(half));
            zassert_equal(started.ticks_until_due(half, cap), cap < 0x80000000U ? cap : 0x80000000U);
        }
    }
}

/* SY-T10: after a poll that sent, the remaining time is the period minus the lateness. */
ZTEST(lcs_sync_schedule, test_lcs_sync_ticks_until_due_after_a_sent_poll)
{
    for (const uint32_t base : kRemainBases) {
        const uint32_t due{base + kPeriod};
        const std::array<uint32_t, 3> lateness{0U, 1U, kPeriod - 1U};
        for (const uint32_t late : lateness) {
            SyncScheduler scheduler{started_at(base)};
            Sink sink;
            const uint32_t tick{due + late};
            const SyncPoll result{scheduler.poll(tick, sink)};
            zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
            zassert_false(result.late);
            zassert_equal(scheduler.ticks_until_due(tick, 0xFFFFFFFFU), kPeriod - late, "late %u", late);
            /* The controller's cap of 10 ticks clamps a remaining time of more than 10. */
            zassert_equal(scheduler.ticks_until_due(tick, kCap), kPeriod - late < kCap ? kPeriod - late : kCap);
        }
    }
}
