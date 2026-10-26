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

/* 5 ms at 100 us per tick. */
constexpr uint32_t kPeriod{50};

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

/* Replays steps on both the scheduler (wrapped ticks) and the model (unwrapped time). */
void replay(const uint64_t base, const std::vector<Step> &steps)
{
    SyncScheduler scheduler;
    Model model;
    for (size_t i{0}; i < steps.size(); ++i) {
        Sink sink;
        sink.accept = steps[i].accept;
        const bool was_due{scheduler.due(wrapped(base, steps[i].dt))};
        const SyncPoll got{scheduler.poll(wrapped(base, steps[i].dt), sink)};
        const Model::Result want{model.poll(static_cast<int64_t>(steps[i].dt), steps[i].accept)};
        zassert_equal(static_cast<int>(got.outcome), static_cast<int>(want.outcome), "outcome at step %zu", i);
        zassert_equal(got.late, want.late, "late at step %zu", i);
        zassert_equal(was_due, want.outcome != SyncOutcome::NotDue, "due() at step %zu", i);
        zassert_equal(sink.frames.size(), want.outcome == SyncOutcome::NotDue ? 0U : 1U, "sends at step %zu", i);
        zassert_equal(scheduler.next_due(), wrapped(base, static_cast<uint64_t>(model.due)), "next_due at step %zu", i);
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
    zassert_equal(scheduler.next_due(), 1234U + 50U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_poll_before_due_is_not_due)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    zassert_false(scheduler.due(1049));
    const SyncPoll result{scheduler.poll(1049, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 1U);
    zassert_equal(scheduler.next_due(), 1050U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_poll_at_due_tick_sends)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    zassert_true(scheduler.due(1050));
    const SyncPoll result{scheduler.poll(1050, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 2U);
    zassert_equal(scheduler.next_due(), 1100U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_late_by_period_minus_one_keeps_grid)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    const SyncPoll result{scheduler.poll(1099, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(result.late);
    zassert_equal(sink.frames.size(), 2U);
    zassert_equal(scheduler.next_due(), 1100U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_late_by_one_period_realigns)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    const SyncPoll result{scheduler.poll(1100, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_true(result.late);
    zassert_equal(sink.frames.size(), 2U);
    zassert_equal(scheduler.next_due(), 1150U);
    /* One SYNC only: no catch-up burst. */
    const SyncPoll again{scheduler.poll(1100, sink)};
    zassert_equal(static_cast<int>(again.outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_equal(sink.frames.size(), 2U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_rejected_send_is_skipped_not_retried)
{
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(1000, sink);
    sink.accept = false;
    const SyncPoll result{scheduler.poll(1050, sink)};
    zassert_equal(static_cast<int>(result.outcome), static_cast<int>(SyncOutcome::Skipped));
    zassert_false(result.late);
    zassert_equal(scheduler.next_due(), 1100U);
    const SyncPoll retry{scheduler.poll(1051, sink)};
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
    zassert_equal(scheduler.next_due(), 550U);
    zassert_false(scheduler.due(549));
}

ZTEST(lcs_sync_schedule, test_lcs_sync_wrap_cases_repeat_across_counter_overflow)
{
    /* First poll at 0xFFFFFFFF - 60 = 4294967235; due 4294967285, then 4294967335 = 39 after the wrap. */
    SyncScheduler scheduler;
    Sink sink;
    (void)scheduler.poll(4294967235U, sink);
    zassert_equal(scheduler.next_due(), 4294967285U);

    /* SY-2 across the wrap: before the due tick. */
    zassert_equal(static_cast<int>(scheduler.poll(4294967284U, sink).outcome), static_cast<int>(SyncOutcome::NotDue));
    /* SY-3: exactly the due tick; the next due is 4294967335 mod 2^32 = 39. */
    zassert_equal(static_cast<int>(scheduler.poll(4294967285U, sink).outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_equal(scheduler.next_due(), 39U);
    /* Polls after the wrap, still before 39. */
    zassert_equal(static_cast<int>(scheduler.poll(38U, sink).outcome), static_cast<int>(SyncOutcome::NotDue));
    zassert_false(scheduler.due(38U));
    /* SY-4: period - 1 late, grid kept. */
    const SyncPoll late_less{scheduler.poll(88U, sink)};
    zassert_equal(static_cast<int>(late_less.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(late_less.late);
    zassert_equal(scheduler.next_due(), 89U);
    /* SY-5: exactly one period late, realigned to now + period. */
    const SyncPoll late_one{scheduler.poll(139U, sink)};
    zassert_equal(static_cast<int>(late_one.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_true(late_one.late);
    zassert_equal(scheduler.next_due(), 189U);
    zassert_equal(sink.frames.size(), 4U);

    /* SY-5 where the late poll itself is the one that passes the wrap. */
    SyncScheduler second;
    (void)second.poll(4294967290U, sink);
    const SyncPoll across{second.poll(44U, sink)};
    zassert_equal(static_cast<int>(across.outcome), static_cast<int>(SyncOutcome::Sent));
    zassert_false(across.late);
    zassert_equal(second.next_due(), 94U);
    const SyncPoll late_across{second.poll(144U, sink)};
    zassert_true(late_across.late);
    zassert_equal(second.next_due(), 194U);
}

ZTEST(lcs_sync_schedule, test_lcs_sync_poll_tables_match_the_model)
{
    const std::vector<Step> steps{
        {0, true},   {0, true},   {49, true},  {50, true},  {51, true},  {99, true},   {100, false}, {100, true},
        {149, true}, {150, true}, {250, true}, {250, true}, {300, false}, {301, true},  {349, true},  {350, true},
        {449, true}, {450, true}, {500, true}, {549, true}, {599, true}, {600, false}, {1000, true}, {1049, true},
        {1050, true}, {1100, true}};
    const std::array<uint64_t, 7> bases{0ULL,           1ULL,           0x7FFFFFFFULL - 100U, 0x80000000ULL - 60U,
                                        0xFFFFFFFFULL - 120U, 0xFFFFFFFFULL - 49U, 0xFFFFFFFFULL};
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
            t += next() % 130U;
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

ZTEST(lcs_sync_schedule, test_lcs_sync_period_constant_is_5ms_in_50_ticks)
{
    zassert_equal(lexxhard::lcs::kSyncPeriodUs, 5000U);
    zassert_equal(lexxhard::lcs::sync_period_ticks(lexxhard::lcs::kSyncPeriodUs), kPeriod);
    zassert_equal(SyncScheduler::kPeriodTicks, kPeriod);
}
