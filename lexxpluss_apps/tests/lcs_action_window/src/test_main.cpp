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

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <random>
#include <vector>

#include <zephyr/ztest.h>

#include "lcs_action_window.hpp"
#include "lcs_frame.hpp"
#include "lcs_request.hpp"
#include "lcs_sync_schedule.hpp"
#include "lcs_timing.hpp"

using lexxhard::lcs::ActionOutcome;
using lexxhard::lcs::ActionWindow;
using lexxhard::lcs::DownlinkFrame;
using lexxhard::lcs::Request;
using lexxhard::lcs::SyncOutcome;
using lexxhard::lcs::SyncPoll;
using lexxhard::lcs::SyncScheduler;
using lexxhard::lcs::kActionWindowStartTicks;

ZTEST_SUITE(lcs_action_window, NULL, NULL, NULL, NULL, NULL);

namespace {

constexpr uint32_t kPeriod{100};
constexpr uint32_t kNearWrap{0xFFFFFFF0U};

constexpr uint8_t kCmdA{lexxhard::lcs::kCmdAbort};

/* A fake request queue and a fake transmitter around one ActionWindow. */
struct Harness {
    ActionWindow window;
    std::deque<Request> queue;
    std::vector<DownlinkFrame> frames; /* accepted frames only */
    uint32_t send_calls{0};
    uint32_t pop_calls{0};
    bool accept{true};

    ActionOutcome poll(const uint32_t tick)
    {
        return window.poll(
            tick,
            [this]() -> std::optional<Request> {
                if (queue.empty()) {
                    return std::nullopt;
                }
                return queue.front();
            },
            [this]() {
                ++pop_calls;
                queue.pop_front();
            },
            [this](const DownlinkFrame &frame) {
                ++send_calls;
                if (accept) {
                    frames.push_back(frame);
                }
                return accept;
            });
    }

    void sync_sent(const uint32_t tick, const bool late = false)
    {
        window.on_sync(SyncPoll{SyncOutcome::Sent, late}, tick);
    }

    void sync_skipped(const uint32_t tick)
    {
        window.on_sync(SyncPoll{SyncOutcome::Skipped, false}, tick);
    }

    void push(const uint8_t target, const uint16_t param)
    {
        queue.push_back(Request{kCmdA, target, param});
    }
};

int outcome_int(const ActionOutcome outcome)
{
    return static_cast<int>(outcome);
}

void expect_outcome(const ActionOutcome got, const ActionOutcome want, const char *what)
{
    zassert_equal(outcome_int(got), outcome_int(want), "%s: outcome %d, expected %d", what, outcome_int(got),
                  outcome_int(want));
}

}

ZTEST(lcs_action_window, test_sc05_01_basic_send_matches_encoding_and_pops)
{
    /* Catches: a wrong frame, or a sent request that stays in the queue. */
    Harness h;
    h.sync_sent(1000);
    h.push(2, 0x1234);
    expect_outcome(h.poll(1000 + lexxhard::lcs::kActionWindowStartTicks), ActionOutcome::Sent, "send");
    zassert_equal(h.frames.size(), 1U);
    zassert_equal(h.frames[0], lexxhard::lcs::encode_downlink(2, kCmdA, 0x1234));
    zassert_equal(h.pop_calls, 1U);
    zassert_true(h.queue.empty());
}

ZTEST(lcs_action_window, test_sc05_02_window_start_boundary_and_wrap)
{
    /* Catches: a start one tick early or late, and a non-wrap-safe elapsed comparison. */
    for (const uint32_t origin : {1000U, kNearWrap}) {
        for (const uint32_t dt : {kActionWindowStartTicks - 1, kActionWindowStartTicks, kActionWindowStartTicks + 1}) {
            Harness h;
            h.sync_sent(origin);
            h.push(0, 1);
            const ActionOutcome got = h.poll(origin + dt);
            expect_outcome(got, dt >= kActionWindowStartTicks ? ActionOutcome::Sent : ActionOutcome::NotSent, "start boundary");
            zassert_equal(h.frames.size(), dt >= kActionWindowStartTicks ? 1U : 0U);
            zassert_equal(h.send_calls, dt >= kActionWindowStartTicks ? 1U : 0U);
        }
        Harness h;
        h.sync_sent(origin);
        h.push(0, 1);
        expect_outcome(h.poll(origin + kActionWindowStartTicks - 1), ActionOutcome::NotSent, "before start");
        expect_outcome(h.poll(origin + kActionWindowStartTicks), ActionOutcome::Sent, "at start");
    }
}

ZTEST(lcs_action_window, test_sc05_03_latest_start_boundary_and_wrap)
{
    /* Catches: a latest start one tick early or late, and a non-wrap-safe elapsed comparison. */
    for (const uint32_t origin : {1000U, kNearWrap}) {
        for (const uint32_t dt : {94U, 95U, 96U}) {
            Harness h;
            h.sync_sent(origin);
            h.push(0, 1);
            const bool inside{dt <= 95U};
            expect_outcome(h.poll(origin + dt), inside ? ActionOutcome::Sent : ActionOutcome::NotSent, "latest");
            zassert_equal(h.frames.size(), inside ? 1U : 0U);
            zassert_equal(h.queue.size(), inside ? 0U : 1U);
        }
        Harness h;
        h.sync_sent(origin);
        h.push(0, 1);
        expect_outcome(h.poll(origin + 96U), ActionOutcome::NotSent, "after the window");
        h.sync_sent(origin + kPeriod);
        expect_outcome(h.poll(origin + kPeriod + kActionWindowStartTicks), ActionOutcome::Sent, "next window");
    }
}

ZTEST(lcs_action_window, test_sc05_04_two_requests_one_per_period)
{
    /* Catches: more than one ACTION in a period, or a second request lost. */
    Harness h;
    h.sync_sent(0);
    h.push(0, 1);
    h.push(0, 2);
    expect_outcome(h.poll(kActionWindowStartTicks), ActionOutcome::Sent, "first");
    for (uint32_t dt{kActionWindowStartTicks + 1}; dt < kPeriod; ++dt) {
        expect_outcome(h.poll(dt), ActionOutcome::NotSent, "second waits");
    }
    zassert_equal(h.queue.size(), 1U);
    h.sync_sent(kPeriod);
    expect_outcome(h.poll(kPeriod + kActionWindowStartTicks), ActionOutcome::Sent, "second in the next period");
    zassert_equal(h.frames.size(), 2U);
    zassert_equal(h.frames[0], lexxhard::lcs::encode_downlink(0, kCmdA, 1));
    zassert_equal(h.frames[1], lexxhard::lcs::encode_downlink(0, kCmdA, 2));
    zassert_true(h.queue.empty());
}

ZTEST(lcs_action_window, test_sc05_05_broadcast_target_gives_one_frame)
{
    /* Catches: a broadcast request expanded to one frame per board. */
    Harness h;
    h.sync_sent(0);
    h.push(lexxhard::lcs::kBroadcastId, 7);
    expect_outcome(h.poll(kActionWindowStartTicks), ActionOutcome::Sent, "broadcast");
    for (uint32_t dt{kActionWindowStartTicks + 1}; dt < kPeriod; ++dt) {
        expect_outcome(h.poll(dt), ActionOutcome::NotSent, "no more frames");
    }
    zassert_equal(h.frames.size(), 1U);
    zassert_equal(h.frames[0], lexxhard::lcs::encode_downlink(lexxhard::lcs::kBroadcastId, kCmdA, 7));
}

ZTEST(lcs_action_window, test_sc05_06_no_action_before_first_sync)
{
    /* Catches: an ACTION sent with an undefined phase. */
    Harness h;
    h.push(0, 1);
    for (uint32_t tick{0}; tick < 300; ++tick) {
        expect_outcome(h.poll(tick), ActionOutcome::NotSent, "before the first SYNC");
    }
    zassert_equal(h.send_calls, 0U);
    h.sync_sent(500);
    expect_outcome(h.poll(500 + kActionWindowStartTicks - 1), ActionOutcome::NotSent, "before the window");
    expect_outcome(h.poll(500 + kActionWindowStartTicks), ActionOutcome::Sent, "first window");
    zassert_equal(h.frames.size(), 1U);
}

ZTEST(lcs_action_window, test_sc05_07_send_depends_on_sync_phase_only)
{
    /* Catches: a window that waits for a child uplink. */
    Harness h;
    h.sync_sent(40000);
    h.push(3, 9);
    expect_outcome(h.poll(40000 + 50U), ActionOutcome::Sent, "no other input");
    zassert_equal(h.frames.size(), 1U);
}

ZTEST(lcs_action_window, test_sc05_08_sync_has_priority_over_pending_action)
{
    /* Catches: an ACTION that delays or replaces a due SYNC. */
    SyncScheduler scheduler;
    Harness h;
    struct Sent {
        uint32_t tick;
        uint8_t cmd;
    };
    std::vector<Sent> log;
    uint32_t now{0};
    for (uint32_t tick{0}; tick <= kPeriod + kActionWindowStartTicks + 12; ++tick) {
        now = tick;
        if (tick == 96) {
            h.push(1, 5); /* phase 96: beyond the latest start */
        }
        const SyncPoll poll = scheduler.poll(tick, [&](const DownlinkFrame &frame) {
            log.push_back(Sent{now, frame[3]});
            return true;
        });
        if (poll.outcome != SyncOutcome::NotDue) {
            h.window.on_sync(poll, tick);
        }
        h.window.poll(
            tick,
            [&]() -> std::optional<Request> {
                if (h.queue.empty()) {
                    return std::nullopt;
                }
                return h.queue.front();
            },
            [&]() { h.queue.pop_front(); },
            [&](const DownlinkFrame &frame) {
                log.push_back(Sent{now, frame[3]});
                return true;
            });
    }
    zassert_equal(log.size(), 3U);
    zassert_equal(log[0].tick, 0U);
    zassert_equal(log[0].cmd, lexxhard::lcs::kCmdLinkSync);
    zassert_equal(log[1].tick, kPeriod);
    zassert_equal(log[1].cmd, lexxhard::lcs::kCmdLinkSync);
    zassert_equal(log[2].tick, kPeriod + kActionWindowStartTicks);
    zassert_equal(log[2].cmd, kCmdA);
}

ZTEST(lcs_action_window, test_sc05_09_refused_keeps_request_and_does_not_retry)
{
    /* Catches: a retry in the same window, a request popped on refusal, or a request lost. */
    Harness h;
    h.accept = false;
    h.sync_sent(0);
    h.push(0, 1);
    expect_outcome(h.poll(kActionWindowStartTicks), ActionOutcome::Refused, "refused");
    zassert_equal(h.queue.size(), 1U);
    zassert_equal(h.pop_calls, 0U);
    for (uint32_t dt{kActionWindowStartTicks + 1}; dt < kPeriod; ++dt) {
        expect_outcome(h.poll(dt), ActionOutcome::NotSent, "no second attempt");
    }
    zassert_equal(h.send_calls, 1U);
    h.accept = true;
    h.sync_sent(kPeriod);
    expect_outcome(h.poll(kPeriod + kActionWindowStartTicks), ActionOutcome::Sent, "next window");
    zassert_equal(h.send_calls, 2U);
    zassert_equal(h.frames.size(), 1U);
    zassert_true(h.queue.empty());
}

ZTEST(lcs_action_window, test_sc05_10_skipped_sync_makes_phase_undefined)
{
    /* Catches: a window kept open relative to the old SYNC after a skipped SYNC. */
    Harness h;
    h.sync_sent(0);
    h.push(0, 1);
    /* The skip is reported inside the old window, where only an undefined phase stops the send. */
    const uint32_t skip_tick{kActionWindowStartTicks + 12};
    h.sync_skipped(skip_tick);
    for (uint32_t tick{skip_tick}; tick < 2 * kPeriod; ++tick) {
        expect_outcome(h.poll(tick), ActionOutcome::NotSent, "skipped period");
    }
    zassert_equal(h.send_calls, 0U);
    zassert_equal(h.queue.size(), 1U);
    h.sync_sent(2 * kPeriod);
    expect_outcome(h.poll(2 * kPeriod + kActionWindowStartTicks), ActionOutcome::Sent, "after the next Sent");
    zassert_equal(h.frames.size(), 1U);
}

ZTEST(lcs_action_window, test_sc05_10b_late_sync_resets_phase)
{
    /* Catches: a late SYNC treated like a skipped one, or the phase kept from the old SYNC. */
    Harness h;
    h.sync_sent(4900);
    h.sync_sent(5000, true);
    h.push(0, 1);
    expect_outcome(h.poll(5000 + kActionWindowStartTicks - 1), ActionOutcome::NotSent, "before the window");
    expect_outcome(h.poll(5000 + kActionWindowStartTicks), ActionOutcome::Sent, "relative to the late SYNC");
    zassert_equal(h.frames.size(), 1U);
}

ZTEST(lcs_action_window, test_sc05_11_pin_window_constants)
{
    /* Catches: a changed timing constant that silently breaks the window premise. */
    using namespace lexxhard::lcs;
    static_assert(kScbSyncLeadUs == 60);
    static_assert(kScbSyncArrivalUs == 220);
    static_assert(kScbSyncArrivalUs == kScbSyncLeadUs + 160);
    static_assert(kLcsId1EndFromArrivalUs == 2546);
    static_assert(kActionPostId1MarginUs == 200);
    static_assert(kActionWindowStartUs == 2966);
    static_assert(kActionWindowStartUs == kScbSyncArrivalUs + kLcsId1EndFromArrivalUs + kActionPostId1MarginUs);
    static_assert(kActionLengthUs == 171);
    static_assert(kActionGuardUs == 200);
    static_assert(kSyncPeriodUs - kActionLengthUs - kActionGuardUs == 9629);
    static_assert(kActionWindowStartTicks == 31);
    static_assert(kActionWindowLatestTicks == 95);
    static_assert((kActionWindowLatestTicks + 1) * kTickUs + kActionLengthUs + kActionGuardUs <= kSyncPeriodUs);
    static_assert((kActionWindowStartTicks - 1) * kTickUs >= kActionWindowStartUs);
    zassert_equal(kActionWindowStartTicks, 31U);
    zassert_equal(kActionWindowLatestTicks, 95U);
    zassert_true(kActionWindowStartTicks < kActionWindowLatestTicks);
}

ZTEST(lcs_action_window, test_sc05_12_property_one_frame_per_request)
{
    /* Catches: a request sent twice, never sent, or lost across periods. */
    std::mt19937 rng{12345U};
    Harness h;
    uint16_t next_param{0};
    constexpr uint32_t kBase{0xFFFFFF00U};
    uint32_t period{0};
    for (; period < 60; ++period) {
        const uint32_t origin{kBase + period * kPeriod};
        h.sync_sent(origin);
        const uint32_t arrivals{rng() % 3U};
        for (uint32_t i{0}; i < arrivals; ++i) {
            h.push(static_cast<uint8_t>(rng() % 2U), next_param++);
        }
        for (uint32_t dt{0}; dt < kPeriod; ++dt) {
            h.poll(origin + dt);
        }
    }
    while (!h.queue.empty() && period < 1000) {
        const uint32_t origin{kBase + period * kPeriod};
        h.sync_sent(origin);
        for (uint32_t dt{0}; dt < kPeriod; ++dt) {
            h.poll(origin + dt);
        }
        ++period;
    }
    zassert_true(h.queue.empty());
    zassert_equal(h.frames.size(), static_cast<size_t>(next_param));
    zassert_equal(h.pop_calls, static_cast<uint32_t>(next_param));
    zassert_equal(h.send_calls, static_cast<uint32_t>(next_param));
    for (size_t i{0}; i < h.frames.size(); ++i) {
        zassert_equal(h.frames[i][4], static_cast<uint8_t>(i >> 8), "param high at %zu", i);
        zassert_equal(h.frames[i][5], static_cast<uint8_t>(i & 0xFFU), "param low at %zu", i);
    }
}

ZTEST(lcs_action_window, test_sc05_13_seeded_schedule)
{
    /* Catches: an attempt outside the window, two attempts in a period, or a lost or doubled request. */
    for (const uint32_t seed : {1U, 2U, 3U, 4U}) {
        std::mt19937 rng{seed};
        Harness h;
        uint32_t pushed{0};
        uint32_t period_attempt_calls{0};
        uint32_t current_dt{0};
        bool phase_defined{false};
        constexpr uint32_t kBase{0xFFFFFE00U};

        for (uint32_t period{0}; period < 200; ++period) {
            const uint32_t origin{kBase + period * kPeriod};
            const bool skipped{rng() % 10U == 0U};
            const bool late{!skipped && rng() % 10U == 0U};
            h.accept = rng() % 7U != 0U;
            if (skipped) {
                h.sync_skipped(origin);
                phase_defined = false;
            } else {
                h.sync_sent(origin, late);
                phase_defined = true;
            }
            const bool arrives{rng() % 2U == 0U};
            const uint32_t arrival_dt{rng() % kPeriod};
            const uint32_t calls_before{h.send_calls};
            for (current_dt = 0; current_dt < kPeriod; ++current_dt) {
                if (arrives && current_dt == arrival_dt) {
                    h.push(0, static_cast<uint16_t>(pushed++));
                }
                const uint32_t before{h.send_calls};
                h.poll(origin + current_dt);
                if (h.send_calls != before) {
                    zassert_true(phase_defined, "attempt with an undefined phase");
                    zassert_true(current_dt >= kActionWindowStartTicks && current_dt <= 95U, "attempt at dt %u", current_dt);
                }
            }
            period_attempt_calls = h.send_calls - calls_before;
            zassert_true(period_attempt_calls <= 1U, "attempts in a period: %u", period_attempt_calls);
        }
        /* Drain with clean periods. */
        h.accept = true;
        for (uint32_t period{200}; period < 1200 && !h.queue.empty(); ++period) {
            const uint32_t origin{kBase + period * kPeriod};
            h.sync_sent(origin);
            for (uint32_t dt{0}; dt < kPeriod; ++dt) {
                h.poll(origin + dt);
            }
        }
        zassert_true(h.queue.empty());
        zassert_equal(h.frames.size(), static_cast<size_t>(pushed), "seed %u", seed);
        zassert_equal(h.pop_calls, pushed, "seed %u", seed);
    }
}

ZTEST(lcs_action_window, test_sc05_14_accepted_request_is_never_sent_again)
{
    /* Catches: a consumed request that is sent again in the same or a later period. */
    Harness h;
    h.sync_sent(0);
    h.push(0, 1);
    expect_outcome(h.poll(kActionWindowStartTicks), ActionOutcome::Sent, "send");
    for (uint32_t period{0}; period < 4; ++period) {
        if (period > 0) {
            h.sync_sent(period * kPeriod);
        }
        for (uint32_t dt{period == 0 ? kActionWindowStartTicks + 1 : 0U}; dt < kPeriod; ++dt) {
            expect_outcome(h.poll(period * kPeriod + dt), ActionOutcome::NotSent, "consumed");
        }
    }
    zassert_equal(h.frames.size(), 1U);
    zassert_equal(h.send_calls, 1U);
    zassert_equal(h.pop_calls, 1U);
}

ZTEST(lcs_action_window, test_sc05_15_arrival_after_empty_poll_is_sent_in_the_same_window)
{
    /* Catches: an empty poll that uses up the period's attempt, so a request arriving later in the window
     * waits a whole period. */
    Harness h;
    h.sync_sent(1000);
    expect_outcome(h.poll(1000 + lexxhard::lcs::kActionWindowStartTicks), ActionOutcome::NotSent, "first empty poll");
    expect_outcome(h.poll(1000 + lexxhard::lcs::kActionWindowStartTicks + 12U), ActionOutcome::NotSent,
                   "second empty poll");
    zassert_equal(h.send_calls, 0U);
    zassert_equal(h.pop_calls, 0U);
    h.push(1, 0x0042);
    expect_outcome(h.poll(1000 + 50U), ActionOutcome::Sent, "arrival in the same window");
    zassert_equal(h.frames.size(), 1U);
    zassert_equal(h.frames[0], lexxhard::lcs::encode_downlink(1, kCmdA, 0x0042));
    zassert_equal(h.pop_calls, 1U);
    zassert_true(h.queue.empty());
}
