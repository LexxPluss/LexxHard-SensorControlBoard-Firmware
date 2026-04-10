/*
 * Copyright (c) 2024, LexxPluss Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "pgood_debouncer.hpp"

using namespace lexxhard::board_controller;

// ---------------------------------------------------------------------------
// Test fixture helpers
//
// Use a minimal config with sampling_period_ms=1 and ng_count=3 so that
// each tick() call advances the prescaler to 1 (== period), triggering a
// sample immediately. This keeps tests simple while still exercising the
// prescaler path.
// ---------------------------------------------------------------------------
static constexpr PgoodConfig kTestConfig {
    .v24        = {.sampling_period_ms = 1, .ng_count = 3},
    .peripheral = {.sampling_period_ms = 1, .ng_count = 3},
    .mtr_l      = {.sampling_period_ms = 1, .ng_count = 3},
    .mtr_r      = {.sampling_period_ms = 1, .ng_count = 3},
};
using D  = PgoodDebouncerT<kTestConfig>;
using St = D::State;
using Ix = D::SignalIndex;

// Convenience: tick only the 24V signal NG/OK, all others OK, no maintenance.
static bool tick_24v(D& d, bool ng)
{
    return d.tick(ng, false, false, false, false);
}

// Advance prescaler to the sampling point and apply the given input.
// With sampling_period_ms=1 this is just one tick().
static bool sample_24v(D& d, bool ng) { return tick_24v(d, ng); }

// ---------------------------------------------------------------------------
// UT-001: 1x NG then OK -> no shutdown, ng_observed resets to 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT001_SingleNgThenOkNoShutdown)
{
    D d;

    EXPECT_FALSE(sample_24v(d, true));   // ng_observed -> 1, PENDING_NG
    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::PENDING_NG);
    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 1u);

    EXPECT_FALSE(sample_24v(d, false));  // ng_observed -> 0, OK
    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::OK);
    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 0u);

    EXPECT_FALSE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// UT-002: 2x consecutive NG then OK -> no shutdown, ng_observed resets to 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT002_TwoNgThenOkNoShutdown)
{
    D d;

    EXPECT_FALSE(sample_24v(d, true));   // ng_observed -> 1
    EXPECT_FALSE(sample_24v(d, true));   // ng_observed -> 2
    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::PENDING_NG);
    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 2u);

    EXPECT_FALSE(sample_24v(d, false));  // ng_observed -> 0, OK
    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::OK);
    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 0u);

    EXPECT_FALSE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// UT-003: 3x consecutive NG -> shutdown on third sample
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT003_ThreeConsecutiveNgTriggersShutdown)
{
    D d;

    EXPECT_FALSE(sample_24v(d, true));   // ng_observed -> 1, PENDING_NG
    EXPECT_FALSE(sample_24v(d, true));   // ng_observed -> 2, PENDING_NG
    EXPECT_TRUE(sample_24v(d, true));    // ng_observed -> 3, NG_CONFIRMED

    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 3u);
    EXPECT_TRUE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// UT-003b: NG_CONFIRMED is sticky - true is returned even after OK input
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT003b_NgConfirmedIsSticky)
{
    D d;
    sample_24v(d, true);
    sample_24v(d, true);
    sample_24v(d, true);  // NG_CONFIRMED

    EXPECT_TRUE(sample_24v(d, false));   // sticky -> still true
    EXPECT_EQ(d.get_signal(Ix::V24).state, St::NG_CONFIRMED);
}

// ---------------------------------------------------------------------------
// UT-003c: reset() clears NG_CONFIRMED back to OK
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT003c_ResetClearsNgConfirmed)
{
    D d;
    sample_24v(d, true);
    sample_24v(d, true);
    sample_24v(d, true);
    EXPECT_TRUE(d.is_ng_confirmed());

    d.reset();

    EXPECT_FALSE(d.is_ng_confirmed());
    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::OK);
    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 0u);
    EXPECT_EQ(d.get_signal(Ix::V24).prescaler,   0u);
    EXPECT_FALSE(sample_24v(d, false));
}

// ---------------------------------------------------------------------------
// UT-004: NG on MTR_L only -> only MTR_L counter increments
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT004_MtrLNgOnlyMtrLCounterIncrements)
{
    D d;

    bool shutdown = d.tick(false, false, true, false, false);
    EXPECT_FALSE(shutdown);

    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed,        0u);
    EXPECT_EQ(d.get_signal(Ix::PERIPHERAL).ng_observed, 0u);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).ng_observed,      1u);
    EXPECT_EQ(d.get_signal(Ix::MTR_R).ng_observed,      0u);

    EXPECT_EQ(d.get_signal(Ix::V24).state,        St::OK);
    EXPECT_EQ(d.get_signal(Ix::PERIPHERAL).state, St::OK);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state,      St::PENDING_NG);
    EXPECT_EQ(d.get_signal(Ix::MTR_R).state,      St::OK);
}

// ---------------------------------------------------------------------------
// UT-005: Simultaneous NG on all 4 signals -> all counters independent
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT005_AllSignalsNgSimultaneously)
{
    D d;

    EXPECT_FALSE(d.tick(true, true, true, true, false));  // all -> 1
    EXPECT_FALSE(d.tick(true, true, true, true, false));  // all -> 2
    EXPECT_TRUE(d.tick(true, true, true, true, false));   // all -> 3, shutdown

    EXPECT_EQ(d.get_signal(Ix::V24).state,        St::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Ix::PERIPHERAL).state, St::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state,      St::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Ix::MTR_R).state,      St::NG_CONFIRMED);
}

// ---------------------------------------------------------------------------
// Maintenance: MTR_L/R NG is masked (treated as OK)
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, MaintenanceMasksMtrSignals)
{
    D d;

    for (int i{0}; i < 3; ++i) {
        EXPECT_FALSE(d.tick(false, false, true, true, /*is_maintenance=*/true));
    }
    EXPECT_EQ(d.get_signal(Ix::MTR_L).ng_observed, 0u);
    EXPECT_EQ(d.get_signal(Ix::MTR_R).ng_observed, 0u);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state,       St::OK);
    EXPECT_EQ(d.get_signal(Ix::MTR_R).state,       St::OK);
    EXPECT_FALSE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// Initial state: all OK with counters at 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, InitialStateIsOk)
{
    D d;

    EXPECT_FALSE(d.is_ng_confirmed());
    for (auto idx : {Ix::V24, Ix::PERIPHERAL, Ix::MTR_L, Ix::MTR_R}) {
        EXPECT_EQ(d.get_signal(idx).state,       St::OK);
        EXPECT_EQ(d.get_signal(idx).ng_observed, 0u);
        EXPECT_EQ(d.get_signal(idx).prescaler,   0u);
    }
}

// ---------------------------------------------------------------------------
// Recovery: PENDING_NG -> OK -> re-accumulates from zero
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, RecoveryFromPendingNgThenReaccumulates)
{
    D d;

    sample_24v(d, true);   // ng_observed -> 1, PENDING_NG
    sample_24v(d, false);  // ng_observed -> 0, OK
    sample_24v(d, true);   // ng_observed -> 1, PENDING_NG
    sample_24v(d, true);   // ng_observed -> 2

    EXPECT_EQ(d.get_signal(Ix::V24).ng_observed, 2u);
    EXPECT_EQ(d.get_signal(Ix::V24).state,       St::PENDING_NG);
    EXPECT_FALSE(d.is_ng_confirmed());

    EXPECT_TRUE(sample_24v(d, true));   // ng_observed -> 3, NG_CONFIRMED
}

// ---------------------------------------------------------------------------
// ID-001: tick() remains true after NG_CONFIRMED (idempotency)
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, ID001_TickReturnsTrueUntilReset)
{
    D d;
    sample_24v(d, true);
    sample_24v(d, true);
    sample_24v(d, true);  // NG_CONFIRMED

    for (int i{0}; i < 5; ++i) {
        EXPECT_TRUE(tick_24v(d, false)) << "tick " << i << " after NG_CONFIRMED must return true";
    }

    d.reset();
    EXPECT_FALSE(tick_24v(d, false));
}

// ---------------------------------------------------------------------------
// PS-001: First sample occurs after sampling_period_ms ticks (not immediately)
// ---------------------------------------------------------------------------
TEST(PrescalerTest, PS001_FirstSampleAfterPeriod)
{
    // period=3, ng_count=2
    constexpr PgoodConfig cfg {
        .v24        = {.sampling_period_ms = 3, .ng_count = 2},
        .peripheral = {.sampling_period_ms = 3, .ng_count = 2},
        .mtr_l      = {.sampling_period_ms = 3, .ng_count = 2},
        .mtr_r      = {.sampling_period_ms = 3, .ng_count = 2},
    };
    PgoodDebouncerT<cfg> d;

    // tick 1, 2: prescaler < period, no sample
    EXPECT_FALSE(d.tick(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(PgoodDebouncerT<cfg>::SignalIndex::V24).ng_observed, 0u);

    EXPECT_FALSE(d.tick(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(PgoodDebouncerT<cfg>::SignalIndex::V24).ng_observed, 0u);

    // tick 3: prescaler reaches period -> first sample
    EXPECT_FALSE(d.tick(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(PgoodDebouncerT<cfg>::SignalIndex::V24).ng_observed, 1u);
    EXPECT_EQ(d.get_signal(PgoodDebouncerT<cfg>::SignalIndex::V24).state,
              PgoodDebouncerT<cfg>::State::PENDING_NG);
}

// ---------------------------------------------------------------------------
// PS-002: Ticks before period do not change state
// ---------------------------------------------------------------------------
TEST(PrescalerTest, PS002_TicksBeforePeriodNoTransition)
{
    constexpr PgoodConfig cfg {
        .v24        = {.sampling_period_ms = 5, .ng_count = 2},
        .peripheral = {.sampling_period_ms = 5, .ng_count = 2},
        .mtr_l      = {.sampling_period_ms = 5, .ng_count = 2},
        .mtr_r      = {.sampling_period_ms = 5, .ng_count = 2},
    };
    PgoodDebouncerT<cfg> d;
    using Ix2 = PgoodDebouncerT<cfg>::SignalIndex;
    using St2 = PgoodDebouncerT<cfg>::State;

    for (int i{0}; i < 4; ++i) {
        EXPECT_FALSE(d.tick(true, false, false, false, false));
        EXPECT_EQ(d.get_signal(Ix2::V24).state,       St2::OK);
        EXPECT_EQ(d.get_signal(Ix2::V24).ng_observed, 0u);
    }
}

// ---------------------------------------------------------------------------
// PS-003: >= comparison handles a skipped tick gracefully
// ---------------------------------------------------------------------------
TEST(PrescalerTest, PS003_GeComparisonHandlesSkippedTick)
{
    constexpr PgoodConfig cfg {
        .v24        = {.sampling_period_ms = 2, .ng_count = 2},
        .peripheral = {.sampling_period_ms = 2, .ng_count = 2},
        .mtr_l      = {.sampling_period_ms = 2, .ng_count = 2},
        .mtr_r      = {.sampling_period_ms = 2, .ng_count = 2},
    };
    // Manually advance prescaler past the period by calling tick() twice in
    // the same "real ms" — the second tick still samples correctly.
    PgoodDebouncerT<cfg> d;
    using Ix2 = PgoodDebouncerT<cfg>::SignalIndex;

    // tick 1: prescaler=1 < 2, no sample
    EXPECT_FALSE(d.tick(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(Ix2::V24).ng_observed, 0u);

    // tick 2: prescaler=2 >= 2, sample taken, prescaler resets to 0
    EXPECT_FALSE(d.tick(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(Ix2::V24).ng_observed, 1u);
}

// ---------------------------------------------------------------------------
// PS-004: Signals with different periods confirm independently
// ---------------------------------------------------------------------------
TEST(PrescalerTest, PS004_DifferentPeriodsConfirmIndependently)
{
    // V24: period=3, ng_count=2 -> confirms at tick 6  (3*2)
    // MTR_L: period=1, ng_count=2 -> confirms at tick 2  (1*2)
    constexpr PgoodConfig cfg {
        .v24        = {.sampling_period_ms = 3, .ng_count = 2},
        .peripheral = {.sampling_period_ms = 3, .ng_count = 2},
        .mtr_l      = {.sampling_period_ms = 1, .ng_count = 2},
        .mtr_r      = {.sampling_period_ms = 1, .ng_count = 2},
    };
    PgoodDebouncerT<cfg> d;
    using Ix2 = PgoodDebouncerT<cfg>::SignalIndex;
    using St2 = PgoodDebouncerT<cfg>::State;

    // tick 1: MTR_L ng_observed=1 PENDING; V24 prescaler=1 no sample
    EXPECT_FALSE(d.tick(true, false, true, false, false));
    EXPECT_EQ(d.get_signal(Ix2::MTR_L).ng_observed, 1u);
    EXPECT_EQ(d.get_signal(Ix2::V24).ng_observed,   0u);

    // tick 2: MTR_L ng_observed=2 NG_CONFIRMED -> shutdown; V24 still no sample
    EXPECT_TRUE(d.tick(true, false, true, false, false));
    EXPECT_EQ(d.get_signal(Ix2::MTR_L).state, St2::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Ix2::V24).state,   St2::OK);

    d.reset();

    // After reset, V24 confirms at tick 6
    for (int i{0}; i < 5; ++i) {
        EXPECT_FALSE(d.tick(true, false, false, false, false)) << "tick " << i + 1;
    }
    EXPECT_TRUE(d.tick(true, false, false, false, false));  // tick 6
    EXPECT_EQ(d.get_signal(Ix2::V24).state, St2::NG_CONFIRMED);
}

// ---------------------------------------------------------------------------
// MT-001: PENDING_NG in MTR_L when maintenance starts -> counter resets via mask
// ---------------------------------------------------------------------------
TEST(MaintenanceTest, MT001_PendingNgClearedOnMaintenanceEntry)
{
    D d;

    sample_24v(d, false);  // V24 OK
    // MTR_L: 2x NG without maintenance -> PENDING_NG
    d.tick(false, false, true, false, false);
    d.tick(false, false, true, false, false);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state,       St::PENDING_NG);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).ng_observed, 2u);

    // Maintenance starts: MTR_L input masked to OK -> ng_observed resets
    d.tick(false, false, true, false, /*is_maintenance=*/true);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state,       St::OK);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).ng_observed, 0u);
}

// ---------------------------------------------------------------------------
// MT-002: NG_CONFIRMED in MTR_L persists when maintenance starts
// ---------------------------------------------------------------------------
TEST(MaintenanceTest, MT002_NgConfirmedPersistsOnMaintenanceEntry)
{
    D d;

    // Confirm MTR_L NG
    d.tick(false, false, true, false, false);
    d.tick(false, false, true, false, false);
    d.tick(false, false, true, false, false);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state, St::NG_CONFIRMED);

    // Enter maintenance: NG_CONFIRMED stays (shutdown suppression is separate)
    d.tick(false, false, true, false, /*is_maintenance=*/true);
    EXPECT_EQ(d.get_signal(Ix::MTR_L).state, St::NG_CONFIRMED);
    EXPECT_TRUE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// AtomicFlagTest
//
// is_ng_confirmed() reads an std::atomic<bool> that tick() writes with
// memory_order_release/acquire. These tests use std::thread to run tick()
// (simulating the ISR writer) and is_ng_confirmed() (simulating a reader in
// another interrupt context) concurrently, verifying that:
//   - No data race occurs (detectable with ThreadSanitizer).
//   - The reader eventually observes NG_CONFIRMED once it is established.
//   - The reader never observes true before confirmation is reached.
// ---------------------------------------------------------------------------

// AT-001: Concurrent writer drives signals to NG_CONFIRMED while a reader
//         polls is_ng_confirmed(). After the writer completes all ticks and
//         signals done, the reader must have observed true at least once.
//
//         The release/acquire ordering between writer_done.store(release) and
//         writer_done.load(acquire) in the reader guarantees that the reader's
//         final check sees the NG_CONFIRMED state established by tick().
//         TSan verifies there is no data race on ng_confirmed_ during concurrent
//         access.
TEST(AtomicFlagTest, AT001_ConcurrentReaderSeesConfirmationEventually)
{
    D d;
    std::atomic<bool> writer_done{false};
    std::atomic<bool> reader_saw_true{false};

    std::thread reader([&]() {
        while (!writer_done.load(std::memory_order_acquire)) {
            if (d.is_ng_confirmed()) {
                reader_saw_true.store(true, std::memory_order_relaxed);
            }
        }
        // Final check: writer_done(release) -> load(acquire) guarantees
        // ng_confirmed_ written by tick() is visible here.
        if (d.is_ng_confirmed()) {
            reader_saw_true.store(true, std::memory_order_relaxed);
        }
    });

    // Writer: advance to NG_CONFIRMED (ng_count=3 ticks with period=1)
    for (int i{0}; i < 3; ++i) {
        d.tick(true, false, false, false, false);
    }
    writer_done.store(true, std::memory_order_release);
    reader.join();

    EXPECT_TRUE(d.is_ng_confirmed());
    EXPECT_TRUE(reader_saw_true.load(std::memory_order_relaxed));
}

// AT-003: Concurrent reader never observes is_ng_confirmed()=true while
//         signals are still accumulating (PENDING_NG, not yet confirmed).
//         Writer stops before confirmation; reader must see false throughout.
TEST(AtomicFlagTest, AT003_ConcurrentReaderNeverSesTrueBeforeConfirmation)
{
    D d;

    std::atomic<bool> writer_done{false};
    std::atomic<bool> reader_saw_true{false};

    std::thread reader([&]() {
        while (!writer_done.load(std::memory_order_acquire)) {
            if (d.is_ng_confirmed()) {
                reader_saw_true.store(true, std::memory_order_relaxed);
            }
        }
        // One final check after writer finishes
        if (d.is_ng_confirmed()) {
            reader_saw_true.store(true, std::memory_order_relaxed);
        }
    });

    // Writer: only 2 ticks (ng_count=3), never reaches NG_CONFIRMED
    d.tick(true, false, false, false, false);
    d.tick(true, false, false, false, false);
    writer_done.store(true, std::memory_order_release);

    reader.join();

    EXPECT_EQ(d.get_signal(Ix::V24).state, St::PENDING_NG);
    EXPECT_FALSE(reader_saw_true.load(std::memory_order_relaxed));
}

// AT-004: reset() clears the atomic flag such that a concurrently running
//         reader immediately sees false after reset() completes.
TEST(AtomicFlagTest, AT004_ConcurrentReaderSeesFalseAfterReset)
{
    D d;

    // Establish NG_CONFIRMED first
    d.tick(true, false, false, false, false);
    d.tick(true, false, false, false, false);
    d.tick(true, false, false, false, false);
    ASSERT_TRUE(d.is_ng_confirmed());

    std::atomic<bool> reset_done{false};
    std::atomic<bool> reader_saw_true_after_reset{false};

    std::thread reader([&]() {
        // Wait for reset to complete, then verify flag is false.
        while (!reset_done.load(std::memory_order_acquire)) {}
        if (d.is_ng_confirmed()) {
            reader_saw_true_after_reset.store(true, std::memory_order_relaxed);
        }
    });

    d.reset();
    reset_done.store(true, std::memory_order_release);

    reader.join();

    EXPECT_FALSE(reader_saw_true_after_reset.load(std::memory_order_relaxed));
    EXPECT_FALSE(d.is_ng_confirmed());
}

// AT-005: Concurrent reader sees true as soon as any one signal is NG_CONFIRMED,
//         even while others are still PENDING_NG.
TEST(AtomicFlagTest, AT005_ConcurrentReaderSeesTrueOnPartialConfirmation)
{
    // MTR_L: ng_count=2 (confirms at tick 2); V24: ng_count=5
    constexpr PgoodConfig kCfg {
        .v24        = {.sampling_period_ms = 1, .ng_count = 5},
        .peripheral = {.sampling_period_ms = 1, .ng_count = 5},
        .mtr_l      = {.sampling_period_ms = 1, .ng_count = 2},
        .mtr_r      = {.sampling_period_ms = 1, .ng_count = 5},
    };
    PgoodDebouncerT<kCfg> d;
    using Ix2 = PgoodDebouncerT<kCfg>::SignalIndex;
    using St2 = PgoodDebouncerT<kCfg>::State;

    std::atomic<bool> writer_done{false};
    std::atomic<bool> reader_observed_true{false};

    std::thread reader([&]() {
        while (!writer_done.load(std::memory_order_acquire)) {}
        if (d.is_ng_confirmed()) {
            reader_observed_true.store(true, std::memory_order_relaxed);
        }
    });

    // tick 1 and 2: MTR_L reaches NG_CONFIRMED; V24 is still PENDING_NG
    d.tick(true, false, true, false, false);
    d.tick(true, false, true, false, false);
    writer_done.store(true, std::memory_order_release);

    reader.join();

    EXPECT_EQ(d.get_signal(Ix2::MTR_L).state, St2::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Ix2::V24).state,   St2::PENDING_NG);
    EXPECT_TRUE(reader_observed_true.load(std::memory_order_relaxed));
}
