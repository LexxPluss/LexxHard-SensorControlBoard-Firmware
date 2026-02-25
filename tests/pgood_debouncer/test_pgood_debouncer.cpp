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
#include "pgood_debouncer.hpp"

using namespace lexxhard::board_controller;
using State = PgoodDebouncer::State;
using Idx   = PgoodDebouncer::SignalIndex;

// Convenience: update only the 24V signal, all others OK, not in maintenance.
static bool update_24v(PgoodDebouncer& d, bool ng)
{
    return d.update(ng, false, false, false, false);
}

// ---------------------------------------------------------------------------
// UT-001: 1x NG + OK -> no shutdown, counter resets to 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT001_SingleNgThenOkNoShutdown)
{
    PgoodDebouncer d;

    EXPECT_FALSE(update_24v(d, true));  // ng_count -> 1, PENDING_NG
    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::PENDING_NG);
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 1u);

    EXPECT_FALSE(update_24v(d, false));  // ng_count -> 0, OK
    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::OK);
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 0u);

    EXPECT_FALSE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// UT-002: 2x consecutive NG, then OK -> no shutdown, counter resets to 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT002_TwoNgThenOkNoShutdown)
{
    PgoodDebouncer d;

    EXPECT_FALSE(update_24v(d, true));  // ng_count -> 1
    EXPECT_FALSE(update_24v(d, true));  // ng_count -> 2
    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::PENDING_NG);
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 2u);

    EXPECT_FALSE(update_24v(d, false));  // ng_count -> 0, OK
    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::OK);
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 0u);

    EXPECT_FALSE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// UT-003: 3x consecutive NG -> shutdown triggered on the third sample
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT003_ThreeConsecutiveNgTriggersShutdown)
{
    PgoodDebouncer d;

    EXPECT_FALSE(update_24v(d, true));  // ng_count -> 1, PENDING_NG
    EXPECT_FALSE(update_24v(d, true));  // ng_count -> 2, PENDING_NG
    EXPECT_TRUE(update_24v(d, true));   // ng_count -> 3, NG_CONFIRMED -> shutdown

    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 3u);
    EXPECT_TRUE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// UT-003b: NG_CONFIRMED is sticky - further calls still return true
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT003b_NgConfirmedIsSticky)
{
    PgoodDebouncer d;
    update_24v(d, true);
    update_24v(d, true);
    update_24v(d, true);  // confirmed

    // Even if GPIO returns OK on subsequent polls, stays confirmed
    EXPECT_TRUE(update_24v(d, false));
    EXPECT_EQ(d.get_signal(Idx::V24).state, State::NG_CONFIRMED);
}

// ---------------------------------------------------------------------------
// UT-003c: reset() clears NG_CONFIRMED back to OK
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT003c_ResetClearsNgConfirmed)
{
    PgoodDebouncer d;
    update_24v(d, true);
    update_24v(d, true);
    update_24v(d, true);  // confirmed
    EXPECT_TRUE(d.is_ng_confirmed());

    d.reset();

    EXPECT_FALSE(d.is_ng_confirmed());
    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::OK);
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 0u);
    EXPECT_FALSE(update_24v(d, false));  // normal operation resumes
}

// ---------------------------------------------------------------------------
// UT-004: NG on PG_MTR_L only -> only MTR_L counter increments, others stay 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT004_MtrLNgOnlyMtrLCounterIncrements)
{
    PgoodDebouncer d;

    // One sample: MTR_L NG, rest OK, not in maintenance
    bool shutdown = d.update(false, false, true, false, false);
    EXPECT_FALSE(shutdown);

    EXPECT_EQ(d.get_signal(Idx::V24).ng_count,        0u);
    EXPECT_EQ(d.get_signal(Idx::PERIPHERAL).ng_count, 0u);
    EXPECT_EQ(d.get_signal(Idx::MTR_L).ng_count,      1u);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).ng_count,      0u);

    EXPECT_EQ(d.get_signal(Idx::V24).state,        State::OK);
    EXPECT_EQ(d.get_signal(Idx::PERIPHERAL).state, State::OK);
    EXPECT_EQ(d.get_signal(Idx::MTR_L).state,      State::PENDING_NG);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).state,      State::OK);
}

// ---------------------------------------------------------------------------
// UT-005: Simultaneous NG on all 4 signals -> all counters increment
//         independently, shutdown fires at the 3rd sample
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, UT005_AllSignalsNgSimultaneously)
{
    PgoodDebouncer d;

    EXPECT_FALSE(d.update(true, true, true, true, false));  // all ng_count -> 1
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count,        1u);
    EXPECT_EQ(d.get_signal(Idx::PERIPHERAL).ng_count, 1u);
    EXPECT_EQ(d.get_signal(Idx::MTR_L).ng_count,      1u);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).ng_count,      1u);

    EXPECT_FALSE(d.update(true, true, true, true, false));  // all ng_count -> 2
    EXPECT_EQ(d.get_signal(Idx::V24).ng_count,        2u);
    EXPECT_EQ(d.get_signal(Idx::PERIPHERAL).ng_count, 2u);
    EXPECT_EQ(d.get_signal(Idx::MTR_L).ng_count,      2u);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).ng_count,      2u);

    EXPECT_TRUE(d.update(true, true, true, true, false));   // all ng_count -> 3, shutdown

    EXPECT_EQ(d.get_signal(Idx::V24).state,        State::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Idx::PERIPHERAL).state, State::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Idx::MTR_L).state,      State::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).state,      State::NG_CONFIRMED);
}

// ---------------------------------------------------------------------------
// Maintenance mode: MTR_L/MTR_R NG is masked (treated as OK)
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, MaintenanceMasksMtrSignals)
{
    PgoodDebouncer d;

    // 3x MTR_L and MTR_R NG while in maintenance -> NOT confirmed, no shutdown
    for (int i{0}; i < 3; ++i) {
        EXPECT_FALSE(d.update(false, false, true, true, /*is_maintenance=*/true));
    }
    EXPECT_EQ(d.get_signal(Idx::MTR_L).ng_count, 0u);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).ng_count, 0u);
    EXPECT_EQ(d.get_signal(Idx::MTR_L).state,    State::OK);
    EXPECT_EQ(d.get_signal(Idx::MTR_R).state,    State::OK);
    EXPECT_FALSE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// ShutdownInMaintenance = false: confirmed NG during maintenance is suppressed
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerNoShutdownTest, ShutdownSuppressedInMaintenanceMode)
{
    PgoodDebouncerT<false> d;

    // 3x 24V NG in maintenance -> NG confirmed internally but shutdown suppressed
    for (int i{0}; i < 3; ++i) {
        bool result = d.update(true, false, false, false, /*is_maintenance=*/true);
        EXPECT_FALSE(result) << "Shutdown must be suppressed in maintenance mode "
                                "when ShutdownInMaintenance=false (sample " << i + 1 << ")";
    }
    EXPECT_TRUE(d.is_ng_confirmed());  // state machine still ran to completion
}

TEST(PgoodDebouncerNoShutdownTest, ShutdownOccursOutsideMaintenanceMode)
{
    PgoodDebouncerT<false> d;

    // Same 3x NG but NOT in maintenance -> shutdown fires normally
    EXPECT_FALSE(d.update(true, false, false, false, false));
    EXPECT_FALSE(d.update(true, false, false, false, false));
    EXPECT_TRUE(d.update(true, false, false, false, false));
    EXPECT_TRUE(d.is_ng_confirmed());
}

// ---------------------------------------------------------------------------
// Boundary: initial state is OK with all counters at 0
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, InitialStateIsOk)
{
    PgoodDebouncer d;

    EXPECT_FALSE(d.is_ng_confirmed());
    for (auto idx : {Idx::V24, Idx::PERIPHERAL, Idx::MTR_L, Idx::MTR_R}) {
        EXPECT_EQ(d.get_signal(idx).state,    State::OK);
        EXPECT_EQ(d.get_signal(idx).ng_count, 0u);
    }
}

// ---------------------------------------------------------------------------
// Recovery from PENDING_NG re-accumulates correctly after reset by OK sample
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, RecoveryFromPendingNgThenReaccumulates)
{
    PgoodDebouncer d;

    update_24v(d, true);   // ng_count -> 1, PENDING_NG
    update_24v(d, false);  // ng_count -> 0, OK (recovered)
    update_24v(d, true);   // ng_count -> 1, PENDING_NG again
    update_24v(d, true);   // ng_count -> 2

    EXPECT_EQ(d.get_signal(Idx::V24).ng_count, 2u);
    EXPECT_EQ(d.get_signal(Idx::V24).state,    State::PENDING_NG);
    EXPECT_FALSE(d.is_ng_confirmed());

    EXPECT_TRUE(update_24v(d, true));  // ng_count -> 3, NG_CONFIRMED
}

// ---------------------------------------------------------------------------
// Custom NgConfirmCount=2: shutdown fires on the 2nd consecutive NG sample
// ---------------------------------------------------------------------------
TEST(PgoodDebouncerTest, CustomNgConfirmCount2_ShutdownOnSecondSample)
{
    using D2    = PgoodDebouncerT<true, 2>;
    using Idx2  = D2::SignalIndex;
    using St2   = D2::State;
    D2 d;

    // 1st NG: PENDING_NG, no shutdown
    EXPECT_FALSE(d.update(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(Idx2::V24).state,    St2::PENDING_NG);
    EXPECT_EQ(d.get_signal(Idx2::V24).ng_count, 1u);

    // 2nd NG: NG_CONFIRMED -> shutdown
    EXPECT_TRUE(d.update(true, false, false, false, false));
    EXPECT_EQ(d.get_signal(Idx2::V24).state,    St2::NG_CONFIRMED);
    EXPECT_EQ(d.get_signal(Idx2::V24).ng_count, 2u);
    EXPECT_TRUE(d.is_ng_confirmed());
}

TEST(PgoodDebouncerTest, CustomNgConfirmCount2_SingleNgThenOkNoShutdown)
{
    using D2   = PgoodDebouncerT<true, 2>;
    using Idx2 = D2::SignalIndex;
    using St2  = D2::State;
    D2 d;

    EXPECT_FALSE(d.update(true, false, false, false, false));   // ng_count -> 1, PENDING_NG
    EXPECT_FALSE(d.update(false, false, false, false, false));  // ng_count -> 0, OK
    EXPECT_EQ(d.get_signal(Idx2::V24).state,    St2::OK);
    EXPECT_EQ(d.get_signal(Idx2::V24).ng_count, 0u);
    EXPECT_FALSE(d.is_ng_confirmed());
}
