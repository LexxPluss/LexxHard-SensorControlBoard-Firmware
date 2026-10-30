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
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <type_traits>

#include <zephyr/ztest.h>

#include "lcs_diag.hpp"

using lexxhard::lcs::Diag;
using lexxhard::lcs::format_group;
using lexxhard::lcs::IntervalTracker;
using lexxhard::lcs::kDiagGroups;
using lexxhard::lcs::kDiagLineMax;
using lexxhard::lcs::kDiagWords;

namespace {

static_assert(sizeof(Diag) == kDiagWords * 4);
static_assert(std::is_trivially_copyable_v<Diag>);

constexpr uint32_t kMax{std::numeric_limits<uint32_t>::max()};
/* "rx" plus 8 values of " 4294967295". */
constexpr size_t kLongestLine{90};

Diag make_numbered()
{
    Diag d{};
    d.rx.stats.delivered = 1;
    d.rx.stats.crc_error = 2;
    d.rx.stats.sof_mismatch = 3;
    d.rx.stats.timeout = 4;
    d.rx.stats.uart_error = 5;
    d.rx.stats.foreign_downlink = 6;
    d.rx.stats.unknown_sender = 7;
    d.rx.restarts = 8;
    d.tx.sent = 9;
    d.tx.skipped = 10;
    d.tx.rejected_busy = 11;
    d.tx.timeout = 12;
    d.sync.late = 13;
    d.sync.max_interval = 14;
    d.isr.rx_queue_overflow = 15;
    return d;
}

Diag make_all_max()
{
    Diag d{};
    d.rx.stats = {kMax, kMax, kMax, kMax, kMax, kMax, kMax};
    d.rx.restarts = kMax;
    d.tx = {kMax, kMax, kMax, kMax};
    d.sync = {kMax, kMax};
    d.isr = {kMax};
    return d;
}

}  // namespace

ZTEST_SUITE(lcs_diag, NULL, NULL, NULL, NULL, NULL);

ZTEST(lcs_diag, test_dg1_group_counts_sum_to_words)
{
    size_t total{0};
    size_t expected_first{0};
    for (const auto &g : kDiagGroups) {
        zassert_equal(g.first, expected_first);
        expected_first += g.count;
        total += g.count;
    }
    zassert_equal(total, kDiagWords);
    zassert_equal(kDiagGroups.size(), 4u);
}

ZTEST(lcs_diag, test_dg2_group_order_and_names)
{
    const std::string_view expected[]{"rx", "tx", "sync", "isr"};
    zassert_equal(kDiagGroups.size(), 4u);
    for (size_t i{0}; i < kDiagGroups.size(); ++i) {
        zassert_true(kDiagGroups[i].name == expected[i]);
    }
}

ZTEST(lcs_diag, test_dg3_bit_cast_order_matches_declaration)
{
    const Diag d{make_numbered()};
    const auto words{std::bit_cast<std::array<uint32_t, kDiagWords>>(d)};
    for (size_t i{0}; i < kDiagWords; ++i) {
        zassert_equal(words[i], static_cast<uint32_t>(i + 1));
    }
}

ZTEST(lcs_diag, test_dg4_golden_lines)
{
    const Diag d{make_numbered()};
    const char *const expected[]{
        "rx 1 2 3 4 5 6 7 8",
        "tx 9 10 11 12",
        "sync 13 14",
        "isr 15",
    };
    for (size_t g{0}; g < 4; ++g) {
        char buf[kDiagLineMax];
        const size_t len{format_group(d, g, buf, sizeof(buf))};
        zassert_equal(len, strlen(expected[g]));
        zassert_equal(strcmp(buf, expected[g]), 0);
    }
}

ZTEST(lcs_diag, test_dg4_line_max_is_longest_line_plus_nul)
{
    zassert_equal(kDiagLineMax, kLongestLine + 1);
}

ZTEST(lcs_diag, test_dg4_longest_line_fits_exactly)
{
    const Diag d{make_all_max()};
    char buf[kLongestLine + 1];
    const size_t len{format_group(d, 0, buf, sizeof(buf))};
    zassert_equal(len, kLongestLine);
    zassert_equal(strlen(buf), kLongestLine);
    zassert_equal(strcmp(buf, "rx 4294967295 4294967295 4294967295 4294967295 "
                              "4294967295 4294967295 4294967295 4294967295"), 0);
}

ZTEST(lcs_diag, test_dg4_one_byte_short_returns_zero_and_leaves_buffer)
{
    const Diag d{make_all_max()};
    char buf[kLongestLine + 1];
    memset(buf, 0x5A, sizeof(buf));
    const size_t len{format_group(d, 0, buf, kLongestLine)};
    zassert_equal(len, 0u);
    for (const char c : buf) {
        zassert_equal(c, 0x5A);
    }
}

ZTEST(lcs_diag, test_dg4_group_out_of_range_returns_zero)
{
    const Diag d{make_numbered()};
    char buf[kDiagLineMax];
    memset(buf, 0x5A, sizeof(buf));
    zassert_equal(format_group(d, 4, buf, sizeof(buf)), 0u);
    zassert_equal(buf[0], 0x5A);
}

ZTEST(lcs_diag, test_it1_first_start_gives_no_interval)
{
    IntervalTracker t;
    t.on_start(1000);
    zassert_equal(t.max(), 0u);
}

ZTEST(lcs_diag, test_it2_second_start_gives_difference)
{
    IntervalTracker t;
    t.on_start(1000);
    t.on_start(1100);
    zassert_equal(t.max(), 100u);
}

ZTEST(lcs_diag, test_it3_max_keeps_largest)
{
    IntervalTracker t;
    t.on_start(0);
    t.on_start(100);
    t.on_start(350);
    zassert_equal(t.max(), 250u);
    t.on_start(450);
    zassert_equal(t.max(), 250u);
}

ZTEST(lcs_diag, test_it4_wrap_safe)
{
    IntervalTracker t;
    t.on_start(0xFFFFFFF0u);
    t.on_start(0x10u);
    zassert_equal(t.max(), 0x20u);
}

ZTEST(lcs_diag, test_it5_skipped_sync_shows_double_interval)
{
    IntervalTracker t;
    t.on_start(0);
    t.on_start(100);
    zassert_equal(t.max(), 100u);
    // The SYNC due at 200 was skipped: no on_start call.
    t.on_start(300);
    zassert_equal(t.max(), 200u);
}

ZTEST(lcs_diag, test_it6_reset_returns_to_initial_state)
{
    IntervalTracker t;
    t.on_start(0);
    t.on_start(500);
    t.reset();
    zassert_equal(t.max(), 0u);
    // The first start after reset gives no interval again.
    t.on_start(10000);
    zassert_equal(t.max(), 0u);
    t.on_start(10100);
    zassert_equal(t.max(), 100u);
}
