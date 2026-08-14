/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Properties of the scheduling model, and the sweep it exists to produce.
 *
 * These tests assert **structure only** — no starvation, determinism, and monotonicity in
 * the directions physics requires. They deliberately do not assert that any point-sensor
 * rate is achieved: every input is a design-time figure that no logic analyser has
 * confirmed, so a passing threshold here would be a product claim the evidence cannot
 * support. The sweep prints the numbers; a human reads them.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_schedule_model.hpp"

namespace {

namespace sch = tof_schedule;

constexpr uint32_t k100k{100000}, k400k{400000}, k1M{1000000};
constexpr uint32_t kUntrimmed{1452}, kTrimmed{328};

// Plain arrays rather than braced ranges: this build has no <initializer_list>.
constexpr uint32_t kSpeeds[]{k100k, k400k, k1M};
constexpr uint32_t kBlockSizes[]{kUntrimmed, kTrimmed};
constexpr uint32_t kChunkSizes[]{0, 512, 256, 128, 64};

uint64_t worst_point_interval(const sch::sim_result &r)
{
    uint64_t worst{0};
    for (int i = 0; i < r.device_count; ++i)
        if (r.devices[i].sort == sch::kind::point && r.devices[i].max_interval_ns > worst)
            worst = r.devices[i].max_interval_ns;
    return worst;
}

uint32_t slowest_point_millihz(const sch::sim_result &r)
{
    uint64_t worst_mean{0};
    for (int i = 0; i < r.device_count; ++i)
        if (r.devices[i].sort == sch::kind::point && r.devices[i].mean_interval_ns > worst_mean)
            worst_mean = r.devices[i].mean_interval_ns;
    return worst_mean == 0 ? 0 : static_cast<uint32_t>(1000000000000ull / worst_mean);
}

uint64_t worst_grid_latency(const sch::sim_result &r)
{
    uint64_t worst{0};
    for (int i = 0; i < r.device_count; ++i)
        if (r.devices[i].sort == sch::kind::grid && r.devices[i].max_latency_ns > worst)
            worst = r.devices[i].max_latency_ns;
    return worst;
}

}  // namespace

ZTEST_SUITE(tof_schedule, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------------ structure ----- */

ZTEST(tof_schedule, test_no_source_is_starved)
{
    // The case that has to work: both grids ready at once, at the slowest bus, untrimmed.
    // If any source can be starved, it shows here first.
    for (uint32_t speed : kSpeeds) {
        for (uint32_t bytes : kBlockSizes) {
            const auto r{sch::run(sch::six_sensor_chain(speed, bytes, 0, true))};

            for (int i = 0; i < r.device_count; ++i) {
                zassert_true(r.devices[i].completed_reads > 0,
                             "device %d never serviced at %u Hz, %u bytes", i, speed, bytes);
                zassert_true(r.devices[i].max_interval_ns < r.horizon_ns,
                             "device %d starved at %u Hz, %u bytes", i, speed, bytes);
            }
        }
    }
}

ZTEST(tof_schedule, test_the_model_is_deterministic)
{
    // Two runs of one configuration must be identical, jitter included; otherwise the
    // sweep below is not evidence of anything.
    auto cfg{sch::six_sensor_chain(k400k, kTrimmed, 0, true)};
    cfg.scheduler_jitter_max_ns = 200000;
    cfg.can_enqueue_ns = 50000;

    const auto a{sch::run(cfg)};
    const auto b{sch::run(cfg)};

    zassert_equal(memcmp(&a, &b, sizeof a), 0, "the same configuration gave two answers");
}

ZTEST(tof_schedule, test_a_faster_bus_never_makes_a_point_sensor_worse)
{
    const auto slow{sch::run(sch::six_sensor_chain(k100k, kTrimmed, 0, true))};
    const auto mid{sch::run(sch::six_sensor_chain(k400k, kTrimmed, 0, true))};
    const auto fast{sch::run(sch::six_sensor_chain(k1M, kTrimmed, 0, true))};

    zassert_true(worst_point_interval(mid) <= worst_point_interval(slow));
    zassert_true(worst_point_interval(fast) <= worst_point_interval(mid));
    zassert_true(fast.bus_utilisation_permille <= mid.bus_utilisation_permille);
}

ZTEST(tof_schedule, test_trimming_never_makes_a_point_sensor_worse)
{
    // This is the claim trimming exists to make, and the only one the model can support.
    for (uint32_t speed : kSpeeds) {
        const auto untrimmed{sch::run(sch::six_sensor_chain(speed, kUntrimmed, 0, true))};
        const auto trimmed{sch::run(sch::six_sensor_chain(speed, kTrimmed, 0, true))};

        zassert_true(worst_point_interval(trimmed) <= worst_point_interval(untrimmed),
                     "trimming hurt the point sensors at %u Hz", speed);
    }
}

ZTEST(tof_schedule, test_smaller_chunks_never_lengthen_the_uninterruptible_transfer)
{
    // Chunking's only mechanism: it shortens the block during which nothing else can be
    // served. If that ever failed to hold, the model would be wrong.
    uint64_t previous{UINT64_MAX};
    for (uint32_t chunk : kChunkSizes) {
        const auto r{sch::run(sch::six_sensor_chain(k400k, kTrimmed, chunk, true))};

        zassert_true(r.longest_uninterruptible_ns <= previous,
                     "chunk %u lengthened the uninterruptible transfer", chunk);
        previous = r.longest_uninterruptible_ns;
    }
}

ZTEST(tof_schedule, test_grids_in_phase_are_the_harder_case)
{
    // Both frames arriving together is what the point sensors have to survive; the
    // staggered case must not be the one we tune against.
    const auto together{sch::run(sch::six_sensor_chain(k400k, kUntrimmed, 0, true))};
    const auto staggered{sch::run(sch::six_sensor_chain(k400k, kUntrimmed, 0, false))};

    zassert_true(worst_point_interval(together) >= worst_point_interval(staggered),
                 "the in-phase case was not the worse one, so the model is suspect");
}

/* ---------------------------------------------------------------------- sweep ----- */

ZTEST(tof_schedule, test_print_the_sweep)
{
    // Not an assertion — the table this model exists to produce. Everything in it rests
    // on provisional inputs: 1452 / 328 / 133 bytes and the bus speeds are design-time
    // figures, so read it as "what the schedule would do if those hold".
    static const struct {
        const char *name;
        uint32_t bytes;
        uint32_t chunk;
    } shapes[] = {
        {"untrimmed 1452, atomic", kUntrimmed, 0},   {"trimmed 328, atomic", kTrimmed, 0},
        {"trimmed 328, chunk 128", kTrimmed, 128},   {"trimmed 328, chunk 64", kTrimmed, 64},
        {"untrimmed 1452, chunk 256", kUntrimmed, 256},
    };

    TC_PRINT("\nAssumed point-sensor timing budget: 20 ms. Every figure below moves with it.\n");
    TC_PRINT("\n%-28s %8s %11s %11s %10s %9s %7s\n", "grid read shape", "bus", "worst L4 gap",
             "slowest L4", "grid lat", "longest tx", "bus use");
    for (uint32_t speed : kSpeeds) {
        for (const auto &s : shapes) {
            const auto r{sch::run(sch::six_sensor_chain(speed, s.bytes, s.chunk, true))};

            TC_PRINT("%-28s %6u k %8llu ms %8u mHz %7llu ms %6llu ms %5u.%u%%\n", s.name,
                     speed / 1000, static_cast<unsigned long long>(worst_point_interval(r) / 1000000),
                     slowest_point_millihz(r),
                     static_cast<unsigned long long>(worst_grid_latency(r) / 1000000),
                     static_cast<unsigned long long>(r.longest_uninterruptible_ns / 1000000),
                     r.bus_utilisation_permille / 10, r.bus_utilisation_permille % 10);
        }
    }
    // The point-sensor rate is dominated by an input nobody has fixed. Printing the
    // sensitivity is the only way to stop the table above being read as a product claim.
    static const uint64_t budgets[]{10ull * 1000000, 20ull * 1000000, 33ull * 1000000,
                                    50ull * 1000000};
    TC_PRINT("\nSensitivity to that assumption, at 400 kHz with a trimmed atomic grid read:\n");
    TC_PRINT("%-18s %13s %12s\n", "timing budget", "worst L4 gap", "slowest L4");
    for (uint64_t b : budgets) {
        const auto r{sch::run(sch::six_sensor_chain(k400k, kTrimmed, 0, true, b))};

        TC_PRINT("%15llu ms %10llu ms %8u mHz\n", static_cast<unsigned long long>(b / 1000000),
                 static_cast<unsigned long long>(worst_point_interval(r) / 1000000),
                 slowest_point_millihz(r));
    }

    TC_PRINT("\nRead the chunk rows as an upper bound on what chunking could buy, not as a\n"
             "recommendation: splitting a grid read is only safe if the device's result buffer\n"
             "stays coherent between chunks, which this model does not and cannot show.\n\n");
}
