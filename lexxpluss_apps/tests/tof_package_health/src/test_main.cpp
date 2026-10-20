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

/*
 * The health decision table, exercised over every status value a zone can carry.
 *
 * The suite's last case is the one to read first: OK means the acquisition was normal under this
 * rule, and nothing more. It does not mean the shutter may close.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_package_health.hpp"

using namespace lexxhard::tof_package_health;

namespace {

input grid_of(const uint8_t (&status)[kZones])
{
    input in{};
    in.outcome = acquisition::complete_grid;
    in.zone_status = status;
    in.zone_count = kZones;
    return in;
}

verdict judge_uniform(uint8_t status, uint8_t (&buffer)[kZones])
{
    for (size_t z{0}; z < kZones; ++z)
        buffer[z] = status;
    return judge(grid_of(buffer));
}

}  // namespace

/* EVERY STATUS VALUE, applied to every zone. Exactly two values are normal; the other 254 degrade,
 * and all of them publish. */
ZTEST(tof_package_health, test_all_256_status_values_uniformly)
{
    uint8_t buffer[kZones]{};
    for (unsigned s{0}; s <= 0xFFU; ++s) {
        const verdict v{judge_uniform(static_cast<uint8_t>(s), buffer)};
        const bool should_be_ok{s == kStatusValid || s == kStatusNoTarget};

        zassert_equal(v.state, should_be_ok ? sensor_state::ok : sensor_state::degraded,
                      "status 0x%02x gave %s", s, state_name(v.state));
        zassert_true(v.publish_grid, "status 0x%02x withheld the grid", s);
        zassert_equal(v.avail, availability::active);
        /* The counts add up and attribute every zone. */
        zassert_equal(v.normal_zones + v.no_target_zones + v.abnormal_zones, kZones,
                      "status 0x%02x lost zones", s);
        if (s == kStatusValid)
            zassert_equal(v.normal_zones, kZones);
        else if (s == kStatusNoTarget)
            zassert_equal(v.no_target_zones, kZones);
        else
            zassert_equal(v.abnormal_zones, kZones);
    }
}

/* A COMPLETELY EMPTY FIELD OF VIEW IS NOT A FAULT. Every zone reporting "no target detected" is
 * what a clear space measures as, and calling it degradation would report a fault for the most
 * ordinary reading there is. */
ZTEST(tof_package_health, test_all_no_target_is_ok)
{
    uint8_t buffer[kZones]{};
    const verdict v{judge_uniform(kStatusNoTarget, buffer)};
    zassert_equal(v.state, sensor_state::ok);
    zassert_equal(v.no_target_zones, kZones);
    zassert_equal(v.abnormal_zones, 0U);
    zassert_true(v.publish_grid);
}

/* ONE ZONE IS ENOUGH, at every position and for a spread of abnormal values. A threshold would
 * need a measured number and there is none. */
ZTEST(tof_package_health, test_a_single_abnormal_zone_degrades_the_grid)
{
    static constexpr uint8_t kAbnormal[]{0, 1, 4, 6, 9, 100, 200, 254};
    static constexpr uint8_t kNormalBackground[]{kStatusValid, kStatusNoTarget};

    for (uint8_t background : kNormalBackground) {
        for (uint8_t bad : kAbnormal) {
            for (size_t z{0}; z < kZones; ++z) {
                uint8_t buffer[kZones]{};
                for (size_t i{0}; i < kZones; ++i)
                    buffer[i] = background;
                buffer[z] = bad;

                const verdict v{judge(grid_of(buffer))};
                zassert_equal(v.state, sensor_state::degraded,
                              "status 0x%02x at zone %zu on a 0x%02x field read as %s", bad, z,
                              background, state_name(v.state));
                zassert_equal(v.abnormal_zones, 1U);
                zassert_true(v.publish_grid, "a degraded grid was withheld");
            }
        }
    }
}

/* 6 AND 9 DEGRADE, which diverges from the chain contract's accept_low_confidence option on
 * purpose. DEGRADED is not a refusal: the grid still publishes and the raw status is untouched, so
 * the consumer keeps every option that policy would have given it. */
ZTEST(tof_package_health, test_low_confidence_statuses_degrade_but_still_publish)
{
    static constexpr uint8_t kLowConfidence[]{6, 9};
    for (uint8_t s : kLowConfidence) {
        uint8_t buffer[kZones]{};
        const verdict v{judge_uniform(s, buffer)};
        zassert_equal(v.state, sensor_state::degraded, "status %u", s);
        zassert_true(v.publish_grid, "status %u withheld the grid", s);
        zassert_equal(v.abnormal_zones, kZones);
    }
}

/* A status this firmware does not recognise is not a status it may call normal. */
ZTEST(tof_package_health, test_unknown_statuses_degrade)
{
    static constexpr uint8_t kUnknown[]{2, 3, 7, 8, 10, 11, 42, 128, 253, 254};
    for (uint8_t s : kUnknown) {
        uint8_t buffer[kZones]{};
        const verdict v{judge_uniform(s, buffer)};
        zassert_equal(v.state, sensor_state::degraded, "unknown status %u read as %s", s,
                      state_name(v.state));
    }
}

/* A mixture of the two normal values is normal, in any proportion. */
ZTEST(tof_package_health, test_any_mixture_of_valid_and_no_target_is_ok)
{
    for (size_t valid_count{0}; valid_count <= kZones; ++valid_count) {
        uint8_t buffer[kZones]{};
        for (size_t z{0}; z < kZones; ++z)
            buffer[z] = z < valid_count ? kStatusValid : kStatusNoTarget;

        const verdict v{judge(grid_of(buffer))};
        zassert_equal(v.state, sensor_state::ok, "%zu valid zones read as %s", valid_count,
                      state_name(v.state));
        zassert_equal(v.normal_zones, valid_count);
        zassert_equal(v.no_target_zones, kZones - valid_count);
    }
}

/* An acquisition that did not happen is ERROR, and NO GRID IS MANUFACTURED to accompany it: no
 * all-sentinel frame, nothing for a timestamp to be refreshed on. */
ZTEST(tof_package_health, test_acquisition_failure_is_error_without_a_grid)
{
    input in{};
    in.outcome = acquisition::failed;
    const verdict v{judge(in)};
    zassert_equal(v.state, sensor_state::error);
    zassert_false(v.publish_grid, "a failed acquisition published a grid");
    zassert_equal(v.avail, availability::active, "a failure is not the same as never started");
    zassert_equal(v.normal_zones, 0U);
    zassert_equal(v.no_target_zones, 0U);
    zassert_equal(v.abnormal_zones, 0U);
}

/* Structurally invalid data is ERROR, and the claim "this is a complete grid" is checked rather
 * than trusted: the caller's own buffer is the one input this module can verify. */
ZTEST(tof_package_health, test_structurally_invalid_data_is_error)
{
    uint8_t buffer[kZones]{};
    for (size_t z{0}; z < kZones; ++z)
        buffer[z] = kStatusValid;

    {
        input in{};
        in.outcome = acquisition::structurally_invalid;
        in.zone_status = buffer;
        in.zone_count = kZones;
        const verdict v{judge(in)};
        zassert_equal(v.state, sensor_state::error);
        zassert_false(v.publish_grid);
    }
    /* A complete_grid that is short, long, or has no buffer at all. */
    static constexpr size_t kBadCounts[]{0, 1, 63, 65, 128};
    for (size_t count : kBadCounts) {
        input in{};
        in.outcome = acquisition::complete_grid;
        in.zone_status = buffer;
        in.zone_count = count;
        const verdict v{judge(in)};
        zassert_equal(v.state, sensor_state::error, "zone_count %zu was accepted", count);
        zassert_false(v.publish_grid);
    }
    {
        input in{};
        in.outcome = acquisition::complete_grid;
        in.zone_status = nullptr;
        in.zone_count = kZones;
        const verdict v{judge(in)};
        zassert_equal(v.state, sensor_state::error, "a null buffer was accepted");
        zassert_false(v.publish_grid);
    }
}

/* INACTIVE is carried by the INDEPENDENT diagnostic, and the three-state enum is not extended.
 * The state is ERROR because "no data" is what the requirement's third value means and there is no
 * other; the distinction lives in availability. */
ZTEST(tof_package_health, test_inactive_uses_the_separate_diagnostic)
{
    input in{};
    in.outcome = acquisition::never_started;
    const verdict v{judge(in)};
    zassert_equal(v.state, sensor_state::error);
    zassert_equal(v.avail, availability::inactive);
    zassert_false(v.publish_grid, "an inactive sensor published a grid");

    /* The enum still has exactly the requirement's three values. If a fourth is ever added this
     * fails, which is the point. */
    zassert_equal(static_cast<uint8_t>(sensor_state::ok), 0U);
    zassert_equal(static_cast<uint8_t>(sensor_state::degraded), 1U);
    zassert_equal(static_cast<uint8_t>(sensor_state::error), 2U);
}

/* NO HYSTERESIS. The next complete grid is judged by its own content, so recovery is immediate in
 * both directions -- and the module is stateless, so it could not behave otherwise. */
ZTEST(tof_package_health, test_recovery_is_immediate_and_there_is_no_hysteresis)
{
    uint8_t good[kZones]{};
    uint8_t bad[kZones]{};
    for (size_t z{0}; z < kZones; ++z) {
        good[z] = kStatusValid;
        bad[z] = kStatusValid;
    }
    bad[17] = 0;   // "data not updated" in one zone

    input failure{};
    failure.outcome = acquisition::failed;

    /* ERROR, then a good grid: OK at once, with no intermediate state. */
    zassert_equal(judge(failure).state, sensor_state::error);
    zassert_equal(judge(grid_of(good)).state, sensor_state::ok);

    /* DEGRADED, then a good grid: OK at once. */
    zassert_equal(judge(grid_of(bad)).state, sensor_state::degraded);
    zassert_equal(judge(grid_of(good)).state, sensor_state::ok);

    /* And back, repeatedly: every verdict depends only on its own input. */
    for (int i{0}; i < 5; ++i) {
        zassert_equal(judge(grid_of(bad)).state, sensor_state::degraded, "cycle %d", i);
        zassert_equal(judge(grid_of(good)).state, sensor_state::ok, "cycle %d", i);
        zassert_equal(judge(failure).state, sensor_state::error, "cycle %d", i);
    }
}

/* THE RAW STATUS IS NEVER REWRITTEN. The module takes a const pointer, and this checks that it
 * really leaves the caller's buffer alone -- the verdict is a reading, not an edit. */
ZTEST(tof_package_health, test_the_status_buffer_is_not_modified)
{
    uint8_t buffer[kZones]{};
    uint8_t witness[kZones]{};
    for (size_t z{0}; z < kZones; ++z) {
        buffer[z] = static_cast<uint8_t>(z * 4U);
        witness[z] = buffer[z];
    }
    (void)judge(grid_of(buffer));
    zassert_equal(0, memcmp(buffer, witness, sizeof buffer), "the status buffer was modified");
}

/* WHAT OK DOES NOT MEAN, as an assertion rather than a caveat.
 *
 * A grid where every zone returned no target and a grid where every zone measured a trusted
 * distance BOTH read OK, and this verdict cannot tell them apart. The difference between "nothing
 * was detected" and "a clear path was measured" is exactly what a closing decision turns on -- and
 * per UM3038 a 255 can also come from low reflectivity, strong ambient light, a target out of range
 * or a steep incidence angle. So OK means the acquisition was normal under this rule. It does not
 * mean the shutter may close, and FW-5's interlock keeps its own judgement for that reason. */
ZTEST(tof_package_health, test_ok_does_not_mean_safe_to_close)
{
    uint8_t all_no_target[kZones]{};
    uint8_t all_trusted[kZones]{};
    for (size_t z{0}; z < kZones; ++z) {
        all_no_target[z] = kStatusNoTarget;
        all_trusted[z] = kStatusValid;
    }

    const verdict empty{judge(grid_of(all_no_target))};
    const verdict measured{judge(grid_of(all_trusted))};

    zassert_equal(empty.state, sensor_state::ok);
    zassert_equal(measured.state, sensor_state::ok);
    zassert_equal(empty.state, measured.state,
                  "if these ever differ, the comment above needs rewriting");

    /* The ONLY thing that distinguishes them is a diagnostic count, which is not a gate and is not
     * what the requirement's health byte carries. */
    zassert_not_equal(empty.no_target_zones, measured.no_target_zones);
    zassert_equal(empty.abnormal_zones, measured.abnormal_zones);
}

ZTEST(tof_package_health, test_names)
{
    zassert_equal(0, strcmp(state_name(sensor_state::ok), "OK"));
    zassert_equal(0, strcmp(state_name(sensor_state::degraded), "DEGRADED"));
    zassert_equal(0, strcmp(state_name(sensor_state::error), "ERROR"));
    zassert_equal(0, strcmp(availability_name(availability::inactive), "INACTIVE"));
    zassert_equal(0, strcmp(availability_name(availability::active), "active"));
}

ZTEST_SUITE(tof_package_health, NULL, NULL, NULL, NULL, NULL);
