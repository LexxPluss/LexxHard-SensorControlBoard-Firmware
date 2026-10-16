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
 * tof_package_stream: the publication rule.
 *
 * WHAT THESE TESTS ARE FOR. Three properties decide whether a consumer can trust a grid's identity,
 * and none of them is visible from a single frame:
 *
 * - one real acquisition takes exactly one sequence number, and all 65 of its frames carry it;
 * - a send that fails does not put the grid back on the bus under a later number;
 * - a stream that has stopped acquiring sends no measurement, only diagnostics that belong to no
 *   acquisition.
 *
 * Each is asserted against a RECORDED FRAME LOG rather than a return code, because the return code
 * is the module's own account of itself: the bytes are what a consumer sees. Every frame is decoded
 * back with tof_package_wire_n, so a layout change breaks these tests rather than passing silently.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_package_stream.hpp"
#include "tof_package_wire_n.hpp"

namespace stream = lexxhard::tof_package_stream;
namespace wire = lexxhard::tof_package_wire_n;
namespace health = lexxhard::tof_package_health;

namespace {

/* 65 frames per grid, and the longest test publishes two grids plus diagnostics. */
constexpr size_t kMaxLog{200};

struct logged {
    bool is_data{false};
    uint8_t bytes[wire::kFrameBytes]{};
};

struct spy_state {
    logged log[kMaxLog]{};
    size_t logged_count{0};
    /* Which data send fails, 1-based in call order. 0 means none ever fails. */
    size_t fail_data_call{0};
    size_t data_calls{0};
    bool fail_health{false};
    size_t health_calls{0};
};

spy_state spy{};

void record(bool is_data, const uint8_t *frame, size_t len)
{
    if (spy.logged_count >= kMaxLog || len != wire::kFrameBytes)
        return;
    spy.log[spy.logged_count].is_data = is_data;
    memcpy(spy.log[spy.logged_count].bytes, frame, len);
    ++spy.logged_count;
}

int send_data(void *, const uint8_t *frame, size_t len)
{
    ++spy.data_calls;
    /* The failure is counted and NOT logged: a send that failed did not put bytes on the bus, and a
     * test that logged it could not tell "64 frames sent" from "64 attempted, one lost". */
    if (spy.fail_data_call != 0 && spy.data_calls == spy.fail_data_call)
        return -EIO;
    record(true, frame, len);
    return 0;
}

int send_health(void *, const uint8_t *frame, size_t len)
{
    ++spy.health_calls;
    if (spy.fail_health)
        return -EIO;
    record(false, frame, len);
    return 0;
}

struct seq_state {
    uint32_t next{1};
    /* Which issue() call refuses, 1-based. 0 means none. */
    size_t fail_call{0};
    size_t calls{0};
};

seq_state seq{};

int issue(void *, uint32_t *out)
{
    ++seq.calls;
    if (seq.fail_call != 0 && seq.calls == seq.fail_call)
        return -ENOSPC;
    *out = seq.next++;
    return 0;
}

/* A source that breaks its own contract by handing back the reserved number -- its own function
 * rather than a flag on seq_state, because the reserved value is zero and a "0 means unset" field
 * cannot express it, which is the same confusion the layout refuses on the wire. */
int issue_reserved(void *, uint32_t *out)
{
    ++seq.calls;
    *out = wire::kReservedSequence;
    return 0;
}

stream::sink wired_sink()
{
    stream::sink s{};
    s.send_data = send_data;
    s.send_health = send_health;
    return s;
}

stream::sequence_source wired_source(int (*fn)(void *, uint32_t *) = issue)
{
    stream::sequence_source s{};
    s.issue = fn;
    return s;
}

/* A grid whose zones are all distinguishable, so a reordering or an off-by-one is visible. Status 5
 * is the health rule's "valid", which keeps the verdict OK unless a test changes a zone. */
struct test_grid {
    uint16_t distance_mm[wire::kZones]{};
    uint8_t status[wire::kZones]{};
};

void fill(test_grid &g, uint16_t base = 1000)
{
    for (size_t z{0}; z < wire::kZones; ++z) {
        g.distance_mm[z] = static_cast<uint16_t>(base + z);
        g.status[z] = health::kStatusValid;
    }
}

stream::input grid_input(const test_grid &g)
{
    stream::input in{};
    in.outcome = health::acquisition::complete_grid;
    in.distance_mm = g.distance_mm;
    in.status = g.status;
    in.zone_count = wire::kZones;
    return in;
}

void reset_all()
{
    spy = spy_state{};
    seq = seq_state{};
}

size_t data_frames_logged()
{
    size_t n{0};
    for (size_t i{0}; i < spy.logged_count; ++i)
        if (spy.log[i].is_data)
            ++n;
    return n;
}

size_t health_frames_logged()
{
    return spy.logged_count - data_frames_logged();
}

}  // namespace

ZTEST_SUITE(tof_package_stream, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_package_stream, test_init_refuses_a_partial_sink)
{
    reset_all();
    stream::publisher p{};

    stream::sink half{};
    half.send_data = send_data;
    zassert_equal(p.init(half, wired_source()), -EINVAL,
                  "a sink with no health callback must be refused");

    stream::sink other_half{};
    other_half.send_health = send_health;
    zassert_equal(p.init(other_half, wired_source()), -EINVAL,
                  "a sink with no data callback must be refused");

    stream::sequence_source none{};
    zassert_equal(p.init(wired_sink(), none), -EINVAL, "a source with no issue callback");

    /* And the refusal left it unusable rather than half-wired. */
    stream::outcome o{};
    test_grid g{};
    fill(g);
    zassert_equal(p.publish(grid_input(g), &o), -EPERM, "a refused init must not leave it usable");
    zassert_equal(o, stream::outcome::not_configured, "%s", stream::outcome_name(o));
    zassert_equal(spy.logged_count, 0U, "nothing may go out");
}

ZTEST(tof_package_stream, test_publish_before_init_sends_nothing)
{
    reset_all();
    stream::publisher p{};
    stream::outcome o{};
    test_grid g{};
    fill(g);

    zassert_equal(p.publish(grid_input(g), &o), -EPERM, "no init, no publish");
    zassert_equal(o, stream::outcome::not_configured, "%s", stream::outcome_name(o));
    zassert_equal(spy.data_calls, 0U, "the sink must not be called");
    zassert_equal(spy.health_calls, 0U, "the sink must not be called");
    zassert_equal(seq.calls, 0U, "and no number may be drawn");
}

ZTEST(tof_package_stream, test_a_complete_grid_is_65_frames_with_one_sequence)
{
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish");
    zassert_equal(o, stream::outcome::grid_published, "%s", stream::outcome_name(o));

    zassert_equal(spy.logged_count, wire::kFramesPerGrid, "a grid is 64 data frames and one health");
    zassert_equal(data_frames_logged(), wire::kDataFramesPerGrid, "64 data frames");
    zassert_equal(health_frames_logged(), 1U, "exactly one health frame");
    zassert_equal(seq.calls, 1U, "one acquisition draws exactly one number");

    /* The metadata frame is LAST: the producer's claim that a complete grid was taken must not
     * precede the grid itself onto the wire. */
    zassert_false(spy.log[wire::kFramesPerGrid - 1].is_data, "the last frame is the metadata");
    for (size_t i{0}; i < wire::kDataFramesPerGrid; ++i)
        zassert_true(spy.log[i].is_data, "frame %u should be data", (unsigned)i);

    /* Every data frame: ascending zone, the caller's distance and the caller's raw status, all
     * under one sequence. */
    uint32_t grid_sequence{0};
    for (size_t z{0}; z < wire::kDataFramesPerGrid; ++z) {
        wire::data_frame df{};
        zassert_equal(wire::decode_data(spy.log[z].bytes, wire::kFrameBytes, df), wire::result::ok,
                      "zone %u decodes", (unsigned)z);
        zassert_equal(df.zone, z, "zones ascend: frame %u carries zone %u", (unsigned)z, df.zone);
        zassert_equal(df.distance_mm, g.distance_mm[z], "zone %u distance", (unsigned)z);
        zassert_equal(df.status, g.status[z], "zone %u status", (unsigned)z);
        zassert_not_equal(df.sequence, wire::kReservedSequence, "never the reserved number");
        if (z == 0)
            grid_sequence = df.sequence;
        else
            zassert_equal(df.sequence, grid_sequence,
                          "zone %u carries a different sequence than zone 0", (unsigned)z);
    }

    wire::health_frame meta{};
    zassert_equal(wire::decode_health(spy.log[wire::kFramesPerGrid - 1].bytes, wire::kFrameBytes,
                                      meta),
                  wire::result::ok, "metadata decodes");
    zassert_equal(meta.purpose, wire::health_purpose::grid_metadata, "it is grid metadata");
    zassert_equal(meta.sequence, grid_sequence, "the metadata names the grid's own acquisition");
    zassert_equal(meta.state, health::sensor_state::ok, "all-valid zones judge OK");
    zassert_false(meta.inactive, "a grid that exists cannot be inactive");

    const stream::counters c{p.current()};
    zassert_equal(c.grids_published, 1U, "one grid");
    zassert_equal(c.grids_abandoned, 0U, "none abandoned");
    zassert_equal(c.last_sequence, grid_sequence, "the counter names the grid's number");
}

ZTEST(tof_package_stream, test_a_second_acquisition_gets_its_own_number)
{
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid first{};
    fill(first, 1000);
    test_grid second{};
    fill(second, 2000);

    stream::outcome o{};
    zassert_equal(p.publish(grid_input(first), &o), 0, "first");
    zassert_equal(o, stream::outcome::grid_published, "%s", stream::outcome_name(o));
    wire::data_frame a{};
    zassert_equal(wire::decode_data(spy.log[0].bytes, wire::kFrameBytes, a), wire::result::ok, "a");

    zassert_equal(p.publish(grid_input(second), &o), 0, "second");
    zassert_equal(o, stream::outcome::grid_published, "%s", stream::outcome_name(o));
    wire::data_frame b{};
    zassert_equal(
        wire::decode_data(spy.log[wire::kFramesPerGrid].bytes, wire::kFrameBytes, b),
        wire::result::ok, "b");

    zassert_not_equal(a.sequence, b.sequence, "two acquisitions must not share a number");
    zassert_equal(seq.calls, 2U, "one number per acquisition, not per frame");

    /* And the second grid carries the SECOND measurement, not a retained copy of the first. */
    for (size_t z{0}; z < wire::kDataFramesPerGrid; ++z) {
        wire::data_frame df{};
        zassert_equal(wire::decode_data(spy.log[wire::kFramesPerGrid + z].bytes, wire::kFrameBytes,
                                        df),
                      wire::result::ok, "zone %u", (unsigned)z);
        zassert_equal(df.distance_mm, second.distance_mm[z],
                      "zone %u came from the first grid, not the second", (unsigned)z);
        zassert_equal(df.sequence, b.sequence, "zone %u is numbered as the second grid",
                      (unsigned)z);
    }
}

ZTEST(tof_package_stream, test_a_failed_data_send_abandons_the_grid)
{
    reset_all();
    spy.fail_data_call = 30;  // part-way through, so some frames are already on the bus

    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish returns normally");
    zassert_equal(o, stream::outcome::grid_abandoned, "%s", stream::outcome_name(o));

    /* It stopped at the failure: no further zone was attempted, and no metadata was sent. Sending
     * the metadata anyway would close a grid that is 35 zones short. */
    zassert_equal(spy.data_calls, 30U, "the grid must stop at the failed zone");
    zassert_equal(spy.health_calls, 0U, "no metadata may close an incomplete grid");
    zassert_equal(data_frames_logged(), 29U, "29 frames reached the bus");

    const stream::counters c{p.current()};
    zassert_equal(c.grids_abandoned, 1U, "one abandoned");
    zassert_equal(c.grids_published, 0U, "and none published");
    zassert_equal(c.send_failures, 1U, "one send failure");
}

ZTEST(tof_package_stream, test_an_abandoned_grid_never_returns_under_a_new_number)
{
    reset_all();
    spy.fail_data_call = 10;

    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid lost{};
    fill(lost, 4000);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(lost), &o), 0, "publish");
    zassert_equal(o, stream::outcome::grid_abandoned, "%s", stream::outcome_name(o));
    const uint32_t spent{p.current().last_sequence};
    /* Every frame logged so far is one of the lost grid's, and they are the only places its number
     * may legitimately appear. */
    const size_t frames_before_the_loss{spy.logged_count};
    const size_t data_before_the_loss{data_frames_logged()};

    /* The caller now stops acquiring and only reports. Nothing of the lost grid may follow. */
    spy.fail_data_call = 0;
    stream::input idle{};
    idle.outcome = health::acquisition::failed;
    for (int i{0}; i < 5; ++i) {
        zassert_equal(p.publish(idle, &o), 0, "diagnostic %d", i);
        zassert_equal(o, stream::outcome::diagnostic_published, "%s", stream::outcome_name(o));
    }

    zassert_equal(data_frames_logged(), data_before_the_loss,
                  "a stopped stream sent a measurement after the grid was abandoned");

    /* Then a genuinely new acquisition arrives. It must be numbered afresh, and no frame of it may
     * carry the number the lost grid took with it. */
    test_grid fresh{};
    fill(fresh, 5000);
    zassert_equal(p.publish(grid_input(fresh), &o), 0, "fresh");
    zassert_equal(o, stream::outcome::grid_published, "%s", stream::outcome_name(o));
    zassert_not_equal(p.current().last_sequence, spent,
                      "the abandoned grid's number was reissued to a later acquisition");

    for (size_t i{0}; i < spy.logged_count; ++i) {
        if (!spy.log[i].is_data)
            continue;
        wire::data_frame df{};
        zassert_equal(wire::decode_data(spy.log[i].bytes, wire::kFrameBytes, df), wire::result::ok,
                      "frame %u", (unsigned)i);
        if (df.sequence != spent)
            continue;
        /* The spent number may appear only among the frames that went out BEFORE it was
         * abandoned -- those are the real, incomplete remains of that acquisition. */
        zassert_true(i < frames_before_the_loss,
                     "frame %u reuses the abandoned grid's number", (unsigned)i);
    }
}

ZTEST(tof_package_stream, test_a_failed_metadata_send_abandons_the_grid)
{
    reset_all();
    spy.fail_health = true;

    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish");
    zassert_equal(o, stream::outcome::grid_abandoned, "%s", stream::outcome_name(o));

    zassert_equal(data_frames_logged(), wire::kDataFramesPerGrid, "all 64 data frames went out");
    zassert_equal(spy.health_calls, 1U, "the metadata was attempted once and not retried");
    zassert_equal(p.current().grids_published, 0U, "a grid with no metadata is not published");
    zassert_equal(p.current().grids_abandoned, 1U, "it is abandoned");
}

ZTEST(tof_package_stream, test_an_unavailable_sequence_sends_nothing_at_all)
{
    reset_all();
    seq.fail_call = 1;

    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish");
    zassert_equal(o, stream::outcome::sequence_unavailable, "%s", stream::outcome_name(o));

    /* NOT a diagnostic. The sensor produced a perfectly good grid; what failed was the issuer, and
     * a health frame here would report the sensor as the faulty part. */
    zassert_equal(spy.logged_count, 0U, "nothing may go out");
    zassert_equal(spy.data_calls, 0U, "no data frame");
    zassert_equal(spy.health_calls, 0U, "and no diagnostic either");
    zassert_equal(p.current().sequence_failures, 1U, "counted");
}

ZTEST(tof_package_stream, test_a_source_that_hands_back_the_reserved_number_is_refused)
{
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source(issue_reserved)), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish");
    zassert_equal(o, stream::outcome::sequence_refused_by_layout, "%s", stream::outcome_name(o));

    /* It fails on zone 0, so nothing is on the bus -- which is why this is not grid_abandoned. */
    zassert_equal(spy.data_calls, 0U, "no frame may be sent with the reserved number");
    zassert_equal(spy.logged_count, 0U, "nothing on the bus");
    zassert_equal(p.current().grids_abandoned, 0U, "nothing was abandoned: nothing was sent");
}

ZTEST(tof_package_stream, test_no_acquisition_sends_one_independent_diagnostic)
{
    const health::acquisition cases[]{
        health::acquisition::failed,
        health::acquisition::structurally_invalid,
        health::acquisition::never_started,
    };

    for (size_t i{0}; i < sizeof cases / sizeof cases[0]; ++i) {
        reset_all();
        stream::publisher p{};
        zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

        stream::input in{};
        in.outcome = cases[i];
        stream::outcome o{};
        zassert_equal(p.publish(in, &o), 0, "publish %u", (unsigned)i);
        zassert_equal(o, stream::outcome::diagnostic_published, "%s", stream::outcome_name(o));

        zassert_equal(spy.logged_count, 1U, "exactly one frame for case %u", (unsigned)i);
        zassert_equal(data_frames_logged(), 0U, "no measurement may accompany a fault");
        zassert_equal(seq.calls, 0U, "a diagnostic belongs to no acquisition, so it draws no number");

        wire::health_frame hf{};
        zassert_equal(wire::decode_health(spy.log[0].bytes, wire::kFrameBytes, hf),
                      wire::result::ok, "decodes");
        zassert_equal(hf.purpose, wire::health_purpose::independent_diagnostic,
                      "case %u must be independent, or it would rewrite a grid's health",
                      (unsigned)i);
        zassert_equal(hf.sequence, wire::kReservedSequence, "and must name no acquisition");
        zassert_equal(hf.state, health::sensor_state::error, "case %u is ERROR", (unsigned)i);
        /* Only "never started" is an availability statement; a read that failed is a sensor that
         * is there and not answering, which is a different thing. */
        zassert_equal(hf.inactive, cases[i] == health::acquisition::never_started,
                      "case %u inactive flag", (unsigned)i);
    }
}

ZTEST(tof_package_stream, test_a_failed_acquisition_with_a_grid_in_hand_sends_no_data)
{
    /* THE DANGEROUS SHAPE, and the reason the other no-acquisition cases are not enough: here the
     * caller is holding 64 perfectly good zones from a moment ago and reporting that THIS
     * acquisition failed. Every byte needed to manufacture a grid is present and correctly sized,
     * so nothing stops the module from sending one except the rule itself. A module that leaked it
     * would put last moment's measurement on the bus under a number of its own -- which is the one
     * failure a consumer cannot detect, because the frames would be flawless. */
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid stale{};
    fill(stale, 7000);

    stream::input in{};
    in.outcome = health::acquisition::failed;
    in.distance_mm = stale.distance_mm;
    in.status = stale.status;
    in.zone_count = wire::kZones;

    stream::outcome o{};
    zassert_equal(p.publish(in, &o), 0, "publish");
    zassert_equal(o, stream::outcome::diagnostic_published, "%s", stream::outcome_name(o));

    zassert_equal(data_frames_logged(), 0U, "a failed acquisition sent the grid it was holding");
    zassert_equal(spy.data_calls, 0U, "the data sink must not be called at all");
    zassert_equal(spy.logged_count, 1U, "exactly one diagnostic");
    zassert_equal(seq.calls, 0U, "and no number may be drawn for it");

    wire::health_frame hf{};
    zassert_equal(wire::decode_health(spy.log[0].bytes, wire::kFrameBytes, hf), wire::result::ok,
                  "decodes");
    zassert_equal(hf.purpose, wire::health_purpose::independent_diagnostic, "independent");
    zassert_equal(hf.sequence, wire::kReservedSequence, "names no acquisition");
}

ZTEST(tof_package_stream, test_a_degraded_grid_still_publishes_with_its_raw_status)
{
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    g.status[7] = 4;  // neither valid nor "no target": one abnormal zone is enough for DEGRADED

    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish");
    zassert_equal(o, stream::outcome::grid_published, "degraded publishes: %s",
                  stream::outcome_name(o));

    wire::health_frame meta{};
    zassert_equal(wire::decode_health(spy.log[wire::kFramesPerGrid - 1].bytes, wire::kFrameBytes,
                                      meta),
                  wire::result::ok, "metadata");
    zassert_equal(meta.state, health::sensor_state::degraded, "one abnormal zone is DEGRADED");

    wire::data_frame df{};
    zassert_equal(wire::decode_data(spy.log[7].bytes, wire::kFrameBytes, df), wire::result::ok,
                  "zone 7");
    zassert_equal(df.status, 4, "the raw status must travel untouched");
}

ZTEST(tof_package_stream, test_a_malformed_complete_grid_is_the_callers_defect)
{
    test_grid g{};
    fill(g);

    stream::input missing_distance{grid_input(g)};
    missing_distance.distance_mm = nullptr;
    stream::input missing_status{grid_input(g)};
    missing_status.status = nullptr;
    stream::input wrong_count{grid_input(g)};
    wrong_count.zone_count = wire::kZones - 1;

    /* Ordered so that the two cases a mutation can be CAUGHT on come first. Dropping the guard
     * makes `missing_distance` dereference null and take the whole suite down, and a crash is weaker
     * evidence than an assertion: the case never reaches its check. The other two stay refusable by
     * inspection, so they fail loudly instead. */
    const stream::input cases[]{wrong_count, missing_status, missing_distance};

    for (size_t i{0}; i < sizeof cases / sizeof cases[0]; ++i) {
        reset_all();
        stream::publisher p{};
        zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

        stream::outcome o{};
        zassert_equal(p.publish(cases[i], &o), -EINVAL, "case %u must be refused", (unsigned)i);
        zassert_equal(o, stream::outcome::bad_argument, "%s", stream::outcome_name(o));
        /* NOT a diagnostic: blaming the sensor for the caller's buffer would put a sensor fault on
         * the bus for a software defect. */
        zassert_equal(spy.logged_count, 0U, "case %u sent something", (unsigned)i);
        zassert_equal(seq.calls, 0U, "and must draw no number");
    }
}

ZTEST(tof_package_stream, test_a_stopped_stream_keeps_reporting_and_sends_no_measurement)
{
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "the one good grid");
    zassert_equal(o, stream::outcome::grid_published, "%s", stream::outcome_name(o));

    const uint32_t published{p.current().last_sequence};
    const size_t data_before{data_frames_logged()};

    /* Acquisition stops. The caller keeps reporting, which is the whole of "diagnostics continue
     * while raw stops": there is no flush, no final grid and no repeat of the last one. */
    stream::input stopped{};
    stopped.outcome = health::acquisition::never_started;
    for (int i{0}; i < 10; ++i) {
        zassert_equal(p.publish(stopped, &o), 0, "report %d", i);
        zassert_equal(o, stream::outcome::diagnostic_published, "%s", stream::outcome_name(o));
    }

    zassert_equal(data_frames_logged(), data_before, "a stopped stream sent a measurement");
    zassert_equal(p.current().diagnostics_sent, 10U, "ten reports");
    zassert_equal(p.current().grids_published, 1U, "still one grid");

    /* And not one of those reports names the published grid, so none of them can rewrite its
     * health after a consumer has acted on it. */
    for (size_t i{0}; i < spy.logged_count; ++i) {
        if (spy.log[i].is_data)
            continue;
        wire::health_frame hf{};
        zassert_equal(wire::decode_health(spy.log[i].bytes, wire::kFrameBytes, hf),
                      wire::result::ok, "frame %u", (unsigned)i);
        if (hf.purpose == wire::health_purpose::grid_metadata)
            continue;
        zassert_equal(hf.sequence, wire::kReservedSequence,
                      "diagnostic %u names acquisition %u", (unsigned)i, hf.sequence);
        zassert_not_equal(hf.sequence, published, "a diagnostic claimed the published grid");
    }
}

ZTEST(tof_package_stream, test_a_failed_diagnostic_is_reported_and_counted)
{
    reset_all();
    spy.fail_health = true;

    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    stream::input in{};
    in.outcome = health::acquisition::failed;
    stream::outcome o{};
    zassert_equal(p.publish(in, &o), 0, "publish");
    zassert_equal(o, stream::outcome::diagnostic_failed, "%s", stream::outcome_name(o));
    zassert_equal(p.current().diagnostics_sent, 0U, "a failed send is not a report");
    zassert_equal(p.current().send_failures, 1U, "counted as a send failure");
}

ZTEST(tof_package_stream, test_reinit_starts_a_new_streams_counts)
{
    reset_all();
    stream::publisher p{};
    zassert_equal(p.init(wired_sink(), wired_source()), 0, "init");

    test_grid g{};
    fill(g);
    stream::outcome o{};
    zassert_equal(p.publish(grid_input(g), &o), 0, "publish");
    zassert_equal(p.current().grids_published, 1U, "one");

    zassert_equal(p.init(wired_sink(), wired_source()), 0, "re-init");
    zassert_equal(p.current().grids_published, 0U, "a re-init is a new stream");
    zassert_equal(p.current().last_sequence, wire::kReservedSequence,
                  "and has drawn no number yet");
}
