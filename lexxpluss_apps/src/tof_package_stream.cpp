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

#include "tof_package_stream.hpp"

#include <errno.h>

namespace lexxhard::tof_package_stream {

namespace {

namespace wire = tof_package_wire_n;
namespace health = tof_package_health;

}  // namespace

int publisher::init(const sink &s, const sequence_source &seq)
{
    /* Cleared before the check, so a refused init leaves an object that publishes nothing rather
     * than one still wired to whatever was passed last time. */
    initialised_ = false;

    if (s.send_data == nullptr || s.send_health == nullptr || seq.issue == nullptr)
        return -EINVAL;

    sink_ = s;
    seq_ = seq;
    /* A re-init is a new stream, so the counts start again. Keeping them would make
     * `grids_published` a lifetime figure for an object whose sink has changed, and an operator
     * reading "12 grids published" would have no way to know which bus they went to. */
    counters_ = counters{};
    initialised_ = true;
    return 0;
}

int publisher::send_diagnostic(const health::verdict &v, outcome *result)
{
    wire::health_frame hf{};
    hf.sequence = wire::kReservedSequence;
    hf.state = v.state;
    hf.purpose = wire::health_purpose::independent_diagnostic;
    hf.inactive = v.avail == health::availability::inactive;

    uint8_t frame[wire::kFrameBytes]{};
    if (wire::encode_health(hf, frame, sizeof frame) != wire::result::ok) {
        /* Unreachable: the sequence is the reserved one by construction, the purpose is a literal,
         * and the state comes from the health rule's own enum. Handled rather than asserted because
         * an encoder refusal that did happen must not be reported as a frame that went out. */
        *result = outcome::diagnostic_failed;
        return 0;
    }

    counters_.last_state = v.state;
    if (sink_.send_health(sink_.ctx, frame, sizeof frame) != 0) {
        ++counters_.send_failures;
        *result = outcome::diagnostic_failed;
        return 0;
    }

    ++counters_.diagnostics_sent;
    *result = outcome::diagnostic_published;
    return 0;
}

int publisher::publish(const input &in, outcome *result)
{
    if (result == nullptr)
        return -EINVAL;

    if (!initialised_) {
        *result = outcome::not_configured;
        return -EPERM;
    }

    /* Checked here and not handed to the health rule as `structurally_invalid`, because that value
     * means the SENSOR produced something malformed. A caller that passes a null array or the wrong
     * zone count has a software defect, and reporting it as a sensor fault would send a diagnostic
     * blaming the device. */
    if (in.outcome == acquisition::complete_grid &&
        (in.distance_mm == nullptr || in.status == nullptr || in.zone_count != kZones)) {
        *result = outcome::bad_argument;
        return -EINVAL;
    }

    health::input hi{};
    hi.outcome = in.outcome;
    hi.zone_status = in.status;
    hi.zone_count = in.zone_count;
    const health::verdict v{health::judge(hi)};

    /* Health first, before a number exists -- see the header on why a refused grid must not spend
     * one. */
    if (!v.publish_grid)
        return send_diagnostic(v, result);

    uint32_t sequence{0};
    if (seq_.issue(seq_.ctx, &sequence) != 0) {
        ++counters_.sequence_failures;
        *result = outcome::sequence_unavailable;
        return 0;
    }

    /* Recorded as soon as it is drawn, not once the grid succeeds: the number is spent either way,
     * and an operator diagnosing an abandoned grid needs to know which number it took with it. */
    counters_.last_sequence = sequence;

    for (size_t zone{0}; zone < kZones; ++zone) {
        wire::data_frame df{};
        df.sequence = sequence;
        df.zone = static_cast<uint8_t>(zone);
        df.distance_mm = in.distance_mm[zone];
        df.status = in.status[zone];

        uint8_t frame[wire::kFrameBytes]{};
        if (wire::encode_data(df, frame, sizeof frame) != wire::result::ok) {
            /* The only reachable cause is a source that handed back the reserved number, which the
             * layout refuses end to end. It fails on zone 0, so nothing of this grid is on the bus
             * -- which is why this is not grid_abandoned. */
            *result = outcome::sequence_refused_by_layout;
            return 0;
        }

        if (sink_.send_data(sink_.ctx, frame, sizeof frame) != 0) {
            /* No retry and no queue. The grid is gone with its number; see the header. */
            ++counters_.send_failures;
            ++counters_.grids_abandoned;
            *result = outcome::grid_abandoned;
            return 0;
        }
    }

    wire::health_frame meta{};
    meta.sequence = sequence;
    meta.state = v.state;
    meta.purpose = wire::health_purpose::grid_metadata;
    meta.inactive = false;

    uint8_t frame[wire::kFrameBytes]{};
    if (wire::encode_health(meta, frame, sizeof frame) != wire::result::ok) {
        /* Unreachable by the time we are here: reaching this line means 64 data frames encoded
         * with this sequence, so it is not the reserved one, and the state and purpose are as
         * constrained as in send_diagnostic. The grid is still counted as abandoned, because 64
         * frames ARE on the bus with no metadata to close them. */
        ++counters_.grids_abandoned;
        *result = outcome::grid_abandoned;
        return 0;
    }

    counters_.last_state = v.state;
    if (sink_.send_health(sink_.ctx, frame, sizeof frame) != 0) {
        ++counters_.send_failures;
        ++counters_.grids_abandoned;
        *result = outcome::grid_abandoned;
        return 0;
    }

    ++counters_.grids_published;
    *result = outcome::grid_published;
    return 0;
}

const char *outcome_name(outcome o)
{
    switch (o) {
    case outcome::grid_published:             return "grid_published";
    case outcome::grid_abandoned:             return "grid_abandoned";
    case outcome::sequence_unavailable:       return "sequence_unavailable";
    case outcome::sequence_refused_by_layout: return "sequence_refused_by_layout";
    case outcome::diagnostic_published:       return "diagnostic_published";
    case outcome::diagnostic_failed:          return "diagnostic_failed";
    case outcome::not_configured:             return "not_configured";
    case outcome::bad_argument:               return "bad_argument";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_package_stream
