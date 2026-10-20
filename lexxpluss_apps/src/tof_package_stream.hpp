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

#pragma once

/* Publish one acquisition without retaining its arrays or retrying a failed send.
 * An abandoned grid consumes its number; the next acquisition takes a new one.
 *
 * Order: validate health before spending a sequence, send zones 0..63, then
 * metadata. Data order and metadata-last are producer choices; receivers allow
 * reordering. Refused health must not consume finite journal capacity.
 *
 * No fresh acquisition produces one independent diagnostic (sequence zero),
 * never a synthetic grid or a refreshed timestamp on old data. Diagnostics
 * continue only while the caller supplies non-fresh outcomes; there is no flush.
 * The caller supplies the sink and sequence source. No CAN IDs, clock, vendor
 * headers, retained grid or allocation belong to this module.
 */

#include <stddef.h>
#include <stdint.h>

#include "tof_package_health.hpp"
#include "tof_package_wire_n.hpp"

namespace lexxhard::tof_package_stream {

using tof_package_health::acquisition;
using tof_package_health::sensor_state;

inline constexpr size_t kZones{tof_package_wire_n::kZones};

/* One acquisition as the caller sees it. The arrays are READ inside publish() and never retained;
 * see the header comment on why this module cannot keep them. */
struct input {
    /* What the acquisition layer says happened. `complete_grid` is the only value that can produce
     * data frames, and it is a claim this module checks rather than trusts: the zone arrays have to
     * be there and be exactly kZones long. */
    acquisition outcome{acquisition::failed};
    /* kZones entries each, required when `outcome` is complete_grid. Raw: no clamp, no validity
     * judgement, no status rewrite. */
    const uint16_t *distance_mm{nullptr};
    const uint8_t *status{nullptr};
    size_t zone_count{0};
};

struct sink {
    /* Each takes one already-encoded frame of tof_package_wire_n::kFrameBytes. 0, or a negative
     * errno. The two are separate function pointers rather than one with an identifier argument,
     * because the only thing this module knows is WHICH KIND of frame it made -- putting an
     * identifier in the signature would mean inventing one here. */
    int (*send_data)(void *ctx, const uint8_t *frame, size_t frame_len){nullptr};
    int (*send_health)(void *ctx, const uint8_t *frame, size_t frame_len){nullptr};
    void *ctx{nullptr};
};

struct sequence_source {
    /* 0 with *out written, or a negative errno with *out untouched. The contract this module
     * depends on and cannot verify: never the same number twice, and never
     * tof_package_wire_n::kReservedSequence. A source that breaks either one produces frames that
     * are well formed and wrong, so it is named here as a precondition rather than defended
     * against -- the defence belongs in the source (tof_seq_issuer), which can see its own journal.
     */
    int (*issue)(void *ctx, uint32_t *out){nullptr};
    void *ctx{nullptr};
};

/* What one publish() call did. Every value is an observation about THIS call: nothing here is
 * remembered for the next one. */
enum class outcome : uint8_t {
    /* 64 data frames and the metadata frame were all accepted by the sink. */
    grid_published,
    /* A data or metadata send failed. The grid is gone, its sequence is spent, and no part of it
     * will be sent again. Some frames are already on the bus; the consumer refuses the grid. */
    grid_abandoned,
    /* The sequence source refused. NOTHING was sent -- not even a diagnostic, because a diagnostic
     * here would report the sensor's health when what failed was the issuer, and the caller is the
     * layer that knows the difference. */
    sequence_unavailable,
    /* The source returned a number the wire layout will not carry -- in practice
     * tof_package_wire_n::kReservedSequence, which is the diagnostic's placeholder. Nothing was
     * sent, and it is kept apart from grid_abandoned because the two point at different components:
     * this one says the SOURCE is wrong, and no amount of bus health will fix it. Reachable only
     * from a source that breaks its own contract, which is exactly why it is not an assertion. */
    sequence_refused_by_layout,
    /* Exactly one independent diagnostic went out: either there was no acquisition, or the health
     * rule refused to publish the one there was. */
    diagnostic_published,
    /* The diagnostic's send failed. */
    diagnostic_failed,
    /* init() has not succeeded. */
    not_configured,
    /* A complete_grid whose arrays are missing or not kZones long, or a null `out`. Reported
     * instead of being passed to the health rule as a structural fault, because a caller that got
     * its own buffer wrong is a software defect and not a sensor reading. */
    bad_argument,
};

/* Counts, for an operator reading a stream that is or is not working. Diagnostics, not gates. */
struct counters {
    uint32_t grids_published{0};
    uint32_t grids_abandoned{0};
    uint32_t diagnostics_sent{0};
    uint32_t sequence_failures{0};
    uint32_t send_failures{0};
    /* The last number this module attached to a grid, published or abandoned. Zero means none has
     * been drawn, which kReservedSequence makes unambiguous: no acquisition can ever carry it. */
    uint32_t last_sequence{0};
    /* The health state of the last frame sent, grid metadata or diagnostic. */
    sensor_state last_state{sensor_state::error};
};

class publisher {
public:
    /* -EINVAL unless both sink callbacks and the sequence source's issue callback are present. A
     * partially wired sink is refused rather than used for the half that works: a stream whose data
     * frames go out and whose metadata does not produces 64 orphans per acquisition, which the
     * consumer refuses, so it would look like a bus fault rather than a wiring mistake. */
    int init(const sink &s, const sequence_source &seq);

    /* Publishes one acquisition. Writes `result` and returns 0, or returns a negative errno when
     * the call could not be attempted at all (not configured, bad argument) -- in which case
     * `result` still says which. */
    int publish(const input &in, outcome *result);

    counters current() const { return counters_; }

private:
    /* The one place a health frame with no acquisition is built, used both when the caller had
     * nothing fresh and when the health rule refused the grid it did have. Takes the verdict rather
     * than the acquisition so the two cases cannot drift into reporting different states for the
     * same judgement. */
    int send_diagnostic(const tof_package_health::verdict &v, outcome *result);

    sink sink_{};
    sequence_source seq_{};
    bool initialised_{false};
    counters counters_{};
};

const char *outcome_name(outcome o);

}
