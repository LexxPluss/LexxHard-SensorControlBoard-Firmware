/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_cliff_packer.hpp"

#include <errno.h>

namespace tof_cliff_packer {

namespace {

using namespace tof_cliff_contract;

/* The ULD's "no target detected, no device error" status. Named here rather than
 * written as 255 at three call sites, because the whole zero-target rule turns on it. */
constexpr uint8_t kStatusNone{255};

const status_row *classify(uint8_t raw)
{
    for (size_t i = 0; i < kStatusRowCount; ++i)
        if (kStatusTable[i].raw == raw)
            return &kStatusTable[i];
    return nullptr;
}

reduction refuse(reason why, uint8_t observed)
{
    reduction r{};
    r.outcome = result::unencodable;
    r.why = why;
    r.error = -EPROTO;
    r.observed_status = observed;
    return r;
}

} // namespace

reduction reduce(const struct tof_cliff_sample &sample)
{
    /* No new sample was ready. The entries then hold whatever the previous read left
     * behind, and packing them would transmit a stale distance as a fresh one -- the
     * one thing the contract forbids absolutely ("old values are never re-sent"). This
     * is an ordinary outcome, not an error: nothing was produced this cycle. */
    if (!sample.fresh) {
        reduction r{};
        r.outcome = result::no_frame;
        return r;
    }

    /* The read layer already rejects a count above the array, so reaching either of
     * these means something upstream changed. Refusing loudly beats trusting it. */
    if (sample.target_count > TOF_CLIFF_MAX_TARGETS)
        return refuse(reason::target_count_malformed, 0);
    if (sample.entry_count == 0 || sample.entry_count > TOF_CLIFF_MAX_TARGETS)
        return refuse(reason::no_entries, 0);

    /* Zero targets is exact, not lenient. The ULD writes RangeData[0] with status 255
     * when active_results == 0, and that is the only shape the wire can carry: the
     * bidirectional invariant ties target_count 0 to status 255 and the sentinel. Any
     * other combination is impossible metadata, and rewriting it to 255 would hide an
     * anomaly in the read layer or the ULD behind an ordinary no-target frame. */
    if (sample.target_count == 0) {
        const uint8_t observed{sample.entries[0].range_status};
        if (sample.entry_count != 1)
            return refuse(reason::zero_targets_entry_count_not_one, observed);
        if (observed != kStatusNone)
            return refuse(reason::zero_targets_with_unexpected_status, observed);

        reduction r{};
        r.outcome = result::frame_ready;
        r.cls = status_class::no_target;
        r.range_mm = kSentinelInvalid;
        r.raw_status = kStatusNone;
        r.target_count = 0;
        r.observed_status = kStatusNone;
        return r;
    }

    /* With targets present, entry_count must equal target_count -- that is what the read
     * layer documents. Trusting target_count while the array is shorter would classify
     * entries nobody filled, and whatever is in them would look like device data. */
    if (sample.entry_count != sample.target_count)
        return refuse(reason::entry_count_mismatch, sample.entries[0].range_status);

    /* Classify every target first. An unclassifiable status cannot be reduced to a
     * safety outcome at all, so it refuses the whole measurement rather than being
     * guessed at. */
    const status_row *rows[TOF_CLIFF_MAX_TARGETS]{};
    for (uint8_t i = 0; i < sample.target_count; ++i) {
        /* Status 255 means "no target detected", so it cannot appear alongside targets.
         * Reducing it here would build status 255 with a non-zero target_count, which
         * breaks the wire's bidirectional invariant and the decoder must reject -- so
         * the frame would be discarded after transmission instead of diagnosed here. */
        if (sample.entries[i].range_status == kStatusNone)
            return refuse(reason::none_status_among_targets, kStatusNone);
        rows[i] = classify(sample.entries[i].range_status);
        if (rows[i] == nullptr)
            return refuse(reason::status_undefined, sample.entries[i].range_status);
    }

    /* The surviving class is the highest-priority one present. Scanning the priority
     * order -- rather than sorting the targets -- is required, not just convenient: the
     * transmitted status is selected by ULD index, so the array order must survive. */
    status_class surviving{status_class::valid_range};
    bool found{false};
    for (const status_class candidate : kReductionPriority) {
        for (uint8_t i = 0; i < sample.target_count && !found; ++i)
            if (rows[i]->cls == candidate) {
                surviving = candidate;
                found = true;
            }
        if (found)
            break;
    }

    reduction r{};
    r.cls = surviving;
    r.target_count = sample.target_count;

    /* A NO_SAMPLE status surviving means no frame at all. This is a correct outcome
     * rather than a failure, so `error` stays 0 -- the source's sample_produced bit
     * simply stays clear for this cycle. */
    if (surviving == status_class::no_sample) {
        r.outcome = result::no_frame;
        return r;
    }

    if (surviving == status_class::valid_range) {
        /* Farthest, not nearest: here the hazard is the floor being farther than
         * expected, so a spurious near return must never mask a real drop behind it.
         * Strict `>` keeps the lowest-indexed target when two are equally far, which
         * matches the lowest-index rule used for every other class. */
        uint8_t best{0};
        bool have_best{false};
        for (uint8_t i = 0; i < sample.target_count; ++i) {
            if (rows[i]->cls != status_class::valid_range)
                continue;
            if (!have_best || sample.entries[i].range_mm > sample.entries[best].range_mm) {
                best = i;
                have_best = true;
            }
        }
        const int16_t mm{sample.entries[best].range_mm};
        /* The read layer keeps the range signed and unclamped on purpose, so a negative
         * value reaches here intact. It cannot be encoded, and the ULD is supposed to
         * have flagged it as status 14 -- so seeing it under a VALID_RANGE class means
         * the two disagree, which is a defect rather than a distance. */
        if (mm < 0)
            return refuse(reason::valid_range_negative, sample.entries[best].range_status);
        if (static_cast<uint16_t>(mm) == kSentinelInvalid)
            return refuse(reason::valid_range_is_sentinel, sample.entries[best].range_status);

        r.range_mm = static_cast<uint16_t>(mm);
        r.raw_status = sample.entries[best].range_status;
    } else {
        /* NO_TARGET and SENSOR_FAULT both carry the sentinel and the raw status of the
         * lowest-indexed target in the surviving class -- the first target that
         * condemned the measurement. */
        for (uint8_t i = 0; i < sample.target_count; ++i)
            if (rows[i]->cls == surviving) {
                r.raw_status = sample.entries[i].range_status;
                break;
            }
        r.range_mm = kSentinelInvalid;
    }

    r.outcome = result::frame_ready;
    r.observed_status = r.raw_status;
    return r;
}

namespace {

/* Re-derives the wire invariants from the reduction itself.
 *
 * reduce() cannot produce an inconsistent reduction, but nothing stops a caller
 * building one by hand -- the struct is public so tests and diagnostics can read it.
 * Without this, a hand-assembled reduction would encode into a frame the decoder is
 * obliged to reject, and the defect would surface as a discarded frame on the other
 * side of the bus instead of a refusal here. Cheap defence in depth against a
 * programming error, not against the device. */
bool wire_consistent(const reduction &r)
{
    if (r.target_count > TOF_CLIFF_MAX_TARGETS)
        return false;
    const status_row *row{classify(r.raw_status)};
    if (row == nullptr || row->cls != r.cls)
        return false;
    if (r.cls == status_class::no_sample)
        return false; // never transmitted
    if (r.cls == status_class::valid_range) {
        if (r.range_mm == kSentinelInvalid)
            return false;
    } else if (r.range_mm != kSentinelInvalid) {
        return false;
    }
    /* target_count == 0 if and only if status 255 with the sentinel. */
    const bool zero_targets{r.target_count == 0};
    const bool encoded_none{r.raw_status == kStatusNone && r.range_mm == kSentinelInvalid};
    return zero_targets == encoded_none;
}

} // namespace

bool encode_measurement(const reduction &r, uint8_t source_id, uint8_t mapping_epoch,
                        uint8_t cycle_seq, uint8_t out[8], reason *why)
{
    if (why != nullptr)
        *why = reason::none;

    /* Every rejection happens before the first store, so a refused encode leaves `out`
     * exactly as the caller left it. A half-written payload looks like a frame to
     * anything that forgets to check the return value. */
    if (r.outcome != result::frame_ready)
        return false;
    if (source_id >= kSourceCount) {
        if (why != nullptr)
            *why = reason::source_id_out_of_range;
        return false;
    }
    if (!wire_consistent(r)) {
        if (why != nullptr)
            *why = reason::reduction_inconsistent;
        return false;
    }

    out[0] = static_cast<uint8_t>((kMeasFrameType << 4) | (source_id & 0x0F));
    out[1] = mapping_epoch;
    out[2] = cycle_seq;
    out[3] = static_cast<uint8_t>(r.range_mm >> 8);
    out[4] = static_cast<uint8_t>(r.range_mm & 0xFF);
    out[5] = r.raw_status;
    out[6] = r.target_count;
    out[7] = 0; // reserved; a future capture tick here is a version bump
    return true;
}

bool encode_health(const health_fields &h, uint8_t out[8])
{
    if (h.mapping_state > 0x3)
        return false;
    if (h.failing_chain_position != kChainPositionNone &&
        !(h.failing_chain_position >= 1 && h.failing_chain_position <= 6))
        return false;

    /* Every one of these occupies a nibble on the wire. Masking a wider value would
     * silently turn a producer error into a *different* legal health frame -- 0x1F
     * becoming 0x0F reports three sensors enumerated as four. Refusing names the bug on
     * the side that made it. */
    if (h.flags > 0x0F || h.enumerated_mask > 0x0F || h.model_verified_mask > 0x0F ||
        h.sample_produced_mask > 0x0F || h.sensor_fault_mask > 0x0F)
        return false;

    const uint8_t enumerated{h.enumerated_mask};
    const uint8_t model_verified{h.model_verified_mask};
    const uint8_t produced{h.sample_produced_mask};
    const uint8_t fault{h.sensor_fault_mask};
    const uint8_t flags{h.flags};

    /* With cycle_valid clear the frame describes no cycle, so both per-cycle fields
     * must be empty -- sample_produced_mask and sensor_fault_mask alike. */
    if (!(flags & kCycleValidBit) && (h.cycle_seq != 0 || produced != 0 || fault != 0))
        return false;
    /* sensor_fault_mask classifies a produced sample, so a fault bit outside the
     * produced mask would classify a sample that does not exist. */
    if (fault & static_cast<uint8_t>(~produced) & 0x0F)
        return false;
    /* Naming a failing position with no chain fault and complete masks is
     * contradictory; an incomplete enumeration is itself a reason to name one. */
    if (h.failing_chain_position != kChainPositionNone && !(flags & kChainFaultBits) &&
        enumerated == 0x0F && model_verified == 0x0F)
        return false;

    out[0] = static_cast<uint8_t>((kHealthFrameType << 4) | (kProtocolVersion & 0x0F));
    out[1] = h.mapping_epoch;
    out[2] = h.health_seq;
    out[3] = static_cast<uint8_t>((h.mapping_state << 4) | flags);
    out[4] = static_cast<uint8_t>((enumerated << 4) | model_verified);
    out[5] = static_cast<uint8_t>((produced << 4) | fault);
    out[6] = h.failing_chain_position;
    out[7] = h.cycle_seq;
    return true;
}

} // namespace tof_cliff_packer
