/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * Turns one VL53L4CX read into one cliff measurement frame, applying the wire
 * contract's normative reduction.
 *
 * The reduction is contract-owned rather than an implementation choice, because two
 * packers reducing differently would disagree about the floor while both passing their
 * own tests. This file implements it and nothing else: it does not decide cliff
 * verdicts, does not filter over time, does not track cycles, and does not know what
 * READY means. It also never invents data -- there is no path here that fabricates a
 * distance, reuses a previous one, or turns an unrepresentable input into a plausible
 * frame.
 *
 * WHAT IT REFUSES, AND WHY REFUSING IS THE SAFE DIRECTION
 *
 * Several inputs cannot be encoded at all. The wire ties `target_count == 0` to status
 * 255 and the invalid sentinel, so a read that found nothing while reporting some other
 * status has no representation; likewise a status with no row in the classification
 * table cannot be reduced to a safety outcome. In every such case this layer produces
 * NO frame and says why. That is fail-safe by construction: no frame leaves the
 * source's `sample_produced` bit clear, the missing sample is visible in health, and the
 * freshness budget eventually faults the source. Emitting a "reasonable" frame instead
 * would convert a defect into ordinary-looking data, which is the silent failure this
 * whole design exists to remove.
 *
 * The status classification table and the reduction priority come from
 * tof_cliff_contract.h, generated from the contract. They are deliberately not
 * duplicated here.
 */

#include <cstdint>

#include "tof_cliff_contract.h"
#include "tof_cliff_sample.h" // deliberately not tof_cliff_sensor.h: no ULD here

/* The same four, defined in three places that must agree: the vendor's
 * VL53LX_MAX_RANGE_RESULTS, our TOF_CLIFF_MAX_TARGETS, and the wire contract's
 * kMaxTargets. tof_cliff_sensor.h checks the first pair where the vendor header is
 * visible; this checks the second, here, because this is the first translation unit that
 * sees our constant and the contract's together. Chained, they pin all three -- without
 * making every includer of the sensor header depend on the contract.
 *
 * kMaxTargets is NOT kSourceCount. One is how many returns a sensor can find, the other
 * how many sensors the chain carries. Both are 4 and neither implies the other, which is
 * exactly why using one for the other survives review. */
static_assert(TOF_CLIFF_MAX_TARGETS == tof_cliff_contract::kMaxTargets,
              "the wire contract's kMaxTargets and the ULD's target array have drifted");

namespace tof_cliff_packer {

enum class result : uint8_t {
    frame_ready, // a measurement frame must be transmitted
    no_frame,    // a NO_SAMPLE status survived the reduction: correct to send nothing
    unencodable, // the input has no representation on the wire; see `why`
};

/* Why an input was refused. Distinct values on purpose: "the packer was handed
 * something impossible" and "the device reported something we cannot encode" need
 * different follow-up, and collapsing them would hide which one happened. */
enum class reason : uint8_t {
    none,
    zero_targets_with_unexpected_status,
    zero_targets_entry_count_not_one,
    entry_count_mismatch,
    none_status_among_targets,
    valid_range_negative,
    valid_range_is_sentinel,
    status_undefined,
    target_count_malformed,
    no_entries,
    source_id_out_of_range,
    /* encode_measurement rejected a reduction that reduce() could not have produced.
     * Reaching this means a caller built one by hand, so it names a programming error
     * rather than a device anomaly. */
    reduction_inconsistent,
};

struct reduction {
    result outcome{result::unencodable};
    reason why{reason::none};
    /* Handed back to the acquisition layer: 0, or -EPROTO for metadata the device
     * cannot legally have produced. What that error then does to readiness and health
     * is the acquisition layer's decision, not this layer's. */
    int error{0};
    tof_cliff_contract::status_class cls{tof_cliff_contract::status_class::no_sample};
    uint16_t range_mm{tof_cliff_contract::kSentinelInvalid}; // already wire-encoded
    uint8_t raw_status{0};
    uint8_t target_count{0};
    /* The status actually observed, kept even when the input is refused. The whole
     * point of refusing is that this value must reach a log rather than be quietly
     * rewritten to 255 -- rewriting it would hide an anomaly in the read layer or in
     * the ULD behind an ordinary-looking no-target frame. */
    uint8_t observed_status{0};
};

/* Applies the contract's reduction. Pure: reads the sample, touches nothing else.
 *
 * The zero-target case is exact rather than lenient. `target_count == 0` is legal only
 * as `entry_count == 1` with `entries[0].range_status == 255`, which is what the ULD
 * produces when `active_results == 0`. Anything else is impossible metadata: -EPROTO,
 * no frame, and the observed status preserved. This is enforcement of the wire's
 * existing bidirectional invariant, not a new protocol rule. */
reduction reduce(const struct tof_cliff_sample &sample);

/* Writes the 8 measurement bytes. Returns false unless `r.outcome` is frame_ready, so
 * a caller that ignores the reduction's verdict cannot accidentally transmit.
 *
 * On false, `out` is not written at all -- not even partially. A half-filled payload is
 * worse than no payload: it looks like a frame to anything that forgets to check the
 * return value.
 *
 * The wire invariants are re-derived from `r` before anything is written. reduce()
 * cannot produce an inconsistent reduction, but the struct is public so a caller can
 * build one by hand, and such a frame would be rejected by the decoder rather than
 * refused here -- turning a programming error into a discarded frame on the far side of
 * the bus. `why` is optional and reports which check refused. */
bool encode_measurement(const reduction &r, uint8_t source_id, uint8_t mapping_epoch,
                        uint8_t cycle_seq, uint8_t out[8], reason *why = nullptr);

struct health_fields {
    uint8_t mapping_epoch{0};
    uint8_t health_seq{0};
    uint8_t mapping_state{0}; // tof_cliff_contract mapping_state values, 0x0-0x3
    uint8_t flags{0};         // bits 0-2 chain faults, bit 3 cycle_valid
    uint8_t enumerated_mask{0};
    uint8_t model_verified_mask{0};
    uint8_t sample_produced_mask{0};
    uint8_t sensor_fault_mask{0};
    uint8_t failing_chain_position{tof_cliff_contract::kChainPositionNone};
    uint8_t cycle_seq{0};
};

/* Writes the 8 health bytes. Returns false when the fields contradict the contract --
 * a per-cycle mask set with cycle_valid clear, a fault bit outside the produced mask, a
 * malformed mapping_state or chain position, or any nibble field given a value wider
 * than four bits. Refusing to encode is deliberate: those combinations are producer
 * defects, and a decoder would reject them anyway, so catching them here names the bug
 * on the side that caused it. Masking a too-wide nibble would be worse than refusing,
 * because it produces a different *legal* frame -- an enumerated mask of 0x1F becoming
 * 0x0F reports three sensors as four. On false, `out` is untouched. */
bool encode_health(const health_fields &h, uint8_t out[8]);

} // namespace tof_cliff_packer
