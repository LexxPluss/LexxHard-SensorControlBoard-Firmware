/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* GENERATED FILE -- do not edit. Regenerate with:
 *     docs/can/gen_cliff_golden_vectors.py --emit
 *
 * Contract : tof_cliff_wire_contract.md
 * Version  : commissioning-2026-08-18b
 * SHA-256  : 4b3dae652d73e6eabf17d721a90b06371effbb3aaefb3d25a2218bb860dd47f7
 * Profile  : commissioning-cliff-only-400k
 *
 * COMMISSIONING ONLY -- RELEASE_FORBIDDEN. Regenerate and re-pin both sides after the six-board schedule measurement; status 3/11 remain unvalidated.
 *
 * Test half: layout vectors and their expected verdicts.
 *
 * Scope: frame layout and validation verdicts only. The decoder state machine
 * (cycle assembly, retirement, staleness, event multisets) is NOT covered here,
 * and must not be inferred from these vectors.
 *
 * Zero dependencies on purpose: the SCBDriver tests have no JSON parser.
 *
 * clang-format is disabled for the whole file, starting above the include guard.
 * SCBDriver's CI reformats every .h in the repository with an explicitly named
 * style file, which overrides any directory .clang-format -- so a generated file
 * can only stay byte-identical across the two repositories by opting out here, in
 * the generator, rather than per checkout.
 */

// clang-format off
#pragma once

#include "tof_cliff_contract.h"

namespace tof_cliff_contract {

// verdict comes from the production header: one definition, both sides.
enum class frame_kind : uint8_t { measurement = 0, health = 1 };

struct vector {
    const char *name;
    frame_kind kind;
    uint8_t dlc;          // 8 for every valid frame; other values must be rejected
    uint8_t bytes[8];
    verdict expected;
    const char *why;
};

inline constexpr size_t kVectorCount{87};

inline constexpr vector kVectors[kVectorCount]{
    {"meas_role_0_front_left", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "source_id is a stable logical role, not a chain position"},
    {"meas_role_1_rear_left", frame_kind::measurement, 8, {0x11, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "source_id is a stable logical role, not a chain position"},
    {"meas_role_2_rear_right", frame_kind::measurement, 8, {0x12, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "source_id is a stable logical role, not a chain position"},
    {"meas_role_3_front_right", frame_kind::measurement, 8, {0x13, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "source_id is a stable logical role, not a chain position"},
    {"meas_status_0_valid_range", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "class VALID_RANGE; raw status is transmitted unchanged"},
    {"meas_status_1_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x01, 0x01, 0x00},
     verdict::accept, "class NO_TARGET; raw status is transmitted unchanged"},
    {"meas_status_2_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x02, 0x01, 0x00},
     verdict::accept, "class NO_TARGET; raw status is transmitted unchanged"},
    {"meas_status_3_sensor_fault", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x03, 0x01, 0x00},
     verdict::accept, "class SENSOR_FAULT; raw status is transmitted unchanged"},
    {"meas_status_4_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x04, 0x01, 0x00},
     verdict::accept, "class NO_TARGET; raw status is transmitted unchanged"},
    {"meas_status_5_sensor_fault", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x05, 0x01, 0x00},
     verdict::accept, "class SENSOR_FAULT; raw status is transmitted unchanged"},
    {"meas_status_6_no_sample", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x06, 0x01, 0x00},
     verdict::status_not_transmissible, "class NO_SAMPLE; raw status is transmitted unchanged"},
    {"meas_status_7_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x07, 0x01, 0x00},
     verdict::accept, "class NO_TARGET; raw status is transmitted unchanged"},
    {"meas_status_8_sensor_fault", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x08, 0x01, 0x00},
     verdict::accept, "class SENSOR_FAULT; raw status is transmitted unchanged"},
    {"meas_status_9_sensor_fault", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x09, 0x01, 0x00},
     verdict::accept, "class SENSOR_FAULT; raw status is transmitted unchanged"},
    {"meas_status_10_no_sample", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0a, 0x01, 0x00},
     verdict::status_not_transmissible, "class NO_SAMPLE; raw status is transmitted unchanged"},
    {"meas_status_11_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0b, 0x01, 0x00},
     verdict::accept, "class NO_TARGET; raw status is transmitted unchanged"},
    {"meas_status_12_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0c, 0x01, 0x00},
     verdict::accept, "class NO_TARGET; raw status is transmitted unchanged"},
    {"meas_status_13_sensor_fault", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0d, 0x01, 0x00},
     verdict::accept, "class SENSOR_FAULT; raw status is transmitted unchanged"},
    {"meas_status_14_sensor_fault", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0e, 0x01, 0x00},
     verdict::accept, "class SENSOR_FAULT; raw status is transmitted unchanged"},
    {"meas_status_255_no_target", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00},
     verdict::accept, "the ULD forces 8191 mm when active_results == 0; the packer MUST send the sentinel instead and never forward 8191 as a finite range"},
    {"meas_reject_valid_status_with_sentinel", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x00, 0x01, 0x00},
     verdict::range_contradicts_status, "RANGE_VALID cannot carry the invalid sentinel"},
    {"meas_reject_no_target_with_finite_range", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x02, 0x01, 0x00},
     verdict::range_contradicts_status, "a NO_TARGET class is transmitted with 0xFFFF and its real status; a finite value here would publish an untrustworthy distance as a floor"},
    {"meas_reject_merged_pulse_with_finite_range", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x0b, 0x01, 0x00},
     verdict::range_contradicts_status, "status 11 has a device-valid range, and that is exactly why it must not be forwarded: a step edge merges returns and reads as an intermediate floor"},
    {"meas_reject_sensor_fault_with_finite_range", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x05, 0x01, 0x00},
     verdict::range_contradicts_status, "a SENSOR_FAULT class has no trustworthy distance"},
    {"meas_reject_no_sample_status_6", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x06, 0x01, 0x00},
     verdict::status_not_transmissible, "the two NO_SAMPLE statuses produce no frame at all; the frame existing is itself the defect"},
    {"meas_reject_no_sample_status_10", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0a, 0x01, 0x00},
     verdict::status_not_transmissible, "the two NO_SAMPLE statuses produce no frame at all; the frame existing is itself the defect"},
    {"meas_reject_status_undefined_15", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0f, 0x01, 0x00},
     verdict::status_undefined, "a raw status outside the classification table cannot be classified, so it cannot be reduced to a safety outcome"},
    {"meas_reject_status_undefined_100", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x64, 0x01, 0x00},
     verdict::status_undefined, "a raw status outside the classification table cannot be classified, so it cannot be reduced to a safety outcome"},
    {"meas_reject_status_undefined_254", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0xfe, 0x01, 0x00},
     verdict::status_undefined, "a raw status outside the classification table cannot be classified, so it cannot be reduced to a safety outcome"},
    {"meas_range_0", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00},
     verdict::accept, "zero is encodable and is not the sentinel"},
    {"meas_range_1", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00},
     verdict::accept, "one millimetre"},
    {"meas_range_65534", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xfe, 0x00, 0x01, 0x00},
     verdict::accept, "largest encodable non-sentinel value"},
    {"meas_epoch_0", frame_kind::measurement, 8, {0x10, 0x00, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "mapping_epoch wraps 255 -> 0"},
    {"meas_epoch_1", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "mapping_epoch wraps 255 -> 0"},
    {"meas_epoch_254", frame_kind::measurement, 8, {0x10, 0xfe, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "mapping_epoch wraps 255 -> 0"},
    {"meas_epoch_255", frame_kind::measurement, 8, {0x10, 0xff, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "mapping_epoch wraps 255 -> 0"},
    {"meas_cycle_0", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_cycle_1", frame_kind::measurement, 8, {0x10, 0x01, 0x01, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_cycle_254", frame_kind::measurement, 8, {0x10, 0x01, 0xfe, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_cycle_255", frame_kind::measurement, 8, {0x10, 0x01, 0xff, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_targets_1", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::accept, "target_count is pre-reduction and diagnostic only"},
    {"meas_targets_2", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x02, 0x00},
     verdict::accept, "target_count is pre-reduction and diagnostic only"},
    {"meas_targets_3", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x03, 0x00},
     verdict::accept, "target_count is pre-reduction and diagnostic only"},
    {"meas_targets_4", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x04, 0x00},
     verdict::accept, "target_count is pre-reduction and diagnostic only"},
    {"meas_reject_frame_type", frame_kind::measurement, 8, {0x20, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::frame_type_mismatch, "a mis-routed filter is otherwise a silent mis-decode"},
    {"meas_reject_source_id_4", frame_kind::measurement, 8, {0x14, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::source_id_out_of_range, "source_id must be 0-3"},
    {"meas_reject_source_id_15", frame_kind::measurement, 8, {0x1f, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::source_id_out_of_range, "source_id must be 0-3"},
    {"meas_reject_reserved_nonzero", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x01},
     verdict::reserved_field_nonzero, "byte 7 is reserved for a future capture tick; using it is a version bump"},
    {"meas_reject_target_count_5", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x05, 0x00},
     verdict::target_count_malformed, "target_count > 4 is malformed"},
    {"meas_reject_none_with_finite_range", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0xff, 0x00, 0x00},
     verdict::range_contradicts_status, "status 255 with a finite range; the class rule catches this first"},
    {"meas_reject_none_with_targets", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x01, 0x00},
     verdict::no_target_encoding_inconsistent, "status 255 must carry target_count 0"},
    {"meas_reject_zero_targets_with_valid_status", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x00, 0x00},
     verdict::no_target_encoding_inconsistent, "target_count 0 is only legal as the status-255 encoding"},
    {"meas_reject_zero_targets_with_sentinel_not_255", frame_kind::measurement, 8, {0x10, 0x01, 0x00, 0xff, 0xff, 0x02, 0x00, 0x00},
     verdict::no_target_encoding_inconsistent, "the sentinel alone does not license target_count 0; the invariant needs status 255 as well"},
    {"meas_reject_dlc_7", frame_kind::measurement, 7, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00},
     verdict::dlc_not_8, "DLC is 8 always; a short frame must be rejected before any byte is read"},
    {"health_proven_cycle_valid", frame_kind::health, 8, {0x21, 0x01, 0x07, 0x18, 0xff, 0xf0, 0xff, 0x00},
     verdict::accept, "the ready shape: PROVEN, no chain fault, all four enumerated and produced"},
    {"health_heartbeat_unknown", frame_kind::health, 8, {0x21, 0x01, 0x08, 0x00, 0x00, 0x00, 0xff, 0x00},
     verdict::accept, "a heartbeat during UNKNOWN describes no cycle and depends on no measurement"},
    {"health_mapping_state_unknown", frame_kind::health, 8, {0x21, 0x01, 0x09, 0x08, 0xff, 0xf0, 0xff, 0x03},
     verdict::accept, "all four mapping states are encodable"},
    {"health_mapping_state_proven", frame_kind::health, 8, {0x21, 0x01, 0x09, 0x18, 0xff, 0xf0, 0xff, 0x03},
     verdict::accept, "all four mapping states are encodable"},
    {"health_mapping_state_lost", frame_kind::health, 8, {0x21, 0x01, 0x09, 0x28, 0xff, 0xf0, 0xff, 0x03},
     verdict::accept, "all four mapping states are encodable"},
    {"health_mapping_state_fault", frame_kind::health, 8, {0x21, 0x01, 0x09, 0x38, 0xff, 0xf0, 0xff, 0x03},
     verdict::accept, "all four mapping states are encodable"},
    {"health_chain_position_1", frame_kind::health, 8, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x01, 0x04},
     verdict::accept, "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_2", frame_kind::health, 8, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x02, 0x04},
     verdict::accept, "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_3", frame_kind::health, 8, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x03, 0x04},
     verdict::accept, "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_4", frame_kind::health, 8, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x04, 0x04},
     verdict::accept, "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_5", frame_kind::health, 8, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x05, 0x04},
     verdict::accept, "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_6", frame_kind::health, 8, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x06, 0x04},
     verdict::accept, "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_none", frame_kind::health, 8, {0x21, 0x01, 0x0b, 0x18, 0xff, 0xf0, 0xff, 0x05},
     verdict::accept, "0xFF means no failing position"},
    {"health_seq_0", frame_kind::health, 8, {0x21, 0x01, 0x00, 0x18, 0xff, 0xf0, 0xff, 0x01},
     verdict::accept, "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_seq_1", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x01},
     verdict::accept, "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_seq_254", frame_kind::health, 8, {0x21, 0x01, 0xfe, 0x18, 0xff, 0xf0, 0xff, 0x01},
     verdict::accept, "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_seq_255", frame_kind::health, 8, {0x21, 0x01, 0xff, 0x18, 0xff, 0xf0, 0xff, 0x01},
     verdict::accept, "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_reject_frame_type", frame_kind::health, 8, {0x11, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00},
     verdict::frame_type_mismatch, "frame_type must match the identifier it arrived on"},
    {"health_reject_protocol_version_zero", frame_kind::health, 8, {0x20, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00},
     verdict::protocol_version_zero, "zero is not a valid version, so an all-zero byte cannot pass as one"},
    {"health_reject_protocol_version_2", frame_kind::health, 8, {0x22, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00},
     verdict::protocol_version_unsupported, "a decoder accepts only versions it implements"},
    {"health_reject_mapping_state_4", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x48, 0xff, 0xf0, 0xff, 0x00},
     verdict::mapping_state_malformed, "0x4-0xF are malformed mapping states"},
    {"health_reject_mapping_state_f", frame_kind::health, 8, {0x21, 0x01, 0x01, 0xf8, 0xff, 0xf0, 0xff, 0x00},
     verdict::mapping_state_malformed, "0x4-0xF are malformed mapping states"},
    {"health_reject_chain_position_0", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x39, 0xff, 0xf0, 0x00, 0x00},
     verdict::chain_position_malformed, "a failing position is 1-6 or 0xFF"},
    {"health_reject_chain_position_7", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x39, 0xff, 0xf0, 0x07, 0x00},
     verdict::chain_position_malformed, "a failing position is 1-6 or 0xFF"},
    {"health_reject_chain_position_254", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x39, 0xff, 0xf0, 0xfe, 0x00},
     verdict::chain_position_malformed, "a failing position is 1-6 or 0xFF"},
    {"health_reject_cycle_seq_without_cycle_valid", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x10, 0xff, 0x00, 0xff, 0x03},
     verdict::cycle_fields_inconsistent, "with cycle_valid clear the frame describes no cycle"},
    {"health_reject_produced_mask_without_cycle_valid", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x10, 0xff, 0xf0, 0xff, 0x00},
     verdict::cycle_fields_inconsistent, "sample_produced_mask is a per-cycle field"},
    {"health_reject_fault_mask_without_cycle_valid", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x10, 0xff, 0x01, 0xff, 0x00},
     verdict::cycle_fields_inconsistent, "sensor_fault_mask is per cycle too, so it must also be empty in a heartbeat"},
    {"health_reject_fault_without_sample", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x18, 0xff, 0x34, 0xff, 0x01},
     verdict::mask_fault_without_sample, "sensor_fault_mask classifies a produced sample, so a fault bit outside sample_produced_mask classifies a sample that does not exist"},
    {"health_reject_position_without_chain_fault", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x18, 0xff, 0xf0, 0x03, 0x01},
     verdict::chain_position_without_fault, "naming a failing position with no chain fault and complete masks is contradictory"},
    {"health_position_with_incomplete_mask", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x08, 0x77, 0x00, 0x04, 0x00},
     verdict::accept, "the complement of the case above, and the reason the contract's rule is compound: an incomplete enumeration is itself why a position can be named"},
    {"health_fault_mask_subset", frame_kind::health, 8, {0x21, 0x01, 0x01, 0x18, 0xff, 0xf5, 0xff, 0x02},
     verdict::accept, "a strict subset is legal: two of four produced samples classified SENSOR_FAULT"},
    {"health_reject_dlc_0", frame_kind::health, 0, {0x21, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00},
     verdict::dlc_not_8, "DLC is 8 always"},
};

}  // namespace tof_cliff_contract

// clang-format on
