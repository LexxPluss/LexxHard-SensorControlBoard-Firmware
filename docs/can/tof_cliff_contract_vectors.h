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
 * Version  : commissioning-2026-08-17
 * SHA-256  : 74ecd67460eccd4c616d7887e70124424bf2444a407f1d9c2b75b00cfc038de4
 * Profile  : commissioning-cliff-only-400k
 *
 * COMMISSIONING ONLY -- RELEASE_FORBIDDEN. Regenerate and re-pin both sides after the six-board schedule measurement; status 3/11 remain unvalidated.
 *
 * Scope: frame layout and validation verdicts only. The decoder state machine
 * (cycle assembly, retirement, staleness, event multisets) is NOT covered here.
 *
 * Zero dependencies on purpose: the SCBDriver tests have no JSON parser.
 */

#pragma once

#include <cstddef>
#include <cstdint>

// clang-format off

namespace tof_cliff_contract {

inline constexpr char kContractVersion[]{"commissioning-2026-08-17"};
inline constexpr char kContractSha256[]{"74ecd67460eccd4c616d7887e70124424bf2444a407f1d9c2b75b00cfc038de4"};
inline constexpr char kProfileName[]{"commissioning-cliff-only-400k"};
inline constexpr bool kReleaseForbidden{true};

inline constexpr uint16_t kMeasId{0x216};
inline constexpr uint16_t kHealthId{0x217};
inline constexpr uint8_t kProtocolVersion{0x1};
inline constexpr uint16_t kSentinelInvalid{0xFFFF};
inline constexpr uint8_t kSourceCount{4};
inline constexpr uint8_t kChainPositionNone{0xFF};
inline constexpr uint8_t kCycleMissFault{3};
inline constexpr uint8_t kCycleAdvanceMax{16};

// Commissioning profile. Model-derived, NOT measured. A production build must not
// inherit these values.
inline constexpr uint32_t kTCycleNominalMs{50};
inline constexpr uint32_t kTSkewMaxMs{20};
inline constexpr uint32_t kTHealthDeliveryMaxMs{20};
inline constexpr uint32_t kTHealthNominalMs{100};
inline constexpr uint32_t kTHealthMaxGapMs{300};
inline constexpr uint32_t kTCycleAssemblyMs{100};
inline constexpr uint32_t kTStartupHealthGraceMs{10000};
inline constexpr uint32_t kTMeasMaxGapMs{200};

enum class verdict : uint8_t {
    accept = 0,
    frame_type_mismatch = 1,
    source_id_out_of_range = 2,
    reserved_field_nonzero = 3,
    target_count_malformed = 4,
    no_target_encoding_inconsistent = 5,
    protocol_version_zero = 6,
    protocol_version_unsupported = 7,
    mapping_state_malformed = 8,
    chain_position_malformed = 9,
    cycle_fields_inconsistent = 10,
    chain_position_without_fault = 11,
};

enum class frame_kind : uint8_t { measurement = 0, health = 1 };

struct vector {
    const char *name;
    frame_kind kind;
    uint8_t bytes[8];
    verdict expected;
    const char *why;
};

inline constexpr size_t kVectorCount{71};

inline constexpr vector kVectors[kVectorCount]{
    {"meas_role_0_front_left", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "source_id is a stable logical role, not a chain position"},
    {"meas_role_1_rear_left", frame_kind::measurement, {0x11, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "source_id is a stable logical role, not a chain position"},
    {"meas_role_2_rear_right", frame_kind::measurement, {0x12, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "source_id is a stable logical role, not a chain position"},
    {"meas_role_3_front_right", frame_kind::measurement, {0x13, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "source_id is a stable logical role, not a chain position"},
    {"meas_status_0", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_1", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x01, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_2", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x02, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_3", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x03, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_4", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x04, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_5", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x05, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_6", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x06, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_7", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x07, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_8", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x08, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_9", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x09, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_10", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0a, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_11", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0b, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_12", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0c, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_13", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0d, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_14", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0x0e, 0x01, 0x00}, verdict::accept,
     "raw ULD status is transmitted unchanged; the class is a decoder concern"},
    {"meas_status_255_no_target", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00}, verdict::accept,
     "the ULD forces 8191 mm when active_results == 0; the packer MUST send the sentinel instead and never forward 8191 as a finite range"},
    {"meas_range_0", frame_kind::measurement, {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00}, verdict::accept,
     "zero is encodable and is not the sentinel"},
    {"meas_range_1", frame_kind::measurement, {0x10, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00}, verdict::accept,
     "one millimetre"},
    {"meas_range_65534", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xfe, 0x00, 0x01, 0x00}, verdict::accept,
     "largest encodable non-sentinel value"},
    {"meas_epoch_0", frame_kind::measurement, {0x10, 0x00, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "mapping_epoch wraps 255 -> 0"},
    {"meas_epoch_1", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "mapping_epoch wraps 255 -> 0"},
    {"meas_epoch_254", frame_kind::measurement, {0x10, 0xfe, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "mapping_epoch wraps 255 -> 0"},
    {"meas_epoch_255", frame_kind::measurement, {0x10, 0xff, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "mapping_epoch wraps 255 -> 0"},
    {"meas_cycle_0", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_cycle_1", frame_kind::measurement, {0x10, 0x01, 0x01, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_cycle_254", frame_kind::measurement, {0x10, 0x01, 0xfe, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_cycle_255", frame_kind::measurement, {0x10, 0x01, 0xff, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "cycle_seq wraps 255 -> 0; equality, not ordering, prevents splicing"},
    {"meas_targets_1", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::accept,
     "target_count is pre-reduction and diagnostic only"},
    {"meas_targets_2", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x02, 0x00}, verdict::accept,
     "target_count is pre-reduction and diagnostic only"},
    {"meas_targets_3", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x03, 0x00}, verdict::accept,
     "target_count is pre-reduction and diagnostic only"},
    {"meas_targets_4", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x04, 0x00}, verdict::accept,
     "target_count is pre-reduction and diagnostic only"},
    {"meas_reject_frame_type", frame_kind::measurement, {0x20, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::frame_type_mismatch,
     "a mis-routed filter is otherwise a silent mis-decode"},
    {"meas_reject_source_id_4", frame_kind::measurement, {0x14, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::source_id_out_of_range,
     "source_id must be 0-3"},
    {"meas_reject_source_id_15", frame_kind::measurement, {0x1f, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x00}, verdict::source_id_out_of_range,
     "source_id must be 0-3"},
    {"meas_reject_reserved_nonzero", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x01, 0x01}, verdict::reserved_field_nonzero,
     "byte 7 is reserved for a future capture tick; using it is a version bump"},
    {"meas_reject_target_count_5", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x05, 0x00}, verdict::target_count_malformed,
     "target_count > 4 is malformed"},
    {"meas_reject_none_with_finite_range", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0xff, 0x00, 0x00}, verdict::no_target_encoding_inconsistent,
     "status 255 with a finite range breaks the no-target encoding"},
    {"meas_reject_none_with_targets", frame_kind::measurement, {0x10, 0x01, 0x00, 0xff, 0xff, 0xff, 0x01, 0x00}, verdict::no_target_encoding_inconsistent,
     "status 255 must carry target_count 0"},
    {"meas_reject_zero_targets_with_valid_status", frame_kind::measurement, {0x10, 0x01, 0x00, 0x04, 0xd2, 0x00, 0x00, 0x00}, verdict::no_target_encoding_inconsistent,
     "target_count 0 is only legal as the status-255 encoding"},
    {"health_proven_cycle_valid", frame_kind::health, {0x21, 0x01, 0x07, 0x18, 0xff, 0xf0, 0xff, 0x00}, verdict::accept,
     "the ready shape: PROVEN, no chain fault, all four enumerated and produced"},
    {"health_heartbeat_unknown", frame_kind::health, {0x21, 0x01, 0x08, 0x00, 0x00, 0x00, 0xff, 0x00}, verdict::accept,
     "a heartbeat during UNKNOWN describes no cycle and depends on no measurement"},
    {"health_mapping_state_unknown", frame_kind::health, {0x21, 0x01, 0x09, 0x08, 0xff, 0xf0, 0xff, 0x03}, verdict::accept,
     "all four mapping states are encodable"},
    {"health_mapping_state_proven", frame_kind::health, {0x21, 0x01, 0x09, 0x18, 0xff, 0xf0, 0xff, 0x03}, verdict::accept,
     "all four mapping states are encodable"},
    {"health_mapping_state_lost", frame_kind::health, {0x21, 0x01, 0x09, 0x28, 0xff, 0xf0, 0xff, 0x03}, verdict::accept,
     "all four mapping states are encodable"},
    {"health_mapping_state_fault", frame_kind::health, {0x21, 0x01, 0x09, 0x38, 0xff, 0xf0, 0xff, 0x03}, verdict::accept,
     "all four mapping states are encodable"},
    {"health_chain_position_1", frame_kind::health, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x01, 0x04}, verdict::accept,
     "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_2", frame_kind::health, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x02, 0x04}, verdict::accept,
     "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_3", frame_kind::health, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x03, 0x04}, verdict::accept,
     "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_4", frame_kind::health, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x04, 0x04}, verdict::accept,
     "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_5", frame_kind::health, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x05, 0x04}, verdict::accept,
     "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_6", frame_kind::health, {0x21, 0x01, 0x0a, 0x3a, 0x77, 0x00, 0x06, 0x04}, verdict::accept,
     "a failing position is reported with the chain fault that found it"},
    {"health_chain_position_none", frame_kind::health, {0x21, 0x01, 0x0b, 0x18, 0xff, 0xf0, 0xff, 0x05}, verdict::accept,
     "0xFF means no failing position"},
    {"health_seq_0", frame_kind::health, {0x21, 0x01, 0x00, 0x18, 0xff, 0xf0, 0xff, 0x01}, verdict::accept,
     "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_seq_1", frame_kind::health, {0x21, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x01}, verdict::accept,
     "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_seq_254", frame_kind::health, {0x21, 0x01, 0xfe, 0x18, 0xff, 0xf0, 0xff, 0x01}, verdict::accept,
     "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_seq_255", frame_kind::health, {0x21, 0x01, 0xff, 0x18, 0xff, 0xf0, 0xff, 0x01}, verdict::accept,
     "health_seq wraps 255 -> 0 and proves novelty, never age"},
    {"health_reject_frame_type", frame_kind::health, {0x11, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00}, verdict::frame_type_mismatch,
     "frame_type must match the identifier it arrived on"},
    {"health_reject_protocol_version_zero", frame_kind::health, {0x20, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00}, verdict::protocol_version_zero,
     "zero is not a valid version, so an all-zero byte cannot pass as one"},
    {"health_reject_protocol_version_2", frame_kind::health, {0x22, 0x01, 0x01, 0x18, 0xff, 0xf0, 0xff, 0x00}, verdict::protocol_version_unsupported,
     "a decoder accepts only versions it implements"},
    {"health_reject_mapping_state_4", frame_kind::health, {0x21, 0x01, 0x01, 0x48, 0xff, 0xf0, 0xff, 0x00}, verdict::mapping_state_malformed,
     "0x4-0xF are malformed mapping states"},
    {"health_reject_mapping_state_f", frame_kind::health, {0x21, 0x01, 0x01, 0xf8, 0xff, 0xf0, 0xff, 0x00}, verdict::mapping_state_malformed,
     "0x4-0xF are malformed mapping states"},
    {"health_reject_chain_position_0", frame_kind::health, {0x21, 0x01, 0x01, 0x39, 0xff, 0xf0, 0x00, 0x00}, verdict::chain_position_malformed,
     "a failing position is 1-6 or 0xFF"},
    {"health_reject_chain_position_7", frame_kind::health, {0x21, 0x01, 0x01, 0x39, 0xff, 0xf0, 0x07, 0x00}, verdict::chain_position_malformed,
     "a failing position is 1-6 or 0xFF"},
    {"health_reject_chain_position_254", frame_kind::health, {0x21, 0x01, 0x01, 0x39, 0xff, 0xf0, 0xfe, 0x00}, verdict::chain_position_malformed,
     "a failing position is 1-6 or 0xFF"},
    {"health_reject_cycle_seq_without_cycle_valid", frame_kind::health, {0x21, 0x01, 0x01, 0x10, 0xff, 0x00, 0xff, 0x03}, verdict::cycle_fields_inconsistent,
     "with cycle_valid clear the frame describes no cycle"},
    {"health_reject_produced_mask_without_cycle_valid", frame_kind::health, {0x21, 0x01, 0x01, 0x10, 0xff, 0xf0, 0xff, 0x00}, verdict::cycle_fields_inconsistent,
     "sample_produced_mask is a per-cycle field"},
    {"health_reject_position_without_chain_fault", frame_kind::health, {0x21, 0x01, 0x01, 0x18, 0xff, 0xf0, 0x03, 0x01}, verdict::chain_position_without_fault,
     "naming a failing position with no chain fault set is contradictory"},
};

}  // namespace tof_cliff_contract

// clang-format on
