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
 * ArtefactSet : 1a0a2094e928392d29c7153f84de6951159113198714e0b734e6ebd39bf74b07
 * Profile  : commissioning-cliff-only-400k
 *
 * COMMISSIONING ONLY -- RELEASE_FORBIDDEN. Regenerate and re-pin both sides after the six-board schedule measurement; status 3/11 remain unvalidated.
 *
 * Production half: identifiers, field encodings and the status classification
 * table. Contains no test vectors and no timing values -- see
 * tof_cliff_layout_vectors.json for the commissioning profile, which a
 * production configuration must not inherit.
 *
 * clang-format is disabled for the whole file, starting above the include guard.
 * SCBDriver's CI reformats every .h in the repository with an explicitly named
 * style file, which overrides any directory .clang-format -- so a generated file
 * can only stay byte-identical across the two repositories by opting out here, in
 * the generator, rather than per checkout.
 */

// clang-format off
#pragma once

#include <cstddef>
#include <cstdint>

namespace tof_cliff_contract {

inline constexpr char kContractVersion[]{"commissioning-2026-08-18b"};
inline constexpr char kContractSha256[]{"4b3dae652d73e6eabf17d721a90b06371effbb3aaefb3d25a2218bb860dd47f7"};
inline constexpr char kProfileName[]{"commissioning-cliff-only-400k"};

// The contract SHA says which contract. This says which generated artefacts: it
// hashes the contract text together with the generator's own source, so a change
// to what the generator emits is visible even when the contract stands still.
// Both repositories pin it.
inline constexpr char kArtefactSetId[]{"1a0a2094e928392d29c7153f84de6951159113198714e0b734e6ebd39bf74b07"};
inline constexpr bool kReleaseForbidden{true};

inline constexpr uint16_t kMeasId{0x216};
inline constexpr uint16_t kHealthId{0x217};
inline constexpr uint8_t kProtocolVersion{0x1};
inline constexpr uint16_t kSentinelInvalid{0xFFFF};
inline constexpr uint8_t kSourceCount{4};
// How many targets one sensor can report. NOT interchangeable with kSourceCount,
// which is how many sensors the chain carries; both are 4 today and neither implies
// the other.
inline constexpr uint8_t kMaxTargets{4};
inline constexpr uint8_t kChainPositionNone{0xFF};
inline constexpr uint8_t kCycleMissFault{3};
inline constexpr uint8_t kCycleAdvanceMax{16};
inline constexpr uint8_t kDlc{8};
inline constexpr uint8_t kMeasFrameType{0x1};
inline constexpr uint8_t kHealthFrameType{0x2};
inline constexpr uint8_t kCycleValidBit{0x8};
inline constexpr uint8_t kChainFaultBits{0x7};

// The status classification table. Contract-owned so the packer and the decoder
// share it instead of each reimplementing it. NO_SAMPLE statuses are never
// transmitted: a frame carrying one is a producer defect, not an unusable sample.
enum class status_class : uint8_t {
    valid_range = 0,
    no_target = 1,
    sensor_fault = 2,
    no_sample = 3,
};

struct status_row {
    uint8_t raw;
    status_class cls;
    bool validation_pending;
};

inline constexpr size_t kStatusRowCount{16};

inline constexpr status_row kStatusTable[kStatusRowCount]{
    {0, status_class::valid_range, false},
    {1, status_class::no_target, false},
    {2, status_class::no_target, false},
    {3, status_class::sensor_fault, true},
    {4, status_class::no_target, false},
    {5, status_class::sensor_fault, false},
    {6, status_class::no_sample, false},
    {7, status_class::no_target, false},
    {8, status_class::sensor_fault, false},
    {9, status_class::sensor_fault, false},
    {10, status_class::no_sample, false},
    {11, status_class::no_target, true},
    {12, status_class::no_target, false},
    {13, status_class::sensor_fault, false},
    {14, status_class::sensor_fault, false},
    {255, status_class::no_target, false},
};

// Reduction priority, most conservative first. The surviving class is the
// highest-priority one present among the targets.
inline constexpr status_class kReductionPriority[]{
    status_class::sensor_fault,
    status_class::no_sample,
    status_class::no_target,
    status_class::valid_range,
};

// The validation vocabulary, shared by the producer and the consumer. A decoder
// returns one of these; the layout vectors state which one each frame must get.
// Production-visible on purpose: the decoder's return type belongs to the
// contract, not to the test suite that happens to exercise it.
enum class verdict : uint8_t {
    accept = 0,
    dlc_not_8 = 1,
    frame_type_mismatch = 2,
    source_id_out_of_range = 3,
    reserved_field_nonzero = 4,
    target_count_malformed = 5,
    status_undefined = 6,
    status_not_transmissible = 7,
    range_contradicts_status = 8,
    no_target_encoding_inconsistent = 9,
    protocol_version_zero = 10,
    protocol_version_unsupported = 11,
    mapping_state_malformed = 12,
    chain_position_malformed = 13,
    cycle_fields_inconsistent = 14,
    mask_fault_without_sample = 15,
    chain_position_without_fault = 16,
};

}  // namespace tof_cliff_contract

// clang-format on
