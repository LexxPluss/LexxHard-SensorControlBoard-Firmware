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

/*
 * Candidate N's WIRE LAYOUT, development version: the eight bytes of a Package ToF data frame and
 * of a Package ToF health frame.
 *
 * WHY N, AND WHAT "CANDIDATE" STILL MEANS HERE. The requirements document asks for a 32-bit
 * acquisition sequence that firmware generates at acquisition and carries, and a sequence that
 * never restarts IS the boot identity, so N spends no bits on a boot field. That was chosen as the
 * development direction, not proved deployable: N stands or falls on a sequence issuer that
 * survives power loss, and that issuer's real persistence and its field storage acceptance are
 * separate open items (see allmemory/tof_package_sensing/storage_feasibility_issuer.md). This file
 * is the byte layout only. Adopting it does not adopt N.
 *
 * THE L0 CONTRACT IS NOT TOUCHED. These are new frames on new CAN IDs. The ids themselves are a
 * pending registration item -- nothing here names one, deliberately, because the registrar has not
 * confirmed any. tof_grid_packer and the shipping /sensor_set/low_object_* interface are unchanged.
 *
 * DATA FRAME, 8 bytes, one zone per frame:
 *
 *     byte 0..3  uint32 acquisition sequence, big-endian. The FULL number, in every frame.
 *     byte 4     bits 5..0 zone index 0..63; bits 7..6 RESERVED, must be zero
 *     byte 5..6  uint16 distance in mm, big-endian, raw
 *     byte 7     raw ULD zone status, untouched
 *
 * HEALTH FRAME, 8 bytes, TWO DIFFERENT PURPOSES that must not be confused:
 *
 *     byte 0..3  uint32 sequence, big-endian: the grid's sequence for metadata, ZERO for an
 *                independent diagnostic
 *     byte 4     sensor_state, the requirement's three values only (0 ok, 1 degraded, 2 error)
 *     byte 5     bit 0 independent diagnostic; bit 1 INACTIVE; bits 7..2 RESERVED, must be zero
 *     byte 6..7  RESERVED, must be zero
 *
 * - GRID METADATA (bit 0 clear) carries the same sequence as its 64 data frames and is immutable
 *   once sent. A grid publishes only when all 64 zones AND this metadata are in hand. Missing,
 *   conflicting or timed out means the WHOLE GRID is refused. So a successful grid is 65 frames,
 *   and data and metadata must be accepted in any arrival order -- nothing here requires metadata
 *   first or last.
 * - INDEPENDENT DIAGNOSTIC (bit 0 set) carries states that belong to no acquisition: no data,
 *   INACTIVE, a bus fault. It MUST NOT participate in reassembly and MUST NOT refresh any grid's
 *   timestamp. That rule is what keeps a failed acquisition from reusing the last good sequence and
 *   rewriting that grid's health from OK to ERROR after the fact -- a consumer that had already
 *   acted on the OK grid would see its basis change underneath it.
 *
 * WHY THE DIAGNOSTIC'S SEQUENCE IS CHECKED AND NOT MERELY CONVENTIONAL. The flag bit is the
 * discriminator; the zero is a cross-field invariant, and this module refuses a frame that breaks
 * it in either direction:
 *
 * - bit 0 set with a non-zero sequence is refused, because a diagnostic that names an acquisition
 *   invites exactly the reuse the separation exists to prevent;
 * - bit 0 CLEAR with sequence zero is refused too -- metadata_sequence_zero -- because otherwise a
 *   single flipped flag bit turns an independent diagnostic into perfectly well-formed metadata for
 *   the grid numbered 0, and CAN's CRC does not make that impossible, only unlikely.
 *
 * THE CONSEQUENCE IS A CONSTRAINT ON THE ISSUER, STATED HERE SO IT IS NOT DISCOVERED LATER:
 * sequence 0 must never be issued. tof_seq_issuer currently issues it as its very first number
 * (tof_seq_issuer.cpp, next_number_ = first, first = 0 on a freshly commissioned journal), so
 * adopting this layout requires changing that -- one number skipped, once in the device's life --
 * or accepting the alias. This module does not reach into the issuer to fix it; the alias is
 * asserted as a test so the choice is recorded rather than assumed.
 *
 * WHAT THIS MODULE DOES NOT DO. It does not reassemble (tof_package_assoc), does not decide health
 * (tof_package_health), does not allocate sequences (tof_seq_issuer), does not transmit, and knows
 * no CAN id. It validates structure only: a state byte out of the requirement's range, a reserved
 * bit set, an impossible flag combination. It never judges a distance or a status -- every uint16
 * is a distance and all 256 statuses travel untouched, which is the point of the 16+8 widths.
 *
 * RESERVED BITS ARE REFUSED, NOT MASKED AWAY. A decoder that quietly ignores them cannot be
 * extended later: the moment one is given a meaning, every old decoder silently drops it. Refusing
 * now is what makes the spare bits actually spare.
 *
 * Pure: no Zephyr, no vendor headers, no allocation. Tested on native_sim against golden vectors
 * written by an independent generator, `gen_pkg_wire_n_vectors` under `docs/can/`. Its file
 * extension is left out on purpose: scripts/check_language_boundary.sh refuses any mention of one
 * from the application tree, so that a production file can never acquire a build-time dependency
 * on offline tooling by way of a comment someone later turns into a rule.
 */

#include <stddef.h>
#include <stdint.h>

#include "tof_package_health.hpp"

namespace lexxhard::tof_package_wire_n {

inline constexpr size_t kFrameBytes{8};
inline constexpr size_t kZones{64};
/* One zone per data frame, so these are facts about this layout rather than about the payload: 64
 * data frames plus exactly one metadata frame. Independent diagnostics are not counted -- they
 * belong to no grid, which is why they are extra traffic and never one of the 65. */
inline constexpr size_t kDataFramesPerGrid{kZones};
inline constexpr size_t kFramesPerGrid{kDataFramesPerGrid + 1};

/* Field positions, so that the test and the generator can disagree with the code loudly. */
inline constexpr size_t kSequenceOffset{0};
inline constexpr size_t kSequenceBytes{4};
inline constexpr size_t kZoneOffset{4};
inline constexpr size_t kDistanceOffset{5};
inline constexpr size_t kStatusOffset{7};
inline constexpr size_t kStateOffset{4};
inline constexpr size_t kFlagsOffset{5};
inline constexpr size_t kHealthReservedOffset{6};

inline constexpr uint8_t kZoneMask{0x3F};
inline constexpr uint8_t kZoneReservedMask{0xC0};
inline constexpr uint8_t kFlagStandalone{0x01};
inline constexpr uint8_t kFlagInactive{0x02};
inline constexpr uint8_t kFlagsReservedMask{0xFC};

/* The one sequence value this layout cannot carry, because it is the diagnostic's placeholder. */
inline constexpr uint32_t kReservedSequence{0};

using sensor_state = tof_package_health::sensor_state;

/* Reused rather than redeclared. A second enum of the requirement's three states is a defect
 * waiting for whoever writes the second decoder, which is the same reason the byte order follows
 * the existing contract's. */
inline constexpr uint8_t kMaxStateValue{static_cast<uint8_t>(sensor_state::error)};

struct data_frame {
    /* The full acquisition number, not a tag. Never kReservedSequence. */
    uint32_t sequence{0};
    uint8_t zone{0};
    /* Raw. 0xFFFF is a value, not a verdict. */
    uint16_t distance_mm{0};
    /* Raw ULD status. Never rewritten. */
    uint8_t status{0};
};

enum class health_purpose : uint8_t {
    /* Belongs to the acquisition named by `sequence`; one per grid; immutable. */
    grid_metadata,
    /* Belongs to no acquisition. Must not reassemble, must not refresh a grid's timestamp. */
    independent_diagnostic,
};

struct health_frame {
    /* The grid's number for metadata; must be kReservedSequence for a diagnostic. */
    uint32_t sequence{0};
    sensor_state state{sensor_state::error};
    health_purpose purpose{health_purpose::independent_diagnostic};
    /* The availability signal, carried only on a diagnostic -- a grid that exists cannot have come
     * from a sensor that never started. */
    bool inactive{false};
};

enum class result : uint8_t {
    ok,
    /* Not exactly kFrameBytes supplied. Refused before anything is written or read: a short frame
     * must not decode to a zone whose distance reads 0 mm. */
    frame_length_mismatch,
    /* A null buffer, or a null destination. */
    null_buffer,
    /* Encode only: a zone index the 6-bit field cannot hold. On decode this is unreachable -- six
     * bits are 0..63 exactly -- and that is stated rather than left as a gap a reader must verify.
     */
    zone_out_of_range,
    /* A reserved bit was set: byte 4 bits 7..6 on a data frame, byte 5 bits 7..2 or bytes 6..7 on a
     * health frame. */
    reserved_bits_set,
    /* A state byte outside the requirement's {0, 1, 2}. The enum is not extended to absorb it. */
    state_out_of_range,
    /* INACTIVE on a grid-metadata frame. */
    inactive_without_diagnostic,
    /* An independent diagnostic naming an acquisition. */
    diagnostic_sequence_not_zero,
    /* Grid metadata numbered 0 -- the alias that a single flipped flag bit would produce. */
    metadata_sequence_zero,
    /* A `health_purpose` that is neither of the two values. Named rather than normalised: an
     * out-of-domain purpose used to encode as grid metadata, because the flag bit is only set for
     * independent_diagnostic and anything else fell through to "not a diagnostic". That turned a
     * caller's bug into a frame that claims to describe an acquisition, which is the one thing the
     * two purposes exist to keep apart. On DECODE this is unreachable -- the purpose comes from one
     * bit -- and that asymmetry is the point: the domain has to be checked where a value can
     * actually leave it. */
    purpose_out_of_range,
    /* A data frame numbered 0. Refused for a different reason than the metadata case: an all-zero
     * frame -- a stuck bus, an uninitialised buffer that got transmitted -- would otherwise decode
     * to a perfectly plausible measurement of zone 0 at 0 mm, and nothing downstream could tell it
     * from one. Reserving the number end to end is what makes that frame refusable at all. */
    data_sequence_zero,
};

/* Each writes exactly kFrameBytes. On failure nothing is written. */
result encode_data(const data_frame &in, uint8_t *frame, size_t frame_len);
result encode_health(const health_frame &in, uint8_t *frame, size_t frame_len);

/* Each reads exactly kFrameBytes. On failure `out` is untouched. */
result decode_data(const uint8_t *frame, size_t frame_len, data_frame &out);
result decode_health(const uint8_t *frame, size_t frame_len, health_frame &out);

const char *result_name(result r);
const char *purpose_name(health_purpose p);

}
