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
#include "tof_package_wire_n.hpp"

namespace lexxhard::tof_package_wire_n {

namespace {

/* Big-endian by hand rather than through sys_put_be32, because this file is built for native_sim
 * as well and a byte order must not depend on which headers the host happens to have. */
void put_be32(uint8_t *at, uint32_t value)
{
    at[0] = static_cast<uint8_t>(value >> 24);
    at[1] = static_cast<uint8_t>(value >> 16);
    at[2] = static_cast<uint8_t>(value >> 8);
    at[3] = static_cast<uint8_t>(value);
}

uint32_t get_be32(const uint8_t *at)
{
    return (static_cast<uint32_t>(at[0]) << 24) | (static_cast<uint32_t>(at[1]) << 16) |
           (static_cast<uint32_t>(at[2]) << 8) | static_cast<uint32_t>(at[3]);
}

void put_be16(uint8_t *at, uint16_t value)
{
    at[0] = static_cast<uint8_t>(value >> 8);
    at[1] = static_cast<uint8_t>(value);
}

uint16_t get_be16(const uint8_t *at)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(at[0]) << 8) | at[1]);
}

/* Shared by all four entry points so that a null or a wrong length is refused in one place and
 * cannot be refused differently by direction. */
result check_buffer(const void *frame, size_t frame_len)
{
    if (frame == nullptr) {
        return result::null_buffer;
    }
    if (frame_len != kFrameBytes) {
        return result::frame_length_mismatch;
    }
    return result::ok;
}

}

result encode_data(const data_frame &in, uint8_t *frame, size_t frame_len)
{
    const result buffer = check_buffer(frame, frame_len);
    if (buffer != result::ok) {
        return buffer;
    }
    /* Content checks all run before the first byte is written: a refused frame must leave the
     * caller's buffer exactly as it was, so a retry cannot transmit half of a rejected frame. */
    if (in.sequence == kReservedSequence) {
        return result::data_sequence_zero;
    }
    if (in.zone > kZoneMask) {
        return result::zone_out_of_range;
    }

    put_be32(&frame[kSequenceOffset], in.sequence);
    frame[kZoneOffset] = static_cast<uint8_t>(in.zone & kZoneMask);
    put_be16(&frame[kDistanceOffset], in.distance_mm);
    frame[kStatusOffset] = in.status;
    return result::ok;
}

result decode_data(const uint8_t *frame, size_t frame_len, data_frame &out)
{
    const result buffer = check_buffer(frame, frame_len);
    if (buffer != result::ok) {
        return buffer;
    }
    /* Structure before meaning: a reserved bit makes the whole frame a frame this decoder does not
     * understand, so its other fields are not worth reading. */
    if ((frame[kZoneOffset] & kZoneReservedMask) != 0U) {
        return result::reserved_bits_set;
    }

    const uint32_t sequence = get_be32(&frame[kSequenceOffset]);
    if (sequence == kReservedSequence) {
        return result::data_sequence_zero;
    }

    /* Decoded into a local and copied out only on success. `out` must be untouched on refusal, and
     * writing fields as they are parsed would leave it half updated. */
    data_frame decoded;
    decoded.sequence = sequence;
    decoded.zone = static_cast<uint8_t>(frame[kZoneOffset] & kZoneMask);
    decoded.distance_mm = get_be16(&frame[kDistanceOffset]);
    decoded.status = frame[kStatusOffset];
    out = decoded;
    return result::ok;
}

namespace {

/* The cross-field rules, written once so that encode and decode cannot drift apart. They are the
 * whole reason the two health purposes stay separable, so a second copy of them is a defect. */
result check_health_fields(uint32_t sequence, uint8_t state, health_purpose purpose, bool inactive)
{
    /* The domain first. Every rule below asks which of the two purposes this is, and a third value
     * would answer "not a diagnostic" to all of them -- encoding silently as grid metadata. */
    if (purpose != health_purpose::grid_metadata &&
        purpose != health_purpose::independent_diagnostic) {
        return result::purpose_out_of_range;
    }
    if (state > kMaxStateValue) {
        return result::state_out_of_range;
    }
    if (inactive && purpose != health_purpose::independent_diagnostic) {
        return result::inactive_without_diagnostic;
    }
    if (purpose == health_purpose::independent_diagnostic) {
        return sequence == kReservedSequence ? result::ok : result::diagnostic_sequence_not_zero;
    }
    return sequence == kReservedSequence ? result::metadata_sequence_zero : result::ok;
}

}

result encode_health(const health_frame &in, uint8_t *frame, size_t frame_len)
{
    const result buffer = check_buffer(frame, frame_len);
    if (buffer != result::ok) {
        return buffer;
    }

    const uint8_t state = static_cast<uint8_t>(in.state);
    const result fields = check_health_fields(in.sequence, state, in.purpose, in.inactive);
    if (fields != result::ok) {
        return fields;
    }

    uint8_t flags = 0;
    if (in.purpose == health_purpose::independent_diagnostic) {
        flags = static_cast<uint8_t>(flags | kFlagStandalone);
    }
    if (in.inactive) {
        flags = static_cast<uint8_t>(flags | kFlagInactive);
    }

    put_be32(&frame[kSequenceOffset], in.sequence);
    frame[kStateOffset] = state;
    frame[kFlagsOffset] = flags;
    /* Written, not left alone. A reserved byte the sender does not clear is a reserved byte the
     * receiver will one day refuse. */
    frame[kHealthReservedOffset] = 0;
    frame[kHealthReservedOffset + 1] = 0;
    return result::ok;
}

result decode_health(const uint8_t *frame, size_t frame_len, health_frame &out)
{
    const result buffer = check_buffer(frame, frame_len);
    if (buffer != result::ok) {
        return buffer;
    }
    if ((frame[kFlagsOffset] & kFlagsReservedMask) != 0U || frame[kHealthReservedOffset] != 0U ||
        frame[kHealthReservedOffset + 1] != 0U) {
        return result::reserved_bits_set;
    }

    const uint32_t sequence = get_be32(&frame[kSequenceOffset]);
    const uint8_t state = frame[kStateOffset];
    const bool standalone = (frame[kFlagsOffset] & kFlagStandalone) != 0U;
    const bool inactive = (frame[kFlagsOffset] & kFlagInactive) != 0U;
    const health_purpose purpose =
        standalone ? health_purpose::independent_diagnostic : health_purpose::grid_metadata;

    const result fields = check_health_fields(sequence, state, purpose, inactive);
    if (fields != result::ok) {
        return fields;
    }

    health_frame decoded;
    decoded.sequence = sequence;
    decoded.state = static_cast<sensor_state>(state);
    decoded.purpose = purpose;
    decoded.inactive = inactive;
    out = decoded;
    return result::ok;
}

const char *result_name(result r)
{
    switch (r) {
    case result::ok: return "ok";
    case result::frame_length_mismatch: return "frame_length_mismatch";
    case result::null_buffer: return "null_buffer";
    case result::zone_out_of_range: return "zone_out_of_range";
    case result::reserved_bits_set: return "reserved_bits_set";
    case result::state_out_of_range: return "state_out_of_range";
    case result::inactive_without_diagnostic: return "inactive_without_diagnostic";
    case result::diagnostic_sequence_not_zero: return "diagnostic_sequence_not_zero";
    case result::metadata_sequence_zero: return "metadata_sequence_zero";
    case result::data_sequence_zero: return "data_sequence_zero";
    case result::purpose_out_of_range: return "purpose_out_of_range";
    }
    return "unknown";
}

const char *purpose_name(health_purpose p)
{
    switch (p) {
    case health_purpose::grid_metadata: return "grid_metadata";
    case health_purpose::independent_diagnostic: return "independent_diagnostic";
    }
    return "unknown";
}

}
