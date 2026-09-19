/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commission_wire.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_commission_wire {

namespace {

/* Little-endian, spelled out rather than memcpy'd from a struct. The SCB and the IPC are both
 * little-endian today and that is exactly why it is written by hand: a layout that happens to match
 * the compiler's is a layout nobody has specified. */
void put_u32(uint8_t *at, uint32_t v)
{
    at[0] = static_cast<uint8_t>(v & 0xFFU);
    at[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
    at[2] = static_cast<uint8_t>((v >> 16) & 0xFFU);
    at[3] = static_cast<uint8_t>((v >> 24) & 0xFFU);
}

uint32_t get_u32(const uint8_t *at)
{
    return static_cast<uint32_t>(at[0]) | (static_cast<uint32_t>(at[1]) << 8) |
           (static_cast<uint32_t>(at[2]) << 16) | (static_cast<uint32_t>(at[3]) << 24);
}

/* Length and version, in that order, before anything else is touched. Both refusals mean the same
 * thing to the caller -- do not interpret the rest -- and they are separate values because they send
 * an operator to different places: a short frame is a transport or a sender bug, an unknown version
 * is a deployment that has drifted. */
decode_error check_header(const uint8_t *data, size_t len)
{
    if (data == nullptr || len != kFrameLen)
        return decode_error::bad_length;
    if (data[0] != kProtocolVersion)
        return decode_error::bad_version;
    return decode_error::none;
}

bool valid_opcode(uint8_t raw)
{
    return raw == static_cast<uint8_t>(opcode::prove_and_start) ||
           raw == static_cast<uint8_t>(opcode::start_only);
}

} // namespace

void encode_request(const request &in, uint8_t out[kFrameLen])
{
    out[0] = in.version;
    out[1] = static_cast<uint8_t>(in.op);
    out[2] = in.seq;
    out[3] = in.wire_epoch;
    put_u32(&out[4], in.session_token);
}

void encode_session_status(const session_status &in, uint8_t out[kFrameLen])
{
    out[0] = in.version;
    out[1] = static_cast<uint8_t>(status_kind::session);
    put_u32(&out[2], in.session_token);
    out[6] = static_cast<uint8_t>((in.profile_enabled ? 0x01U : 0U) |
                                  (in.transaction_in_progress ? 0x02U : 0U));
    out[7] = 0;
}

void encode_transaction_status(const transaction_status &in, uint8_t out[kFrameLen])
{
    out[0] = in.version;
    out[1] = static_cast<uint8_t>(status_kind::transaction);
    out[2] = in.seq;
    out[3] = in.wire_epoch;
    out[4] = static_cast<uint8_t>(in.ph);
    out[5] = static_cast<uint8_t>(in.res);
    out[6] = static_cast<uint8_t>(in.stage);
    out[7] = static_cast<uint8_t>(in.detail);
}

decode_error decode_request(const uint8_t *data, size_t len, request &out)
{
    if (const decode_error e{check_header(data, len)}; e != decode_error::none)
        return e;

    /* The opcode is validated here and not left to the state machine, because an opcode this build
     * does not implement makes the rest of the frame uninterpretable in the same way a bad version
     * does: what byte 3 means is defined per opcode. */
    if (!valid_opcode(data[1]))
        return decode_error::bad_opcode;

    out.version = data[0];
    out.op = static_cast<opcode>(data[1]);
    out.seq = data[2];
    out.wire_epoch = data[3];
    out.session_token = get_u32(&data[4]);
    return decode_error::none;
}

decode_error decode_status_kind(const uint8_t *data, size_t len, status_kind &out)
{
    if (const decode_error e{check_header(data, len)}; e != decode_error::none)
        return e;
    if (data[1] != static_cast<uint8_t>(status_kind::session) &&
        data[1] != static_cast<uint8_t>(status_kind::transaction))
        return decode_error::bad_kind;
    out = static_cast<status_kind>(data[1]);
    return decode_error::none;
}

decode_error decode_session_status(const uint8_t *data, size_t len, session_status &out)
{
    status_kind kind{};
    if (const decode_error e{decode_status_kind(data, len, kind)}; e != decode_error::none)
        return e;
    if (kind != status_kind::session)
        return decode_error::bad_kind;
    if (data[7] != 0)
        return decode_error::reserved_not_zero;

    out.version = data[0];
    out.session_token = get_u32(&data[2]);
    out.profile_enabled = (data[6] & 0x01U) != 0;
    out.transaction_in_progress = (data[6] & 0x02U) != 0;
    return decode_error::none;
}

decode_error decode_transaction_status(const uint8_t *data, size_t len, transaction_status &out)
{
    status_kind kind{};
    if (const decode_error e{decode_status_kind(data, len, kind)}; e != decode_error::none)
        return e;
    if (kind != status_kind::transaction)
        return decode_error::bad_kind;

    out.version = data[0];
    out.seq = data[2];
    out.wire_epoch = data[3];
    /* phase, result, stage and detail are NOT range-checked into their enums here. A decoder that
     * refused an unknown value would drop the very frame that tells it a newer firmware is present,
     * and the draft's rule is that an unrecognised stage or detail is RENDERED as unknown rather
     * than treated as a protocol error. Rendering is the caller's, not the codec's. */
    out.ph = static_cast<phase>(data[4]);
    out.res = static_cast<result>(data[5]);
    out.stage = static_cast<wire_stage>(data[6]);
    out.detail = static_cast<wire_detail>(data[7]);
    return decode_error::none;
}

} // namespace lexxhard::tof_commission_wire

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
