/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The automatic-commissioning downlink, as bytes. Nothing else.
 *
 * WHAT THIS LAYER IS. Pack and unpack, length and version, and the fixed wire enumerations. It holds
 * no state, makes no decisions and does not know which CAN identifier carries it. The pair was
 * allocated on 2026-09-19 -- 0x218 request, 0x219 status -- and lives in the generated wire
 * contract; this file still names neither, because a codec that knew which identifier carried it
 * would be a codec that could be wrong about one. The binding reads them from the contract and the
 * runtime is handed them.
 *
 * The three layers are deliberately separate: this codec, the mapper that turns the firmware's
 * internal enums into these wire values, and the protocol state machine that owns sessions,
 * idempotency and budgets. Stale tokens and sequence conflicts belong to the third and are absent
 * here -- a codec that knew about them would be a codec that could refuse a frame for a reason its
 * caller could not see.
 *
 * SPECIFIED IN allmemory/hanging_object/draft_commission_downlink_protocol.md, which is a DRAFT: not
 * frozen, no golden-vector artefact, and nothing here may be wired to a real filter until it is.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_commission_wire {

constexpr uint8_t kProtocolVersion{1};

/* Both frames are exactly this long. A shorter one is not a truncated frame to be salvaged: see
 * decode_request(), where the reason is that a salvaged frame would carry an untrustworthy sequence
 * number into the table that exists to make sequence numbers trustworthy. */
constexpr size_t kFrameLen{8};

enum class opcode : uint8_t {
    prove_and_start = 1,
    start_only = 2,
};

enum class status_kind : uint8_t {
    session = 0,
    transaction = 1,
};

enum class phase : uint8_t {
    accepted = 0,
    proving = 1,
    proven = 2,
    starting = 3,
    done = 4,
    refused = 5,
};

/* Terminal means the transaction is over: `done` carries the outcome, `refused` carries why it never
 * ran. Anything else is progress, and a host that treated it as terminal would drop the pending
 * tuple it needs to retransmit with. */
constexpr bool is_terminal(phase p)
{
    return p == phase::done || p == phase::refused;
}

enum class result : uint8_t {
    ok = 0,
    disabled = 1,
    not_permitted = 2,
    busy_chain = 3,
    stale_session = 4,
    seq_conflict = 5,
    /* Reserved and never sent by version 1: a frame whose version this firmware does not implement is
     * discarded without a result frame, so no path produces it. Numbered so a later version that does
     * want to answer a version mismatch has a value waiting. */
    bad_version = 6,
    bad_opcode = 7,
    epoch_reused = 8,
    epoch_mismatch = 9,
    proof_failed = 10,
    start_failed = 11,
    misconfigured = 12,
    seq_space_exhausted = 13,
    attempts_exhausted = 14,
    no_session = 15,
    /* Distinct from busy_chain because the transaction RAN: the chain was re-enumerated and the bus
     * retimed before the authority refused at begin_epoch(). The host owes a new ordinal for it and
     * does not for busy_chain. */
    busy_at_commit = 16,
    internal_error = 17,
};

enum class wire_stage : uint8_t {
    not_started = 0,
    quiesce = 1,
    first_walk = 2,
    tail_isolation = 3,
    second_walk = 4,
    proof_evaluation = 5,
    retime = 6,
    identity_recheck = 7,
    commit = 8,
    acquisition_start = 9,
    complete = 10,
    /* A DECODER-SIDE RENDERING, NEVER TRANSMITTED. It is what an older decoder writes down for a
     * value a newer firmware sent and it has no name for, so evidence records "this version does not
     * know" rather than a bare number. Nothing in the mapper produces it. */
    unknown = 255,
};

enum class wire_detail : uint8_t {
    none = 0,
    chain_busy = 1,
    walk_mismatch = 2,
    tail_would_not_isolate = 3,
    proof_speed_refused = 4,
    product_speed_refused = 5,
    identity_disagreed = 6,
    position_silent = 7,
    commit_refused = 8,
    acquisition_refused = 9,
    unknown = 255,  /* as wire_stage::unknown */
};

struct request {
    uint8_t version{kProtocolVersion};
    /* RAW, AND NOT VALIDATED HERE. The specified order is length, version, session token, opcode --
     * so the codec cannot be the thing that rejects an opcode, or a frame from a previous boot would
     * be judged on its opcode before anyone had established it belongs to this session. The state
     * machine checks it with is_known_opcode() after the token. */
    uint8_t raw_op{static_cast<uint8_t>(opcode::prove_and_start)};
    uint8_t seq{0};
    uint8_t wire_epoch{0};
    uint32_t session_token{0};
};

constexpr bool is_known_opcode(uint8_t raw)
{
    return raw == static_cast<uint8_t>(opcode::prove_and_start) ||
           raw == static_cast<uint8_t>(opcode::start_only);
}

struct session_status {
    uint8_t version{kProtocolVersion};
    uint32_t session_token{0};
    bool profile_enabled{false};
    bool transaction_in_progress{false};
};

struct transaction_status {
    uint8_t version{kProtocolVersion};
    uint8_t seq{0};
    uint8_t wire_epoch{0};
    phase ph{phase::refused};
    result res{result::internal_error};
    wire_stage stage{wire_stage::not_started};
    wire_detail detail{wire_detail::none};
};

enum class decode_error : uint8_t {
    none = 0,
    /* Not eight bytes. Nothing in the frame is interpreted, including the sequence number. */
    bad_length,
    /* A version this build does not implement. Nothing past byte 0 is interpreted, because a field
     * that moved in a later version would otherwise be read as one of ours. */
    bad_version,
    bad_kind,
    /* A reserved byte that is not zero. Refused rather than ignored: ignoring it is how a later
     * version's new field gets silently discarded by an older reader that believed it understood
     * the frame. */
    reserved_not_zero,
};

/* Encoders write exactly kFrameLen bytes and cannot fail: every field is already a valid wire
 * value by construction, which is the mapper's job rather than this one's. */
void encode_request(const request &in, uint8_t out[kFrameLen]);
void encode_session_status(const session_status &in, uint8_t out[kFrameLen]);
void encode_transaction_status(const transaction_status &in, uint8_t out[kFrameLen]);

decode_error decode_request(const uint8_t *data, size_t len, request &out);

/* Peeks the kind so a caller can pick the right decoder without parsing twice. Returns bad_length or
 * bad_version before looking at byte 1. */
decode_error decode_status_kind(const uint8_t *data, size_t len, status_kind &out);
decode_error decode_session_status(const uint8_t *data, size_t len, session_status &out);
decode_error decode_transaction_status(const uint8_t *data, size_t len, transaction_status &out);

} // namespace lexxhard::tof_commission_wire

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
