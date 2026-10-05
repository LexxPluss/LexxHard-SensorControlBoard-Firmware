/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
 * NOT FROZEN, AND ONE DECISION IS STILL OPEN. The layout below is version 1 as implemented; it is
 * not a contract. There is no generated artefact, neither repository pins anything in it, and
 * nothing here may be wired to a real CAN filter until there is one.
 *
 * The open decision, and the reason it blocks freezing, is recorded in section 11 of
 *   L4 -- CAN wire protocol
 *   https://app.notion.com/p/3b2a91d8f61581788558e171745f35e5
 * In short: a transaction status carries `seq` and `wire_epoch` and NO session identity, so a
 * refusal aimed at a transaction that is over is indistinguishable on the wire from a refusal of
 * whatever the host is waiting for now. Checking the pair is not a fix -- both are 8 bits, both
 * recycle, and the protocol puts no lifetime bound on a status frame. The rule that has to be
 * settled first is that a status must carry the session identity OF THE REQUEST IT ANSWERS, not the
 * SCB's current one; stamping the current token would leave the refusal path broken the same way.
 *
 * THIS FILE DOES NOT CLOSE THAT GAP and must not be read as having closed it. The layouts here are
 * the ones the session machine and its vectors are written against so that the behaviour can be
 * reviewed and tested now; the field budget that a session tag needs is in that section, with its
 * three candidates and their costs, and belongs to the revision that freezes this.
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

/* Byte 6 of the session status carries two flags and six RESERVED BITS, and those bits are
 * reserved-zero rather than ignored.
 *
 * The decoder used to read bits 0 and 1 and say nothing about the rest, in a frame whose byte 7 it
 * already refused for being non-zero -- so one reserved byte was enforced and the reserved bits
 * beside it were not. The choice is made the same way the request frame's is: ignoring a reserved
 * field is how a later version's new flag gets silently discarded by an older reader that believed
 * it understood the frame, and a reader that cannot see the flag cannot know it is acting on a
 * partial picture.
 *
 * The cost is stated rather than waved past: a newer firmware that sets bit 2 has its announcement
 * refused by an older host instead of half-understood. That is the intended direction, and the
 * version byte is checked first anyway, so a flag added with a version bump is refused for the
 * version and never reaches this check.
 *
 * WHY THIS IS NOT THE OPPOSITE OF decode_transaction_status(), which deliberately accepts a
 * `wire_stage` or `wire_detail` it does not recognise. Those are OPEN enumerations: they are
 * expected to grow, each value is advisory, and an unrecognised one has an honest rendering --
 * `unknown` -- so the frame is still worth delivering and refusing it would drop the very frame
 * saying a newer firmware is there. A reserved bit has no such rendering. Set, it says a field
 * exists that this reader cannot locate or name, next to two flags the host's behaviour depends on;
 * there is nothing to render and no way to act on a partial picture knowingly. Open values are
 * tolerated and structure is not. */
constexpr uint8_t kSessionFlagsMask{0x03};

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
