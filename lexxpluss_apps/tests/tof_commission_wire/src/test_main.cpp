/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The commissioning downlink codec and the internal-to-wire mapper, against fixed byte vectors.
 *
 * GOLDEN MEANS BYTE-EXACT. Every positive case asserts the actual eight bytes rather than a
 * round trip, because a round trip through one implementation agrees with itself no matter what
 * layout it chose. The vectors here are what the draft specifies, and they are the thing a second
 * implementation on the host side has to match.
 *
 * NOT HERE, DELIBERATELY: stale tokens, sequence conflicts, caching and budgets. Those belong to the
 * protocol state machine, which is a separate layer -- a codec that knew about them could refuse a
 * frame for a reason its caller cannot see.
 *
 * NOT FROZEN. The draft is a draft, the identifiers are symbolic, and nothing here is wired to a CAN
 * filter.
 */

#include <zephyr/ztest.h>

#include <string.h>

#include "tof_commission_map.hpp"
#include "tof_commission_wire.hpp"

namespace wire = lexxhard::tof_commission_wire;
namespace map = lexxhard::tof_commission_map;
namespace cm = lexxhard::tof_commissioning;
namespace au = lexxhard::tof_authority;
namespace pf = lexxhard::tof_proof;

namespace {

void expect_bytes(const uint8_t *got, const uint8_t *want, const char *what)
{
    for (size_t i = 0; i < wire::kFrameLen; ++i)
        zassert_equal(got[i], want[i], "%s: byte %u is 0x%02x, expected 0x%02x", what,
                      static_cast<unsigned>(i), got[i], want[i]);
}

} // namespace

ZTEST_SUITE(tof_commission_wire, NULL, NULL, NULL, NULL, NULL);

/* ---- request, byte for byte ---- */

ZTEST(tof_commission_wire, test_a_prove_and_start_request_encodes_exactly)
{
    wire::request r{};
    r.raw_op = static_cast<uint8_t>(wire::opcode::prove_and_start);
    r.seq = 0x2a;
    r.wire_epoch = 0x07;
    r.session_token = 0xdeadbeefU;

    uint8_t out[wire::kFrameLen]{};
    wire::encode_request(r, out);

    /* version, opcode, seq, epoch, then the token little-endian. */
    const uint8_t want[wire::kFrameLen]{0x01, 0x01, 0x2a, 0x07, 0xef, 0xbe, 0xad, 0xde};
    expect_bytes(out, want, "prove_and_start");
}

ZTEST(tof_commission_wire, test_a_start_only_request_differs_only_in_the_opcode)
{
    wire::request r{};
    r.raw_op = static_cast<uint8_t>(wire::opcode::start_only);
    r.seq = 0x2a;
    r.wire_epoch = 0x07;
    r.session_token = 0xdeadbeefU;

    uint8_t out[wire::kFrameLen]{};
    wire::encode_request(r, out);
    const uint8_t want[wire::kFrameLen]{0x01, 0x02, 0x2a, 0x07, 0xef, 0xbe, 0xad, 0xde};
    expect_bytes(out, want, "start_only");
}

ZTEST(tof_commission_wire, test_a_request_decodes_to_what_was_encoded)
{
    const uint8_t frame[wire::kFrameLen]{0x01, 0x02, 0xff, 0x00, 0x01, 0x00, 0x00, 0x80};
    wire::request got{};
    zassert_equal(wire::decode_request(frame, sizeof frame, got), wire::decode_error::none, "decodes");
    zassert_equal(got.version, 1, "version");
    zassert_equal(got.raw_op, static_cast<uint8_t>(wire::opcode::start_only),
                  "the opcode byte is carried out raw");
    zassert_equal(got.seq, 0xff, "seq is carried, not interpreted -- 0xff is an ordinary identity");
    zassert_equal(got.wire_epoch, 0x00, "epoch 0 is legitimate");
    zassert_equal(got.session_token, 0x80000001U, "token, little-endian");
}

/* ---- malformed ---- */

ZTEST(tof_commission_wire, test_a_frame_that_is_not_eight_bytes_is_refused_whole)
{
    const uint8_t frame[wire::kFrameLen]{0x01, 0x01, 0x2a, 0x07, 0x00, 0x00, 0x00, 0x00};
    wire::request got{};
    got.seq = 0x11;

    static const size_t lengths[]{0, 7, 9};
    for (const size_t len : lengths)
        zassert_equal(wire::decode_request(frame, len, got), wire::decode_error::bad_length,
                      "length %u is refused", static_cast<unsigned>(len));

    /* And nothing was taken from it. A salvaged short frame would carry an untrustworthy sequence
     * number into the table whose whole job is to make sequence numbers trustworthy. */
    zassert_equal(got.seq, 0x11, "the caller's struct is untouched");
    zassert_equal(wire::decode_request(nullptr, wire::kFrameLen, got), wire::decode_error::bad_length,
                  "a null pointer is a length problem, not a crash");
}

ZTEST(tof_commission_wire, test_an_unknown_version_stops_at_byte_zero)
{
    const uint8_t frame[wire::kFrameLen]{0x02, 0x01, 0x2a, 0x07, 0x00, 0x00, 0x00, 0x00};
    wire::request got{};
    got.seq = 0x11;
    zassert_equal(wire::decode_request(frame, sizeof frame, got), wire::decode_error::bad_version,
                  "version 2 is not this build's");
    zassert_equal(got.seq, 0x11, "and nothing past byte 0 was interpreted");
}

ZTEST(tof_commission_wire, test_an_unknown_opcode_is_carried_out_not_refused_here)
{
    /* THE CODEC IS NOT WHERE AN OPCODE IS JUDGED, and this test exists to keep it that way. The
     * specified order is length, version, session token, opcode -- so refusing here would decide an
     * opcode before anyone had established the frame belongs to this session, and a frame from a
     * previous boot would be answered `bad_opcode` when the true answer is `stale_session`. An
     * earlier version did refuse here, which is exactly the order the draft rules out. */
    static const uint8_t opcodes[]{0x00, 0x03, 0xff};
    for (const uint8_t op : opcodes) {
        const uint8_t frame[wire::kFrameLen]{0x01, op, 0x2a, 0x07, 0x00, 0x00, 0x00, 0x00};
        wire::request got{};
        zassert_equal(wire::decode_request(frame, sizeof frame, got), wire::decode_error::none,
                      "opcode 0x%02x decodes", op);
        zassert_equal(got.raw_op, op, "and is carried out unchanged");
        zassert_false(wire::is_known_opcode(op), "while is_known_opcode() says it is not ours");
        zassert_equal(got.seq, 0x2a, "the rest of the frame is still parsed");
    }

    static const uint8_t known[]{0x01, 0x02};
    for (const uint8_t op : known)
        zassert_true(wire::is_known_opcode(op), "opcode 0x%02x is ours", op);
}

ZTEST(tof_commission_wire, test_an_unknown_opcode_round_trips_through_the_encoder)
{
    /* Encoding a raw opcode the codec does not know is not a fault either: the host-side encoder and
     * the firmware-side decoder share this file, and a test harness that wants to put an unknown
     * opcode on the wire is the only way the state machine's bad_opcode path can be exercised at
     * all. */
    wire::request r{};
    r.raw_op = 0x7f;
    r.seq = 0x2a;
    r.wire_epoch = 0x07;
    r.session_token = 0xdeadbeefU;

    uint8_t out[wire::kFrameLen]{};
    wire::encode_request(r, out);
    const uint8_t want[wire::kFrameLen]{0x01, 0x7f, 0x2a, 0x07, 0xef, 0xbe, 0xad, 0xde};
    expect_bytes(out, want, "unknown opcode");
}

/* ---- status, both kinds ---- */

ZTEST(tof_commission_wire, test_a_session_frame_encodes_exactly)
{
    wire::session_status s{};
    s.session_token = 0x01020304U;
    s.profile_enabled = true;
    s.transaction_in_progress = false;

    uint8_t out[wire::kFrameLen]{};
    wire::encode_session_status(s, out);
    const uint8_t want[wire::kFrameLen]{0x01, 0x00, 0x04, 0x03, 0x02, 0x01, 0x01, 0x00};
    expect_bytes(out, want, "session");

    s.transaction_in_progress = true;
    wire::encode_session_status(s, out);
    const uint8_t want2[wire::kFrameLen]{0x01, 0x00, 0x04, 0x03, 0x02, 0x01, 0x03, 0x00};
    expect_bytes(out, want2, "session, transaction in progress");
}

ZTEST(tof_commission_wire, test_a_transaction_frame_encodes_exactly)
{
    wire::transaction_status t{};
    t.seq = 0x2a;
    t.wire_epoch = 0x07;
    t.ph = wire::phase::refused;
    t.res = wire::result::busy_at_commit;
    t.stage = wire::wire_stage::commit;
    t.detail = wire::wire_detail::chain_busy;

    uint8_t out[wire::kFrameLen]{};
    wire::encode_transaction_status(t, out);
    const uint8_t want[wire::kFrameLen]{0x01, 0x01, 0x2a, 0x07, 0x05, 0x10, 0x08, 0x01};
    expect_bytes(out, want, "transaction");
}

ZTEST(tof_commission_wire, test_the_kind_byte_selects_the_decoder)
{
    const uint8_t session[wire::kFrameLen]{0x01, 0x00, 0x04, 0x03, 0x02, 0x01, 0x01, 0x00};
    wire::status_kind kind{};
    zassert_equal(wire::decode_status_kind(session, sizeof session, kind), wire::decode_error::none, "kind");
    zassert_true(kind == wire::status_kind::session, "session");

    wire::transaction_status t{};
    zassert_equal(wire::decode_transaction_status(session, sizeof session, t),
                  wire::decode_error::bad_kind, "a session frame is not a transaction frame");

    const uint8_t bad_kind[wire::kFrameLen]{0x01, 0x02, 0, 0, 0, 0, 0, 0};
    zassert_equal(wire::decode_status_kind(bad_kind, sizeof bad_kind, kind),
                  wire::decode_error::bad_kind, "kind 2 is not defined");
}

ZTEST(tof_commission_wire, test_a_nonzero_reserved_byte_is_refused_not_ignored)
{
    const uint8_t frame[wire::kFrameLen]{0x01, 0x00, 0x04, 0x03, 0x02, 0x01, 0x01, 0x99};
    wire::session_status s{};
    zassert_equal(wire::decode_session_status(frame, sizeof frame, s),
                  wire::decode_error::reserved_not_zero,
                  "ignoring it is how a later version's new field is silently discarded");
}

ZTEST(tof_commission_wire, test_an_unknown_stage_or_detail_decodes_rather_than_failing)
{
    /* The decoder must not refuse a frame from a newer firmware: refusing it drops the very frame
     * that says a newer firmware is there. Rendering an unrecognised value as unknown is the
     * caller's job, not the codec's. */
    const uint8_t frame[wire::kFrameLen]{0x01, 0x01, 0x2a, 0x07, 0x04, 0x00, 0x7b, 0x7c};
    wire::transaction_status t{};
    zassert_equal(wire::decode_transaction_status(frame, sizeof frame, t), wire::decode_error::none,
                  "it decodes");
    zassert_equal(static_cast<uint8_t>(t.stage), 0x7b, "carrying the value it was sent");
    zassert_equal(static_cast<uint8_t>(t.detail), 0x7c, "and the detail too");
}

/* ---- the mapper: every enumerator of all four enums ---- */

ZTEST(tof_commission_wire, test_every_commissioning_stage_maps)
{
    struct row { cm::stage s; wire::result res; wire::wire_stage st; wire::wire_detail d; };
    static const row rows[]{
        {cm::stage::none, wire::result::ok, wire::wire_stage::complete, wire::wire_detail::none},
        {cm::stage::not_configured, wire::result::misconfigured, wire::wire_stage::not_started, wire::wire_detail::none},
        {cm::stage::epoch_out_of_range, wire::result::internal_error, wire::wire_stage::not_started, wire::wire_detail::none},
        {cm::stage::quiesce_failed, wire::result::busy_chain, wire::wire_stage::quiesce, wire::wire_detail::chain_busy},
        {cm::stage::chain_busy, wire::result::busy_chain, wire::wire_stage::quiesce, wire::wire_detail::chain_busy},
        {cm::stage::attempt_refused, wire::result::proof_failed, wire::wire_stage::proof_evaluation, wire::wire_detail::commit_refused},
        {cm::stage::evidence_refused, wire::result::proof_failed, wire::wire_stage::proof_evaluation, wire::wire_detail::walk_mismatch},
        {cm::stage::proof_speed_refused, wire::result::proof_failed, wire::wire_stage::first_walk, wire::wire_detail::proof_speed_refused},
        {cm::stage::product_speed_refused, wire::result::proof_failed, wire::wire_stage::retime, wire::wire_detail::product_speed_refused},
        {cm::stage::identity_recheck_failed, wire::result::proof_failed, wire::wire_stage::identity_recheck, wire::wire_detail::identity_disagreed},
        {cm::stage::commit_refused, wire::result::proof_failed, wire::wire_stage::commit, wire::wire_detail::commit_refused},
    };
    zassert_equal(sizeof rows / sizeof rows[0], 11u, "all eleven stages, counted so a new one shows up here too");
    for (const auto &r : rows) {
        const map::outcome o{map::map_stage(r.s)};
        zassert_true(o.res == r.res, "stage %u result", static_cast<unsigned>(r.s));
        zassert_true(o.stage == r.st, "stage %u wire stage", static_cast<unsigned>(r.s));
        zassert_true(o.detail == r.d, "stage %u detail", static_cast<unsigned>(r.s));
    }
}

ZTEST(tof_commission_wire, test_every_begin_refusal_maps)
{
    zassert_true(map::map_begin_refusal(au::begin_refusal::none).res == wire::result::internal_error,
                 "unreachable with attempt_refused, and mapped anyway");
    zassert_true(map::map_begin_refusal(au::begin_refusal::not_initialised).res == wire::result::misconfigured, "");
    const map::outcome busy{map::map_begin_refusal(au::begin_refusal::acquisition_not_idle)};
    zassert_true(busy.res == wire::result::busy_chain, "before the transaction, so busy_chain");
    zassert_true(busy.stage == wire::wire_stage::quiesce, "");
}

ZTEST(tof_commission_wire, test_every_commit_refusal_maps)
{
    struct row { au::commit_refusal c; wire::result res; };
    static const row rows[]{
        {au::commit_refusal::none, wire::result::ok},
        {au::commit_refusal::not_initialised, wire::result::misconfigured},
        {au::commit_refusal::no_attempt, wire::result::proof_failed},
        {au::commit_refusal::invalid_token, wire::result::proof_failed},
        {au::commit_refusal::wrong_attempt, wire::result::proof_failed},
        {au::commit_refusal::not_commissioning_profile, wire::result::misconfigured},
        {au::commit_refusal::runtime_mapping_mismatch, wire::result::proof_failed},
        {au::commit_refusal::epoch_reused, wire::result::epoch_reused},
        {au::commit_refusal::epoch_space_exhausted, wire::result::attempts_exhausted},
        {au::commit_refusal::acquisition_busy, wire::result::busy_at_commit},
        {au::commit_refusal::epoch_install_failed, wire::result::proof_failed},
        {au::commit_refusal::mapping_install_failed, wire::result::proof_failed},
    };
    zassert_equal(sizeof rows / sizeof rows[0], 12u, "all twelve");
    for (const auto &r : rows)
        zassert_true(map::map_commit_refusal(r.c).res == r.res, "commit refusal %u",
                     static_cast<unsigned>(r.c));

    /* The two the host must tell apart: one means try another ordinal, the other means this boot has
     * no epochs left and a new ordinal will not help. */
    zassert_false(map::map_commit_refusal(au::commit_refusal::epoch_reused).res ==
                      map::map_commit_refusal(au::commit_refusal::epoch_space_exhausted).res,
                  "epoch_reused and epoch_space_exhausted are not the same answer");
}

ZTEST(tof_commission_wire, test_every_proof_refusal_maps)
{
    struct row { pf::refusal p; wire::result res; wire::wire_stage st; };
    static const row rows[]{
        {pf::refusal::none, wire::result::internal_error, wire::wire_stage::proof_evaluation},
        {pf::refusal::missing_evidence, wire::result::internal_error, wire::wire_stage::proof_evaluation},
        {pf::refusal::challenge_invalid, wire::result::internal_error, wire::wire_stage::proof_evaluation},
        {pf::refusal::challenge_stale, wire::result::internal_error, wire::wire_stage::proof_evaluation},
        {pf::refusal::challenge_consumed, wire::result::internal_error, wire::wire_stage::proof_evaluation},
        {pf::refusal::spec_not_commissioning_profile, wire::result::misconfigured, wire::wire_stage::not_started},
        {pf::refusal::spec_no_tail_l4, wire::result::misconfigured, wire::wire_stage::not_started},
        {pf::refusal::spec_no_cliff, wire::result::misconfigured, wire::wire_stage::not_started},
        {pf::refusal::spec_too_few_positions, wire::result::misconfigured, wire::wire_stage::not_started},
        {pf::refusal::walk_position_count, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::walk_spec_rejected, wire::result::misconfigured, wire::wire_stage::first_walk},
        {pf::refusal::walk1_not_complete, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::walk2_not_complete, wire::result::proof_failed, wire::wire_stage::second_walk},
        {pf::refusal::position_not_verified, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::l4_retained, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::address_mismatch, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::address_not_distinct, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::identity_mismatch, wire::result::proof_failed, wire::wire_stage::first_walk},
        {pf::refusal::role_unknown, wire::result::misconfigured, wire::wire_stage::proof_evaluation},
        {pf::refusal::role_duplicate, wire::result::proof_failed, wire::wire_stage::proof_evaluation},
        {pf::refusal::fingerprint_mismatch, wire::result::proof_failed, wire::wire_stage::second_walk},
        {pf::refusal::isolation_not_attempted, wire::result::internal_error, wire::wire_stage::tail_isolation},
        {pf::refusal::isolation_transport_error, wire::result::proof_failed, wire::wire_stage::tail_isolation},
        {pf::refusal::isolation_no_answer, wire::result::proof_failed, wire::wire_stage::tail_isolation},
        {pf::refusal::isolation_wrong_address, wire::result::proof_failed, wire::wire_stage::tail_isolation},
        {pf::refusal::isolation_identity, wire::result::proof_failed, wire::wire_stage::tail_isolation},
        {pf::refusal::isolation_prev_addr_wrong, wire::result::proof_failed, wire::wire_stage::tail_isolation},
        {pf::refusal::isolation_prev_answered, wire::result::proof_failed, wire::wire_stage::tail_isolation},
    };
    zassert_equal(sizeof rows / sizeof rows[0], 28u, "all twenty-eight");
    for (const auto &r : rows) {
        const map::outcome o{map::map_proof_refusal(r.p)};
        zassert_true(o.res == r.res, "proof refusal %u result", static_cast<unsigned>(r.p));
        zassert_true(o.stage == r.st, "proof refusal %u stage", static_cast<unsigned>(r.p));
    }
}

ZTEST(tof_commission_wire, test_the_mapper_never_emits_unknown)
{
    /* unknown is a decoder-side rendering for a value a NEWER firmware sent. A mapper that could
     * produce it would be shipping a hole rather than failing to build. */
    for (uint8_t i = 0; i <= static_cast<uint8_t>(cm::stage::commit_refused); ++i) {
        const map::outcome o{map::map_stage(static_cast<cm::stage>(i))};
        zassert_false(o.stage == wire::wire_stage::unknown, "stage %u", i);
        zassert_false(o.detail == wire::wire_detail::unknown, "stage %u", i);
    }
    for (uint8_t i = 0; i <= static_cast<uint8_t>(pf::refusal::isolation_prev_answered); ++i) {
        const map::outcome o{map::map_proof_refusal(static_cast<pf::refusal>(i))};
        zassert_false(o.stage == wire::wire_stage::unknown, "proof refusal %u", i);
        zassert_false(o.detail == wire::wire_detail::unknown, "proof refusal %u", i);
    }
}

ZTEST(tof_commission_wire, test_map_result_consults_the_right_sub_enum)
{
    cm::outcome r{};
    r.failed_at = cm::stage::commit_refused;
    r.commit = au::commit_refusal::epoch_reused;
    /* A stale value in an unrelated field must not decide the answer. */
    r.proof = pf::refusal::fingerprint_mismatch;
    r.begin = au::begin_refusal::acquisition_not_idle;
    zassert_true(map::map_result(r).res == wire::result::epoch_reused,
                 "commit_refused consults the commit refusal and nothing else");

    r.failed_at = cm::stage::evidence_refused;
    zassert_true(map::map_result(r).res == wire::result::proof_failed, "evidence_refused consults the proof");
    zassert_true(map::map_result(r).stage == wire::wire_stage::second_walk, "and its stage");

    r.failed_at = cm::stage::attempt_refused;
    zassert_true(map::map_result(r).res == wire::result::busy_chain, "attempt_refused consults begin");
}
