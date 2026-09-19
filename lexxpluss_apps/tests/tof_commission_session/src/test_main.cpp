/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The protocol state machine: sessions, the validation order, idempotency and the per-boot budgets.
 *
 * The RNG is injected so the no-entropy path -- unreachable on a bench -- is a test rather than a
 * hope. The transaction is injected too, because what is under test is which requests are answered
 * and how, not what a walk means.
 */

#include <zephyr/ztest.h>

#include <string.h>

#include "tof_commission_session.hpp"
#include "tof_commission_wire.hpp"

namespace cs = lexxhard::tof_commission;
namespace wire = lexxhard::tof_commission_wire;
namespace cm = lexxhard::tof_commissioning;
namespace pf = lexxhard::tof_proof;

namespace {

struct fakes {
    uint32_t token{0x11223344U};
    bool token_available{true};
    bool permitted{true};
    int prove_rc{0};
    int start_rc{0};
    cm::stage prove_stage{cm::stage::none};
    pf::refusal prove_refusal{pf::refusal::none};

    int draws{0};
    int proves{0};
    int starts{0};
    uint32_t last_epoch{0xFFFFFFFFU};
};

fakes f_{};

int fake_draw(void *, uint32_t *out)
{
    ++f_.draws;
    if (!f_.token_available)
        return -1;
    *out = f_.token;
    return 0;
}

bool fake_permitted(void *)
{
    return f_.permitted;
}

int fake_prove(void *, uint32_t epoch, cm::outcome *out)
{
    ++f_.proves;
    f_.last_epoch = epoch;
    out->failed_at = f_.prove_stage;
    out->proof = f_.prove_refusal;
    return f_.prove_rc;
}

int fake_start(void *)
{
    ++f_.starts;
    return f_.start_rc;
}

cs::hooks wired()
{
    return cs::hooks{fake_draw, fake_permitted, fake_prove, fake_start, nullptr};
}

bool enable(uint8_t proofs = 3, uint8_t starts = 3, bool profile = true)
{
    cs::config c{};
    c.profile_enabled = profile;
    c.max_proof_attempts = proofs;
    c.max_start_attempts = starts;
    return cs::init(c, wired());
}

void build_request(uint8_t out[wire::kFrameLen], uint8_t seq, uint8_t epoch, uint32_t token,
                   wire::opcode op = wire::opcode::prove_and_start)
{
    wire::request r{};
    r.op = op;
    r.seq = seq;
    r.wire_epoch = epoch;
    r.session_token = token;
    wire::encode_request(r, out);
}

/* Drives the worker to a terminal status, as the real worker thread would. */
wire::transaction_status run_to_terminal()
{
    wire::transaction_status last{};
    for (int i = 0; i < 10; ++i) {
        const cs::worker_result w{cs::worker_step()};
        if (w.send_status)
            last = w.status;
        if (w.state == cs::worker_state::idle)
            break;
    }
    return last;
}

} // namespace

static void before(void *)
{
    f_ = fakes{};
}

ZTEST_SUITE(tof_commission_session, NULL, NULL, before, NULL, NULL);

/* ---- session ---- */

ZTEST(tof_commission_session, test_no_entropy_means_no_session_and_no_request_is_judged)
{
    f_.token_available = false;
    zassert_false(enable(), "init fails without a token");
    zassert_false(cs::has_session(), "and announces no session");

    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U);
    const cs::rx_action a{cs::handle_request(frame, sizeof frame)};
    zassert_true(a.send_status, "a well-formed request is answered");
    zassert_true(a.status.res == wire::result::no_session, "with no_session");
    zassert_false(a.send_session, "and no session frame, because there is none to announce");
    zassert_false(a.queued, "nothing is queued");
    zassert_equal(f_.proves, 0, "and nothing is proved");
}

ZTEST(tof_commission_session, test_a_zero_token_is_not_a_session)
{
    /* Zero is reserved as "no token": a draw that returns it is a draw that failed. */
    f_.token = 0;
    zassert_false(enable(), "zero is refused");
    zassert_false(cs::has_session(), "");
}

/* ---- validation order ---- */

ZTEST(tof_commission_session, test_a_short_frame_is_discarded_and_the_session_re_announced)
{
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 5, 7, 0x11223344U);

    const cs::rx_action a{cs::handle_request(frame, 7)};
    zassert_false(a.send_status, "no result frame: a result answers a request, and this is not one");
    zassert_true(a.send_session, "the session is re-announced instead");
    zassert_equal(cs::stats().discarded_bad_length, 1u, "counted");
}

ZTEST(tof_commission_session, test_an_unknown_version_is_discarded_the_same_way)
{
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 5, 7, 0x11223344U);
    frame[0] = 2;

    const cs::rx_action a{cs::handle_request(frame, sizeof frame)};
    zassert_false(a.send_status, "");
    zassert_true(a.send_session, "");
    zassert_equal(cs::stats().discarded_bad_version, 1u, "counted");
}

ZTEST(tof_commission_session, test_a_stale_token_is_refused_without_touching_the_table)
{
    zassert_true(enable(), "");
    uint8_t stale[wire::kFrameLen]{};
    build_request(stale, 9, 7, 0xDEADBEEFU);

    const cs::rx_action a{cs::handle_request(stale, sizeof stale)};
    zassert_true(a.status.res == wire::result::stale_session, "refused");
    zassert_true(a.send_session, "and the current session is sent so the host can resynchronise");
    zassert_equal(cs::stats().refused_stale_session, 1u, "counted");

    /* THE POINT: sequence 9 was NOT consumed by a frame from a finished boot. The live host may
     * still use it. */
    uint8_t live[wire::kFrameLen]{};
    build_request(live, 9, 7, 0x11223344U);
    const cs::rx_action b{cs::handle_request(live, sizeof live)};
    zassert_true(b.queued, "sequence 9 is still free for the live session");
    zassert_false(b.status.res == wire::result::seq_conflict, "not a conflict");
}

ZTEST(tof_commission_session, test_a_disabled_profile_refuses_before_the_table)
{
    zassert_true(enable(3, 3, false), "on, but the profile is off");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 3, 7, 0x11223344U);
    const cs::rx_action a{cs::handle_request(frame, sizeof frame)};
    zassert_true(a.status.res == wire::result::disabled, "refused");
    zassert_false(a.queued, "nothing queued");
    zassert_equal(f_.proves, 0, "and nothing proved");
}

/* ---- idempotency ---- */

ZTEST(tof_commission_session, test_an_exact_retransmission_replays_and_does_not_re_prove)
{
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 4, 7, 0x11223344U);

    zassert_true(cs::handle_request(frame, sizeof frame).queued, "accepted");
    const wire::transaction_status done{run_to_terminal()};
    zassert_true(done.ph == wire::phase::done, "it completed");
    zassert_equal(f_.proves, 1, "one proof");

    const cs::rx_action again{cs::handle_request(frame, sizeof frame)};
    zassert_true(again.send_status, "answered");
    zassert_true(again.status.ph == wire::phase::done, "from the table");
    zassert_false(again.queued, "not queued again");
    zassert_equal(f_.proves, 1, "and the chain is NOT re-enumerated");
    zassert_equal(cs::stats().replayed_from_table, 1u, "counted as a replay");
}

ZTEST(tof_commission_session, test_the_same_sequence_with_a_different_payload_is_a_conflict)
{
    zassert_true(enable(), "");
    uint8_t first[wire::kFrameLen]{};
    build_request(first, 4, 7, 0x11223344U);
    zassert_true(cs::handle_request(first, sizeof first).queued, "");
    (void)run_to_terminal();

    uint8_t different[wire::kFrameLen]{};
    build_request(different, 4, 8, 0x11223344U);  /* same seq, different epoch */
    const cs::rx_action a{cs::handle_request(different, sizeof different)};
    zassert_true(a.status.res == wire::result::seq_conflict, "refused");
    zassert_equal(f_.proves, 1, "and nothing ran");

    uint8_t other_op[wire::kFrameLen]{};
    build_request(other_op, 4, 7, 0x11223344U, wire::opcode::start_only);
    zassert_true(cs::handle_request(other_op, sizeof other_op).status.res == wire::result::seq_conflict,
                 "a different opcode is a different request too");
}

ZTEST(tof_commission_session, test_an_earlier_request_is_still_replayable_later)
{
    /* The reason the table holds every request of the session and not just the last one: a frame
     * delayed behind something else can arrive after a later request has already been answered.
     *
     * Both requests fail on purpose. A SUCCESSFUL commissioning is terminal for the boot -- the
     * sequencer reports already_started afterwards and proves nothing more, which is right and is
     * why this case has to be built from failures to exercise two transactions at all. */
    zassert_true(enable(5, 5), "");
    f_.prove_rc = -5;
    f_.prove_stage = cm::stage::evidence_refused;
    f_.prove_refusal = pf::refusal::fingerprint_mismatch;
    uint8_t a1[wire::kFrameLen]{}, a2[wire::kFrameLen]{};
    build_request(a1, 1, 11, 0x11223344U);
    build_request(a2, 2, 12, 0x11223344U);

    zassert_true(cs::handle_request(a1, sizeof a1).queued, "");
    (void)run_to_terminal();
    zassert_true(cs::handle_request(a2, sizeof a2).queued, "");
    (void)run_to_terminal();
    zassert_equal(f_.proves, 2, "two proofs");

    const cs::rx_action late{cs::handle_request(a1, sizeof a1)};
    zassert_true(late.send_status, "the older one is still answerable");
    zassert_equal(late.status.wire_epoch, 11, "with its own epoch");
    zassert_equal(f_.proves, 2, "and it did not run again");
}

ZTEST(tof_commission_session, test_the_table_refuses_when_full_rather_than_evicting)
{
    zassert_true(enable(255, 255), "");
    uint8_t frame[wire::kFrameLen]{};
    int accepted{0};
    for (int i = 0; i < 40; ++i) {
        build_request(frame, static_cast<uint8_t>(i + 1), static_cast<uint8_t>(i + 1), 0x11223344U);
        const cs::rx_action a{cs::handle_request(frame, sizeof frame)};
        if (a.status.res == wire::result::seq_space_exhausted)
            break;
        zassert_true(a.queued, "accepted while there is room");
        ++accepted;
        (void)run_to_terminal();
    }
    zassert_true(accepted > 0 && accepted < 40, "it filled up");

    /* And it stays refused: evicting would re-open replay of exactly the request evicted. */
    build_request(frame, 99, 99, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).status.res ==
                     wire::result::seq_space_exhausted,
                 "still refused");

    /* The earliest entry is still replayable, which is what eviction would have destroyed. Asserted
     * on the CONTENT, not merely that something came back: a refusal also sets send_status, so a
     * weaker check here passed against a store that had evicted entry 1 -- found by mutating the
     * eviction rule and watching this test stay green. */
    const uint32_t replays_before{cs::stats().replayed_from_table};
    build_request(frame, 1, 1, 0x11223344U);
    const cs::rx_action survivor{cs::handle_request(frame, sizeof frame)};
    zassert_true(survivor.send_status, "answered");
    zassert_equal(survivor.status.seq, 1, "it is request 1's own status");
    zassert_equal(survivor.status.wire_epoch, 1, "with request 1's epoch");
    zassert_true(survivor.status.ph == wire::phase::done, "replayed, not refused");
    zassert_equal(cs::stats().replayed_from_table, replays_before + 1,
                  "and counted as a replay rather than a fresh refusal");
}

/* ---- the RX path does no work ---- */

ZTEST(tof_commission_session, test_the_receive_path_never_proves)
{
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 4, 7, 0x11223344U);

    const cs::rx_action a{cs::handle_request(frame, sizeof frame)};
    zassert_true(a.queued, "queued");
    zassert_true(a.status.ph == wire::phase::accepted, "and acknowledged as accepted");
    zassert_equal(f_.proves, 0, "the transaction has NOT run in the callback");
    zassert_equal(f_.starts, 0, "");

    (void)run_to_terminal();
    zassert_equal(f_.proves, 1, "it runs on the worker");
}

ZTEST(tof_commission_session, test_a_second_request_while_running_is_busy)
{
    zassert_true(enable(), "");
    uint8_t a1[wire::kFrameLen]{}, a2[wire::kFrameLen]{};
    build_request(a1, 1, 11, 0x11223344U);
    build_request(a2, 2, 12, 0x11223344U);

    zassert_true(cs::handle_request(a1, sizeof a1).queued, "");
    const cs::rx_action b{cs::handle_request(a2, sizeof a2)};
    zassert_true(b.status.res == wire::result::busy_chain, "refused while one is in flight");
    zassert_false(b.queued, "");
    zassert_true(cs::announcement().transaction_in_progress, "and the announcement says so");
}

/* ---- budgets accumulate across requests ---- */

ZTEST(tof_commission_session, test_the_proof_budget_is_per_boot_not_per_sequence)
{
    zassert_true(enable(2, 3), "two proof attempts for this boot");
    f_.prove_rc = -5;
    f_.prove_stage = cm::stage::evidence_refused;
    f_.prove_refusal = pf::refusal::fingerprint_mismatch;

    uint8_t frame[wire::kFrameLen]{};
    for (uint8_t i = 1; i <= 4; ++i) {
        build_request(frame, i, static_cast<uint8_t>(10 + i), 0x11223344U);
        const cs::rx_action a{cs::handle_request(frame, sizeof frame)};
        if (!a.queued)
            break;
        (void)run_to_terminal();
    }

    /* THE POINT: a new sequence number does not buy a new budget. Two attempts, then exhausted --
     * otherwise a host retry loop turns a bounded retry into an unbounded one by accident. */
    zassert_equal(f_.proves, 2, "exactly the boot's budget, across different sequences");

    build_request(frame, 9, 99, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "a further request is accepted");
    const wire::transaction_status t{run_to_terminal()};
    zassert_true(t.res == wire::result::attempts_exhausted, "and told the budget is gone");
    zassert_equal(f_.proves, 2, "without proving again");
}

/* ---- outcomes reach the wire through the mapper ---- */

ZTEST(tof_commission_session, test_a_failed_proof_reports_its_stage_and_detail)
{
    zassert_true(enable(), "");
    f_.prove_rc = -5;
    f_.prove_stage = cm::stage::product_speed_refused;

    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "");
    const wire::transaction_status t{run_to_terminal()};

    zassert_true(t.ph == wire::phase::refused, "refused");
    zassert_true(t.res == wire::result::proof_failed, "");
    zassert_true(t.stage == wire::wire_stage::retime, "the stage the transaction reached");
    zassert_true(t.detail == wire::wire_detail::product_speed_refused, "and why it stopped there");
    zassert_equal(f_.starts, 0, "acquisition is never started after a failed proof");
}

ZTEST(tof_commission_session, test_a_failed_start_is_reported_apart_from_a_failed_proof)
{
    zassert_true(enable(), "");
    f_.start_rc = -1;

    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "");
    const wire::transaction_status t{run_to_terminal()};

    zassert_true(t.res == wire::result::start_failed, "not proof_failed");
    zassert_equal(f_.proves, 1, "the mapping WAS proven");
    zassert_true(t.stage == wire::wire_stage::acquisition_start, "and it got that far");
}

ZTEST(tof_commission_session, test_the_epoch_comes_from_the_request)
{
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 0xA5, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "");
    (void)run_to_terminal();
    zassert_equal(f_.last_epoch, 0xA5U, "the firmware never generates one");
}
