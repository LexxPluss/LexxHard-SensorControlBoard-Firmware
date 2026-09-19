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

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <string.h>

#include "tof_commission_session.hpp"
#include "tof_commission_wire.hpp"

namespace cs = lexxhard::tof_commission;
namespace wire = lexxhard::tof_commission_wire;
namespace cm = lexxhard::tof_commissioning;
namespace pf = lexxhard::tof_proof;

namespace {

/* Set while the prove hook is running, so a test can reach into the state machine from INSIDE the
 * transaction -- the one moment at which holding the lock across a proof would be visible. */
bool probe_lock_during_prove_{false};
bool probe_ran_{false};

/* Holds the prove hook open, so the race window is a state the test creates rather than one it hopes
 * to hit. `prove_entered` is given when a worker is inside the transaction with the lock released;
 * `prove_release` is what lets it finish. */
bool block_prove_{false};
K_SEM_DEFINE(prove_entered, 0, 1);
K_SEM_DEFINE(prove_release, 0, 1);

struct fakes {
    uint32_t token{0x11223344U};
    bool token_available{true};
    /* Draws that come back zero before the real one. Zero is reserved as "no token", and a single
     * zero from a healthy generator is an ordinary sample rather than a fault. */
    int zero_draws{0};
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
    if (f_.zero_draws > 0) {
        --f_.zero_draws;
        *out = 0;
        return 0;
    }
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
    if (probe_lock_during_prove_) {
        /* announcement() takes the same spinlock the worker takes. With CONFIG_SPIN_VALIDATE the
         * kernel asserts on a recursive take, so if the worker ever held the lock across the
         * transaction this call would fail the test instead of quietly starving the CAN callback for
         * the length of an enumeration. */
        (void)cs::announcement();
        probe_ran_ = true;
    }
    if (block_prove_) {
        /* ONLY THE FIRST PROOF WAITS. A second one means the claim failed, and it must return rather
         * than block: the test should then fail saying that two proofs ran, which is the defect,
         * instead of timing out on a join, which is only a symptom of it. */
        block_prove_ = false;
        k_sem_give(&prove_entered);
        k_sem_take(&prove_release, K_FOREVER);
    }
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

void build_raw_request(uint8_t out[wire::kFrameLen], uint8_t seq, uint8_t epoch, uint32_t token,
                       uint8_t raw_op)
{
    wire::request r{};
    r.raw_op = raw_op;
    r.seq = seq;
    r.wire_epoch = epoch;
    r.session_token = token;
    wire::encode_request(r, out);
}

void build_request(uint8_t out[wire::kFrameLen], uint8_t seq, uint8_t epoch, uint32_t token,
                   wire::opcode op = wire::opcode::prove_and_start)
{
    build_raw_request(out, seq, epoch, token, static_cast<uint8_t>(op));
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
    probe_lock_during_prove_ = false;
    probe_ran_ = false;
    block_prove_ = false;
    k_sem_reset(&prove_entered);
    k_sem_reset(&prove_release);
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

ZTEST(tof_commission_session, test_a_single_zero_draw_is_retried_rather_than_fatal)
{
    /* One zero is a sample, not a verdict on the entropy source. Refusing on it would turn a healthy
     * generator into a board with no session roughly once in 2^32 boots, for no reason. */
    f_.zero_draws = 1;
    zassert_true(enable(), "the second draw is taken");
    zassert_true(cs::has_session(), "and it is a session");
    zassert_equal(f_.draws, 2, "exactly one retry");
    zassert_equal(cs::session_token(), 0x11223344U, "with the non-zero token");

    /* Two in a row is treated as no entropy at all -- one retry, not a loop. */
    f_ = fakes{};
    f_.zero_draws = 2;
    zassert_false(enable(), "refused");
    zassert_equal(f_.draws, 2, "and it did not keep drawing");
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

/* ---- the opcode boundary ---- */

ZTEST(tof_commission_session, test_start_only_without_a_proof_starts_nothing_and_proves_nothing)
{
    /* THE MOST IMPORTANT BOUNDARY IN THE PROTOCOL. `start_only` promises it re-enumerates nothing.
     * The sequencer has one entry point and decides from its own state, so a start_only that reached
     * it while nothing was proven would run a FULL PROOF and report success -- and an earlier version
     * did exactly that, because the worker never read the opcode. */
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U, wire::opcode::start_only);

    zassert_true(cs::handle_request(frame, sizeof frame).queued, "accepted for the worker");
    const wire::transaction_status t{run_to_terminal()};

    zassert_true(t.ph == wire::phase::refused, "refused");
    zassert_true(t.res == wire::result::epoch_mismatch,
                 "nothing is proven, so no epoch is the proven one");
    zassert_equal(f_.proves, 0, "AND NOT ONE PROOF RAN");
    zassert_equal(f_.starts, 0, "and acquisition was not started either");
}

ZTEST(tof_commission_session, test_start_only_with_the_proven_epoch_starts_without_re_proving)
{
    /* The case the opcode exists for: the proof succeeded and the start did not. */
    zassert_true(enable(), "");
    f_.start_rc = -1;

    uint8_t first[wire::kFrameLen]{};
    build_request(first, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(first, sizeof first).queued, "");
    zassert_true(run_to_terminal().res == wire::result::start_failed, "the start refused");
    zassert_equal(f_.proves, 1, "but the mapping is proven");

    f_.start_rc = 0;
    uint8_t retry[wire::kFrameLen]{};
    build_request(retry, 2, 7, 0x11223344U, wire::opcode::start_only);
    zassert_true(cs::handle_request(retry, sizeof retry).queued, "");
    const wire::transaction_status t{run_to_terminal()};

    zassert_true(t.ph == wire::phase::done, "it completed");
    zassert_true(t.res == wire::result::ok, "");
    zassert_equal(f_.proves, 1, "and the chain was NOT re-enumerated");
    zassert_equal(f_.starts, 2, "only the start was retried");
}

ZTEST(tof_commission_session, test_start_only_for_an_epoch_that_is_not_the_proven_one_is_refused)
{
    zassert_true(enable(), "");
    f_.start_rc = -1;
    uint8_t first[wire::kFrameLen]{};
    build_request(first, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(first, sizeof first).queued, "");
    zassert_true(run_to_terminal().res == wire::result::start_failed, "");

    f_.start_rc = 0;
    uint8_t wrong[wire::kFrameLen]{};
    build_request(wrong, 2, 8, 0x11223344U, wire::opcode::start_only);
    zassert_true(cs::handle_request(wrong, sizeof wrong).queued, "");
    const wire::transaction_status t{run_to_terminal()};

    zassert_true(t.res == wire::result::epoch_mismatch, "epoch 8 is not the epoch that was proven");
    zassert_equal(t.wire_epoch, 8, "answered against the epoch the host asked about");
    zassert_equal(f_.proves, 1, "it did not escalate to a proof of epoch 8");
    zassert_equal(f_.starts, 1, "and it did not start the mapping under the wrong epoch either");
}

ZTEST(tof_commission_session, test_a_request_for_a_new_epoch_after_a_proof_is_not_reported_as_done)
{
    /* THE ACCOUNTING LIE THIS EXISTS TO PREVENT. Once a proof is held the sequencer proves nothing
     * else this boot: stepped again it reports already_started without touching the chain. Reporting
     * that as done/ok would tell the host that epoch 8 was accepted while epoch 7 is what is
     * installed -- and the host's persisted `accepted` field cannot recover from that. */
    zassert_true(enable(), "");
    uint8_t first[wire::kFrameLen]{};
    build_request(first, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(first, sizeof first).queued, "");
    zassert_true(run_to_terminal().res == wire::result::ok, "epoch 7 is commissioned");

    uint8_t newer[wire::kFrameLen]{};
    build_request(newer, 2, 8, 0x11223344U);
    zassert_true(cs::handle_request(newer, sizeof newer).queued, "");
    const wire::transaction_status t{run_to_terminal()};
    zassert_true(t.res == wire::result::epoch_mismatch, "not ok");
    zassert_true(t.ph == wire::phase::refused, "and not done");
    zassert_equal(f_.proves, 1, "nothing re-proved");

    /* And it does not over-refuse: the epoch that IS running is still answered as done, because a
     * host retransmitting under a new sequence has got what it asked for. */
    uint8_t same[wire::kFrameLen]{};
    build_request(same, 3, 7, 0x11223344U);
    zassert_true(cs::handle_request(same, sizeof same).queued, "");
    const wire::transaction_status ok{run_to_terminal()};
    zassert_true(ok.ph == wire::phase::done, "the running epoch is done");
    zassert_true(ok.res == wire::result::ok, "");
    zassert_equal(f_.proves, 1, "still nothing re-proved");
}

/* ---- busy is a cached answer, not a recomputed one ---- */

ZTEST(tof_commission_session, test_a_busy_request_retransmitted_after_the_transaction_ends_replays_busy)
{
    /* An earlier version checked `running_` BEFORE claiming a table entry, so the busy answer left
     * nothing behind -- and the identical frame, arriving again once the chain was free, was treated
     * as a brand new request and RAN a transaction the host had already been refused. */
    zassert_true(enable(5, 5), "");
    uint8_t running[wire::kFrameLen]{}, busy[wire::kFrameLen]{};
    build_request(running, 1, 11, 0x11223344U);
    build_request(busy, 2, 12, 0x11223344U);

    zassert_true(cs::handle_request(running, sizeof running).queued, "");
    zassert_true(cs::handle_request(busy, sizeof busy).status.res == wire::result::busy_chain,
                 "refused while one is in flight");
    (void)run_to_terminal();
    zassert_equal(f_.proves, 1, "one transaction ran");

    const uint32_t replays_before{cs::stats().replayed_from_table};
    const cs::rx_action again{cs::handle_request(busy, sizeof busy)};
    zassert_true(again.send_status, "answered");
    zassert_true(again.status.res == wire::result::busy_chain, "with the SAME answer as before");
    zassert_false(again.queued, "not queued");
    zassert_equal(f_.proves, 1, "and no second transaction ran");
    zassert_equal(cs::stats().replayed_from_table, replays_before + 1, "replayed from the table");
}

/* ---- the opcode is judged after the token, never before ---- */

ZTEST(tof_commission_session, test_an_unknown_opcode_is_refused_but_only_after_the_token)
{
    zassert_true(enable(), "");

    uint8_t stale[wire::kFrameLen]{};
    build_raw_request(stale, 1, 7, 0xDEADBEEFU, 0x7f);
    const cs::rx_action a{cs::handle_request(stale, sizeof stale)};
    zassert_true(a.status.res == wire::result::stale_session,
                 "a frame from a finished boot is stale first, whatever its opcode");

    uint8_t live[wire::kFrameLen]{};
    build_raw_request(live, 1, 7, 0x11223344U, 0x7f);
    const cs::rx_action b{cs::handle_request(live, sizeof live)};
    zassert_true(b.status.res == wire::result::bad_opcode, "and only then judged on its opcode");
    zassert_false(b.queued, "");
    zassert_equal(f_.proves, 0, "nothing ran");

    /* And, like `disabled`, it left no entry: the check sits before the table. */
    uint8_t good[wire::kFrameLen]{};
    build_request(good, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(good, sizeof good).queued, "sequence 1 is still free");
}

/* ---- phases ---- */

ZTEST(tof_commission_session, test_the_worker_emits_one_terminal_status_and_no_intermediate_phase)
{
    /* `proving`, `proven` and `starting` are OPTIONAL DIAGNOSTICS in the wire enumeration and this
     * firmware emits none of them: the transaction is a single blocking call with no observable
     * interior. Pinned as a test so a host is never written against a progress frame that does not
     * exist. */
    zassert_true(enable(), "");
    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "");

    int statuses{0};
    wire::transaction_status last{};
    for (int i = 0; i < 10; ++i) {
        const cs::worker_result w{cs::worker_step()};
        if (w.send_status) {
            ++statuses;
            last = w.status;
            zassert_true(wire::is_terminal(w.status.ph), "every frame the worker sends is terminal");
        }
        if (w.state == cs::worker_state::idle)
            break;
    }
    zassert_equal(statuses, 1, "exactly one status frame for one transaction");
    zassert_true(last.ph == wire::phase::done, "and it is the outcome");
}

/* ---- the lock is short, and never held across the work ---- */

ZTEST(tof_commission_session, test_the_lock_is_not_held_across_the_transaction)
{
    /* RX runs in the CAN callback and the transaction takes hundreds of milliseconds. The worker
     * therefore takes the lock to pick the job up, RELEASES it, runs the proof, and takes it again to
     * publish the terminal status. This test enters the state machine from inside the proof hook,
     * which is exactly where a lock held across the work would show up. */
    zassert_true(enable(), "");
    probe_lock_during_prove_ = true;

    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "");
    const wire::transaction_status t{run_to_terminal()};

    zassert_true(probe_ran_, "the probe really ran inside the transaction");
    zassert_true(t.ph == wire::phase::done, "and the transaction still completed normally");
}

/* ---- two workers, one job ---- */

namespace {

K_THREAD_STACK_DEFINE(worker_a_stack, 4096);
K_THREAD_STACK_DEFINE(worker_b_stack, 4096);
struct k_thread worker_a_thread;
struct k_thread worker_b_thread;

cs::worker_result result_a_{};
cs::worker_result result_b_{};

void run_worker_a(void *, void *, void *)
{
    result_a_ = cs::worker_step();
}

void run_worker_b(void *, void *, void *)
{
    result_b_ = cs::worker_step();
}

} // namespace

ZTEST(tof_commission_session, test_a_second_worker_takes_nothing_and_runs_nothing)
{
    /* THE WINDOW IS REAL AND IS OPENED ON PURPOSE. The worker releases the lock for the transaction,
     * which is the whole point of the split -- so without a claim, two threads in worker_step() both
     * see the same queued job, both take it out of the table and both run it: one chain enumerated
     * twice under ONE host-issued epoch, and two terminal statuses for one sequence number. Here the
     * prove hook
     * blocks until the test lets it go, so the second worker is guaranteed to arrive while the first
     * is inside the transaction rather than when the scheduler happens to allow it. */
    zassert_true(enable(), "");
    block_prove_ = true;
    result_a_ = cs::worker_result{};
    result_b_ = cs::worker_result{};

    uint8_t frame[wire::kFrameLen]{};
    build_request(frame, 1, 7, 0x11223344U);
    zassert_true(cs::handle_request(frame, sizeof frame).queued, "queued");

    k_thread_create(&worker_a_thread, worker_a_stack, K_THREAD_STACK_SIZEOF(worker_a_stack),
                    run_worker_a, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);

    /* A is now inside prove(), holding the claim, with the spinlock released. */
    zassert_equal(k_sem_take(&prove_entered, K_SECONDS(5)), 0, "the first worker reached the proof");

    k_thread_create(&worker_b_thread, worker_b_stack, K_THREAD_STACK_SIZEOF(worker_b_stack),
                    run_worker_b, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    zassert_equal(k_thread_join(&worker_b_thread, K_SECONDS(5)), 0, "the second worker returned");

    /* It returned while the transaction was still running, and it did nothing. The count comes
     * first because it is the defect stated plainly: without the claim, this reads 2. */
    zassert_equal(f_.proves, 1, "the chain was NOT enumerated a second time");
    zassert_equal(f_.starts, 0, "and nothing was started by it");
    zassert_false(result_b_.send_status, "AND SENT NO STATUS: one request has one terminal answer");
    zassert_true(result_b_.state == cs::worker_state::running, "reported as running");

    k_sem_give(&prove_release);
    zassert_equal(k_thread_join(&worker_a_thread, K_SECONDS(5)), 0, "the first worker finished");

    zassert_equal(f_.proves, 1, "one proof");
    zassert_equal(f_.starts, 1, "one start");
    zassert_true(result_a_.send_status, "the worker that did the work sent the status");
    zassert_true(result_a_.status.ph == wire::phase::done, "which is the outcome");
    zassert_true(result_a_.state == cs::worker_state::idle, "and the slot is free again");

    /* And the answer is in the table, so a retransmission is still replayed rather than re-run --
     * the claim closed a race without touching idempotency. */
    const uint32_t replays_before{cs::stats().replayed_from_table};
    const cs::rx_action again{cs::handle_request(frame, sizeof frame)};
    zassert_true(again.send_status, "answered");
    zassert_true(again.status.ph == wire::phase::done, "from the table");
    zassert_equal(f_.proves, 1, "without running anything");
    zassert_equal(cs::stats().replayed_from_table, replays_before + 1, "counted as a replay");

    /* Nothing is claimed any more: the next step finds an empty slot rather than a job it may not
     * touch. */
    zassert_true(cs::worker_step().state == cs::worker_state::idle, "idle");
}
