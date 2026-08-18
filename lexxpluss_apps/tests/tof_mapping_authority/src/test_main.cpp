/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for the cliff mapping authority.
 *
 * The authority is module-level state, so every test re-inits it. That is not a shortcut
 * around a design problem: there is exactly one cliff mapping on a robot, and making it an
 * instance so tests need no reset would invite a second one to exist.
 *
 * The epoch bitmap is the reason begin_epoch is injected. A test needs the cycle reset to
 * fail on demand, because "a failed reset leaves the state non-PROVEN" is the property that
 * keeps a half-committed transaction off the wire, and nothing else can produce it.
 */

#include <zephyr/ztest.h>

#include "tof_mapping_authority.hpp"

namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace au = lexxhard::tof_authority;
namespace acq = lexxhard::tof_acq;

namespace {

constexpr enm::id_bytes kL7Id{0xf0, 0x02};
constexpr enm::id_bytes kL4Id{0xeb, 0xaa};

int begin_epoch_rc{0};
int begin_epoch_calls{0};

int fake_begin_epoch()
{
    ++begin_epoch_calls;
    return begin_epoch_rc;
}

enm::chain_spec product_spec()
{
    enm::chain_spec s{};
    s.positions = 6;
    s.at[0] = {enm::model::l7cx, 0x2A, 0, enm::l4_role::unknown};
    s.at[1] = {enm::model::l7cx, 0x2B, 1, enm::l4_role::unknown};
    s.at[2] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    s.at[3] = {enm::model::l4cx, 0x2D, -1, enm::l4_role::rear_left};
    s.at[4] = {enm::model::l4cx, 0x2E, -1, enm::l4_role::rear_right};
    s.at[5] = {enm::model::l4cx, 0x2F, -1, enm::l4_role::front_right};
    s.alloff_pulses = 8;
    return s;
}

enm::chain_spec runtime_spec_storage{product_spec()};

enm::chain_result clean_walk(const enm::chain_spec &s, bool l7_retained)
{
    enm::chain_result r{};
    r.status = enm::chain_status::complete;
    r.spec = enm::spec_error::none;
    r.positions = s.positions;
    for (size_t i{0}; i < s.positions; ++i) {
        const bool l7{s.at[i].expected == enm::model::l7cx};
        r.at[i].verdict = (l7 && l7_retained) ? enm::outcome::retained
                                              : enm::outcome::enumerated;
        r.at[i].address = s.at[i].target_addr;
        r.at[i].seen = l7 ? kL7Id : kL4Id;
    }
    r.source_allowed[0] = true;
    r.source_allowed[1] = true;
    return r;
}

pf::isolation_observation clean_isolation(const enm::chain_spec &s)
{
    pf::isolation_observation o{};
    o.attempted = true;
    o.tail_probe = enm::probe_state::ack;
    o.answering_addr = s.at[s.positions - 1].target_addr;
    o.id_read_ok = true;
    o.seen = kL4Id;
    o.prev_probe = enm::probe_state::nack;
    o.prev_addr = s.at[s.positions - 2].target_addr;
    return o;
}

struct transaction {
    enm::chain_spec spec{product_spec()};
    enm::chain_result walk1{clean_walk(spec, false)};
    enm::chain_result walk2{clean_walk(spec, true)};
    pf::isolation_observation isolation{clean_isolation(spec)};

    pf::evidence evidence() const
    {
        pf::evidence ev{};
        ev.spec = &spec;
        ev.walk1 = &walk1;
        ev.walk2 = &walk2;
        ev.isolation = isolation;
        return ev;
    }
};

void fresh_authority()
{
    begin_epoch_rc = 0;
    begin_epoch_calls = 0;
    runtime_spec_storage = product_spec();

    au::config cfg{};
    cfg.runtime_spec = &runtime_spec_storage;
    cfg.begin_epoch = fake_begin_epoch;
    zassert_equal(au::init(cfg), 0);
}

/* The whole happy path, since almost every test needs it up to some point. */
au::commit_refusal prove(const transaction &t, uint8_t epoch)
{
    const pf::challenge c{au::begin_proof()};
    pf::verdict v{au::evaluate(t.evidence(), c)};
    zassert_true(v.granted(), "evidence refused: %d", static_cast<int>(v.reason));
    return au::commit_proof(static_cast<pf::proof_token &&>(v.token), epoch);
}

} // namespace

ZTEST_SUITE(tof_mapping_authority, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_mapping_authority, test_it_starts_unknown_with_no_epoch)
{
    fresh_authority();
    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::not_ready);
    zassert_equal(s.epoch, 0);
    zassert_equal(s.failing_position, 0xFF);
    zassert_equal(au::installed_mapping().positions, 0u);
}

ZTEST(tof_mapping_authority, test_init_refuses_an_incomplete_configuration)
{
    au::config no_spec{};
    no_spec.begin_epoch = fake_begin_epoch;
    zassert_equal(au::init(no_spec), -EINVAL);

    au::config no_hook{};
    no_hook.runtime_spec = &runtime_spec_storage;
    zassert_equal(au::init(no_hook), -EINVAL);
}

ZTEST(tof_mapping_authority, test_a_committed_proof_publishes_proven_with_its_epoch)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 7), au::commit_refusal::none);

    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::proven);
    zassert_equal(s.epoch, 7);
    zassert_equal(au::installed_mapping().positions, 6u);
    zassert_equal(au::installed_mapping().at[5].role, enm::l4_role::front_right);
    /* The cycle reset happened, exactly once, as part of the commit. */
    zassert_equal(begin_epoch_calls, 1);
}

ZTEST(tof_mapping_authority, test_the_enumeration_masks_stay_zero_until_the_role_table_is_frozen)
{
    /* Not an oversight, and pinned so it cannot become one silently: the contract keys these
     * by source_id and this firmware has no honest source_id for a cliff position yet. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 3), au::commit_refusal::none);

    const au::snapshot s{au::current()};
    zassert_equal(s.enumerated_mask, 0);
    zassert_equal(s.model_verified_mask, 0);
}

ZTEST(tof_mapping_authority, test_beginning_a_proof_revokes_a_proven_mapping_as_lost)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 4), au::commit_refusal::none);

    (void)au::begin_proof();
    const au::snapshot s{au::current()};
    /* LOST, not UNKNOWN. Something the consumer was trusting has gone away, and its recovery
     * path differs from a mapping that was never proven. */
    zassert_equal(s.state, acq::mapping_state::lost);
    /* The epoch survives: the measurements already accepted were correlated under it. */
    zassert_equal(s.epoch, 4);
    zassert_equal(au::installed_mapping().positions, 0u);
}

ZTEST(tof_mapping_authority, test_beginning_a_proof_from_unknown_stays_unknown)
{
    /* The asymmetry matters: reporting LOST from UNKNOWN would tell a consumer that something
     * it had been trusting disappeared, when nothing ever had been. */
    fresh_authority();
    (void)au::begin_proof();
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
}

ZTEST(tof_mapping_authority, test_a_token_from_a_superseded_attempt_cannot_be_committed)
{
    /* The replay this ordering exists to stop. The first attempt's token describes a chain
     * that was correct when it was proven; the second begin_proof() is about to move enable
     * lines, so that description stops being about the live machine. */
    fresh_authority();
    const transaction t;

    const pf::challenge first{au::begin_proof()};
    pf::verdict v{au::evaluate(t.evidence(), first)};
    zassert_true(v.granted());

    (void)au::begin_proof(); // a new attempt supersedes it

    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 9),
                  au::commit_refusal::wrong_attempt);
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
}

ZTEST(tof_mapping_authority, test_a_fabricated_token_authorises_nothing)
{
    fresh_authority();
    (void)au::begin_proof();
    pf::proof_token forged{}; // the only token a caller can build
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(forged), 5),
                  au::commit_refusal::invalid_token);
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
}

ZTEST(tof_mapping_authority, test_commit_without_an_attempt_is_refused)
{
    fresh_authority();
    pf::proof_token forged{};
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(forged), 5),
                  au::commit_refusal::no_attempt);
}

ZTEST(tof_mapping_authority, test_a_committed_attempt_cannot_be_committed_again)
{
    fresh_authority();
    const transaction t;

    const pf::challenge c{au::begin_proof()};
    pf::verdict v{au::evaluate(t.evidence(), c)};
    zassert_true(v.granted());
    pf::proof_token first{static_cast<pf::proof_token &&>(v.token)};
    /* A copy of the same authority is impossible -- the token is move-only -- so the second
     * attempt has to reuse the moved-from object, which is empty by construction. */
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(first), 11),
                  au::commit_refusal::none);
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(first), 12),
                  au::commit_refusal::no_attempt);
    zassert_equal(au::current().epoch, 11);
}

ZTEST(tof_mapping_authority, test_a_proof_of_a_different_chain_cannot_install_itself)
{
    /* Sound proof, wrong machine. The evaluator cannot catch this -- it never sees the
     * runtime configuration -- and it is the reason the authority re-checks rather than
     * trusting the token. */
    fresh_authority();
    transaction t;
    t.spec.at[4].target_addr = 0x3E; // proven at an address acquisition will not read
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);

    const pf::challenge c{au::begin_proof()};
    pf::verdict v{au::evaluate(t.evidence(), c)};
    zassert_true(v.granted(), "the proof itself is sound: %d", static_cast<int>(v.reason));
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 6),
                  au::commit_refusal::runtime_mapping_mismatch);
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
    zassert_equal(begin_epoch_calls, 0, "nothing may be installed before every check passes");
}

ZTEST(tof_mapping_authority, test_a_role_table_that_differs_from_the_runtime_one_is_refused)
{
    /* The same check on the field that actually decides which physical corner a range
     * describes. Two roles swapped is a proof of the same wiring and a different robot. */
    fresh_authority();
    transaction t;
    t.spec.at[2].role = enm::l4_role::rear_right;
    t.spec.at[4].role = enm::l4_role::front_left;
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);

    const pf::challenge c{au::begin_proof()};
    pf::verdict v{au::evaluate(t.evidence(), c)};
    zassert_true(v.granted());
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 6),
                  au::commit_refusal::runtime_mapping_mismatch);
}

ZTEST(tof_mapping_authority, test_epoch_zero_is_refused)
{
    /* 0 is what the health frame carries while no epoch has been issued. If it were also a
     * real epoch, "proven under 0" and "never proven" would read the same in one frame. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 0), au::commit_refusal::epoch_zero);
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
}

ZTEST(tof_mapping_authority, test_an_epoch_used_this_power_cycle_is_refused)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 5), au::commit_refusal::none);
    zassert_equal(prove(t, 5), au::commit_refusal::epoch_reused);
    /* And the refusal leaves the mapping revoked rather than still proven: begin_proof()
     * already published LOST. */
    zassert_equal(au::current().state, acq::mapping_state::lost);
}

ZTEST(tof_mapping_authority, test_reuse_is_checked_against_every_epoch_not_just_the_last_one)
{
    /* The bitmap earns its 32 bytes here. Comparing against only the previous value would let
     * 1, 2, 1 through, and then two different mappings share an epoch -- a consumer
     * correlating by (source_id, epoch, cycle_seq) cannot tell their frames apart. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 1), au::commit_refusal::none);
    zassert_equal(prove(t, 2), au::commit_refusal::none);
    zassert_equal(prove(t, 1), au::commit_refusal::epoch_reused);
}

ZTEST(tof_mapping_authority, test_the_epoch_space_runs_out_rather_than_wrapping)
{
    /* 255 real epochs, then a refusal. A machine re-proven 255 times in one power cycle has a
     * problem that silently reusing an epoch would hide. */
    fresh_authority();
    const transaction t;
    for (int e{1}; e <= 255; ++e)
        zassert_equal(prove(t, static_cast<uint8_t>(e)), au::commit_refusal::none, "epoch %d", e);

    zassert_equal(au::epochs_used(), 256u, "255 issued plus the reserved zero");
    /* Every value is now used, so whatever the host offers is refused -- and the refusal is
     * the exhaustion, not a reuse, because there is no unused value left to ask for. */
    zassert_equal(prove(t, 200), au::commit_refusal::epoch_space_exhausted);
    zassert_not_equal(au::current().state, acq::mapping_state::proven);
}

ZTEST(tof_mapping_authority, test_a_failed_cycle_reset_leaves_the_state_non_proven)
{
    /* The transaction property. If the cycle counter could not be reset, a PROVEN with the new
     * epoch would invite the consumer to accept cycle numbers left over from the previous
     * epoch. */
    fresh_authority();
    const transaction t;
    begin_epoch_rc = -EBUSY;
    zassert_equal(prove(t, 8), au::commit_refusal::acquisition_busy);

    const au::snapshot s{au::current()};
    zassert_not_equal(s.state, acq::mapping_state::proven);
    zassert_equal(s.epoch, 0, "the epoch must not be installed either");
    zassert_equal(au::installed_mapping().positions, 0u, "nothing half-installed");
}

ZTEST(tof_mapping_authority, test_any_other_cycle_reset_failure_also_leaves_it_non_proven)
{
    /* -EBUSY says "try again when acquisition stops"; anything else says the acquisition layer
     * is not in a state to be told about epochs at all. Different diagnosis, same refusal to
     * publish PROVEN, and both counted separately so an operator is not sent to wait for an
     * idle that will never come. */
    fresh_authority();
    const transaction t;
    begin_epoch_rc = -EINVAL;
    zassert_equal(prove(t, 9), au::commit_refusal::epoch_install_failed);
    zassert_not_equal(au::current().state, acq::mapping_state::proven);
    zassert_equal(au::current().epoch, 0);
}

ZTEST(tof_mapping_authority, test_an_epoch_refused_by_a_failed_reset_can_be_used_again)
{
    /* It was never issued, so burning it would waste the space for no safety gain -- and
     * would eventually exhaust it on a machine whose acquisition kept being busy. */
    fresh_authority();
    const transaction t;
    begin_epoch_rc = -EBUSY;
    zassert_equal(prove(t, 8), au::commit_refusal::acquisition_busy);
    begin_epoch_rc = 0;
    zassert_equal(prove(t, 8), au::commit_refusal::none);
    zassert_equal(au::current().epoch, 8);
}

ZTEST(tof_mapping_authority, test_a_runtime_loss_reports_lost_and_keeps_the_epoch)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 12), au::commit_refusal::none);

    au::note_mapping_lost();
    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::lost);
    zassert_equal(s.epoch, 12);
    zassert_equal(au::installed_mapping().positions, 0u);
}

ZTEST(tof_mapping_authority, test_a_loss_reported_while_unknown_changes_nothing)
{
    fresh_authority();
    au::note_mapping_lost();
    zassert_equal(au::current().state, acq::mapping_state::not_ready);
}

ZTEST(tof_mapping_authority, test_a_chain_fault_publishes_fault_with_its_position)
{
    fresh_authority();
    au::note_chain_fault(0x2, 5);
    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::fault);
    zassert_equal(s.chain_flags, 0x2);
    zassert_equal(s.failing_position, 5);
}

ZTEST(tof_mapping_authority, test_the_snapshot_survives_a_pack_and_unpack_round_trip)
{
    /* The snapshot is one atomic word so the heartbeat never blocks behind a proof. That only
     * holds if every field really fits, and a field silently truncated by the packing would
     * mis-report the one thing the consumer gates on. */
    fresh_authority();
    au::note_chain_fault(0x7, 0xFF);
    const au::snapshot faulted{au::current()};
    zassert_equal(faulted.chain_flags, 0x7);
    zassert_equal(faulted.failing_position, 0xFF);

    const transaction t;
    zassert_equal(prove(t, 255), au::commit_refusal::none);
    const au::snapshot proven{au::current()};
    zassert_equal(proven.epoch, 255, "the top epoch value must survive the packing");
    zassert_equal(proven.state, acq::mapping_state::proven);
}

ZTEST(tof_mapping_authority, test_the_state_provider_reports_what_the_authority_believes)
{
    /* It reports PROVEN. The acquisition layer's clamp is what decides whether anything acts
     * on it, and that clamp is not this module's business -- keeping the two separate is why
     * lifting the clamp can be a small reviewable commit. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 2), au::commit_refusal::none);
    zassert_equal(au::state_provider(), acq::mapping_state::proven);
}

ZTEST(tof_mapping_authority, test_a_bench_chain_can_be_diagnosed_without_touching_the_attempt)
{
    fresh_authority();
    transaction bench;
    bench.spec.positions = 3;
    bench.spec.at[0] = {enm::model::l7cx, 0x2A, 0, enm::l4_role::unknown};
    bench.spec.at[1] = {enm::model::l7cx, 0x2B, 1, enm::l4_role::unknown};
    bench.spec.at[2] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    bench.walk1 = clean_walk(bench.spec, false);
    bench.walk2 = clean_walk(bench.spec, true);
    bench.isolation = clean_isolation(bench.spec);

    const pf::challenge c{au::begin_proof()};
    zassert_true(au::evaluate_bench(bench.evidence()).clean());
    /* The attempt is untouched, so a diagnostic run cannot burn the operator's attempt. */
    const transaction product;
    zassert_true(au::evaluate(product.evidence(), c).granted());
}

ZTEST(tof_mapping_authority, test_nothing_works_before_init)
{
    /* Not a courtesy check. A commit accepted before the runtime spec exists would compare the
     * proven chain against nothing at all. */
    au::config bad{};
    (void)au::init(bad); // leaves the authority uninitialised

    const transaction t;
    zassert_false(au::begin_proof().valid());
    zassert_false(au::evaluate(t.evidence(), pf::challenge{}).granted());
    pf::proof_token forged{};
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(forged), 3),
                  au::commit_refusal::not_initialised);
}
