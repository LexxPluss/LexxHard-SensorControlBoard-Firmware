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

#include "tof_cliff_packer.hpp"

namespace ctr = tof_cliff_contract;
namespace pk = tof_cliff_packer;

namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;
namespace au = lexxhard::tof_authority;
namespace acq = lexxhard::tof_acq;

namespace {

constexpr enm::id_bytes kL7Id{0xf0, 0x02};
constexpr enm::id_bytes kL4Id{0xeb, 0xaa};

int begin_epoch_rc{0};
int begin_epoch_calls{0};
bool acquisition_is_idle{true};

int fake_begin_epoch()
{
    ++begin_epoch_calls;
    return begin_epoch_rc;
}

bool fake_is_idle()
{
    return acquisition_is_idle;
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

/* The descriptor keying, as the authority sees it: a hook that can refuse. What production does
 * inside it is tof_cliff_runtime's business and is tested there; what matters here is that the
 * transaction treats a refusal as a failed commit rather than as a detail after the fact. */
int install_rc{0};
int install_calls{0};
uint8_t install_epoch_seen{0};
bool proven_when_installed{false};

int fake_install(const pf::fingerprint &fp, uint8_t epoch)
{
    ++install_calls;
    install_epoch_seen = epoch;
    /* Recorded, because the ORDER is the property: the keying has to happen while the authority is
     * still not PROVEN. A callback that observed PROVEN would mean publication had already happened,
     * which is the arrangement this transaction replaced. */
    proven_when_installed = au::current().state == acq::mapping_state::proven;
    zassert_true(fp.positions > 0, "the callback was handed an empty mapping");
    return install_rc;
}

void fresh_authority()
{
    begin_epoch_rc = 0;
    begin_epoch_calls = 0;
    acquisition_is_idle = true;
    install_rc = 0;
    install_calls = 0;
    install_epoch_seen = 0;
    proven_when_installed = false;
    runtime_spec_storage = product_spec();

    au::config cfg{};
    cfg.runtime_spec = &runtime_spec_storage;
    cfg.begin_epoch = fake_begin_epoch;
    cfg.acquisition_idle = fake_is_idle;
    cfg.install_mapping = fake_install;
    zassert_equal(au::init(cfg), 0);
    /* Explicitly, through the test-only door. init() deliberately does NOT clear the used-epoch
     * bitmap -- the guarantee is per power cycle, not per configuration -- so a suite that
     * relied on init() to reset it would be testing a product behaviour that must not exist. */
    au::reset_epoch_history_for_test();
}

/* The whole happy path, since almost every test needs it up to some point. */
au::commit_refusal prove(const transaction &t, uint8_t epoch)
{
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened(), "attempt refused: %d", static_cast<int>(a.reason));
    zassert_equal(a.reason, au::begin_refusal::none);
    pf::verdict v{au::evaluate(t.evidence(), a.challenge)};
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
    no_spec.acquisition_idle = fake_is_idle;
    no_spec.install_mapping = fake_install;
    zassert_equal(au::init(no_spec), -EINVAL);

    au::config no_epoch_hook{};
    no_epoch_hook.runtime_spec = &runtime_spec_storage;
    no_epoch_hook.acquisition_idle = fake_is_idle;
    no_epoch_hook.install_mapping = fake_install;
    zassert_equal(au::init(no_epoch_hook), -EINVAL);

    /* The idle hook is not optional either: without it the authority cannot tell whether a
     * proof is safe to start, and defaulting to "assume idle" is the unsafe direction. */
    au::config no_idle_hook{};
    no_idle_hook.runtime_spec = &runtime_spec_storage;
    no_idle_hook.begin_epoch = fake_begin_epoch;
    no_idle_hook.install_mapping = fake_install;
    zassert_equal(au::init(no_idle_hook), -EINVAL);

    /* Nor is the install hook. Publishing PROVEN without keying the descriptors that give its
     * source_ids meaning would be publishing a mapping nothing acts on -- and an optional hook
     * would make that the default. */
    au::config no_install_hook{};
    no_install_hook.runtime_spec = &runtime_spec_storage;
    no_install_hook.begin_epoch = fake_begin_epoch;
    no_install_hook.acquisition_idle = fake_is_idle;
    zassert_equal(au::init(no_install_hook), -EINVAL);
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

ZTEST(tof_mapping_authority, test_a_committed_proof_fills_both_enumeration_masks)
{
    /* This test used to assert the masks stayed 0 after a commit, which was wrong: I had
     * stretched "the role table is not frozen" -- a reason PROVEN is unreachable at all -- into
     * "even a proof cannot fill the masks". A granted proof has four known, distinct roles by
     * rule, so the four source bits are exactly what it proved. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 3), au::commit_refusal::none);

    const au::snapshot s{au::current()};
    zassert_equal(s.enumerated_mask, 0xF);
    zassert_equal(s.model_verified_mask, 0xF);
}

ZTEST(tof_mapping_authority, test_the_masks_are_keyed_by_the_contracts_role_table)
{
    /* Not the position index and not an arithmetic cast of the enum. The bit for a role is the
     * source_id the wire contract assigns it, so a chain whose roles are mounted in a different
     * order still produces the same four bits -- and if the mapping were derived from the
     * position instead, this test would pass while the frames named the wrong corners. */
    fresh_authority();
    transaction t;
    t.spec.at[2].role = enm::l4_role::front_right; // source 3
    t.spec.at[3].role = enm::l4_role::rear_right;  // source 2
    t.spec.at[4].role = enm::l4_role::rear_left;   // source 1
    t.spec.at[5].role = enm::l4_role::front_left;  // source 0
    runtime_spec_storage = t.spec;
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);

    zassert_equal(prove(t, 4), au::commit_refusal::none);
    zassert_equal(au::current().enumerated_mask, 0xF);
    zassert_equal(au::installed_mapping().at[2].role, enm::l4_role::front_right);
}

ZTEST(tof_mapping_authority, test_a_runtime_loss_keeps_the_enumeration_masks)
{
    /* The contract defines these as the last enumeration ATTEMPT, not as current trust: state
     * carries the trust. A sensor vanishing at runtime does not change the fact that the last
     * enumeration enumerated all four, and zeroing them would misreport the field's own
     * meaning -- and throw away the diagnostic distinction between "lost after a clean
     * enumeration" and "never enumerated cleanly at all". */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 5), au::commit_refusal::none);
    zassert_equal(au::current().enumerated_mask, 0xF);

    au::note_mapping_lost();
    const au::snapshot lost{au::current()};
    zassert_equal(lost.state, acq::mapping_state::lost);
    zassert_equal(lost.enumerated_mask, 0xF);
    zassert_equal(lost.model_verified_mask, 0xF);
}

ZTEST(tof_mapping_authority, test_starting_a_new_proof_clears_the_enumeration_masks)
{
    /* The other half of the same rule, and the only place they are cleared. From here the enable
     * lines are about to move, so no completed enumeration describes the chain that is about to
     * exist -- reporting the old one would describe a machine that is being taken apart. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 6), au::commit_refusal::none);
    zassert_equal(au::current().enumerated_mask, 0xF);

    (void)au::begin_proof();
    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::lost);
    zassert_equal(s.enumerated_mask, 0);
    zassert_equal(s.model_verified_mask, 0);
    zassert_equal(s.epoch, 6, "the epoch still survives");
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

    const pf::challenge first{au::begin_proof().challenge};
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

    const pf::challenge c{au::begin_proof().challenge};
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

    const pf::challenge c{au::begin_proof().challenge};
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

    const pf::challenge c{au::begin_proof().challenge};
    pf::verdict v{au::evaluate(t.evidence(), c)};
    zassert_true(v.granted());
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 6),
                  au::commit_refusal::runtime_mapping_mismatch);
}

ZTEST(tof_mapping_authority, test_epoch_zero_is_a_real_epoch)
{
    /* It was refused for a while, on the theory that "PROVEN under epoch 0" and "never proven"
     * would be confusable. They are not: mapping_state is in the same frame as the epoch, and
     * cycle_valid separates "no cycle" from "cycle 0". Meanwhile the contract has the host
     * increment modulo 256, so 0 comes round on every 256th proof and a firmware that refused
     * it would stall commissioning on a machine that had done nothing wrong. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 0), au::commit_refusal::none);
    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::proven);
    zassert_equal(s.epoch, 0);
    /* And it is spent like any other value. */
    zassert_equal(prove(t, 0), au::commit_refusal::epoch_reused);
}

ZTEST(tof_mapping_authority, test_the_host_may_wrap_255_to_zero)
{
    /* The host's documented behaviour, so it has to work rather than merely not crash. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 255), au::commit_refusal::none);
    zassert_equal(prove(t, 0), au::commit_refusal::none);
    zassert_equal(au::current().epoch, 0);
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
    /* All 256 values, then a refusal. A machine re-proven 256 times in one power cycle has a
     * problem that silently reusing an epoch would hide. */
    fresh_authority();
    const transaction t;
    for (int e{0}; e <= 255; ++e)
        zassert_equal(prove(t, static_cast<uint8_t>(e)), au::commit_refusal::none, "epoch %d", e);

    zassert_equal(au::epochs_used(), 256u, "all 256 values are real epochs");
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

/* Builds the health frame the PUBLISHER would build from a snapshot, and asks the REAL encoder
 * whether it is acceptable. The field copies mirror production: tof_cliff_can.cpp's
 * production_authorisation() takes epoch, both masks, chain_flags and failing_position straight
 * from the snapshot and clamps only the state, and the heartbeat in tof_cliff_publisher.cpp zeroes
 * the per-cycle fields because it describes no cycle.
 *
 * What is deliberately NOT restated here is any of encode_health()'s refusal conditions. Copying
 * those would pin what this test believes the contract says, and would go on passing on the day
 * the contract changed -- which is the whole failure mode being guarded against. */
bool the_publisher_could_send(const au::snapshot &s)
{
    pk::health_fields h{};
    h.mapping_epoch = s.epoch;
    h.health_seq = 1;
    /* FAULT on the wire is 3, and the two enumerations agree on nothing but zero -- a cast would
     * put PROVEN on the wire. clamp_mapping_state() rewrites only `proven`, so a faulted snapshot
     * reaches the encoder unchanged. */
    h.mapping_state = 0x3;
    h.flags = static_cast<uint8_t>(s.chain_flags & 0x7);
    h.enumerated_mask = s.enumerated_mask;
    h.model_verified_mask = s.model_verified_mask;
    h.failing_chain_position = s.failing_position;
    h.sample_produced_mask = 0;
    h.sensor_fault_mask = 0;
    h.cycle_seq = 0;

    uint8_t frame[8]{};
    return pk::encode_health(h, frame);
}

ZTEST(tof_mapping_authority, test_the_no_position_constant_mirrors_the_wire_contract)
{
    /* The authority does not include the contract header on purpose -- it is wire-agnostic, and
     * the publisher is where the two meet. This is the assertion that keeps the mirror honest. */
    zassert_equal(au::kNoFailingPosition, ctr::kChainPositionNone);
}

/* THE INVARIANT, and the reason this suite links the packer.
 *
 * A snapshot the encoder refuses does not surface as an error. Both health producers answer
 * encode_health() == false by incrementing a counter and returning, so it surfaces as SILENCE --
 * the heartbeat stops at the exact moment a chain fault is meant to be reported. note_chain_fault()
 * therefore has to guarantee an encodable result for EVERY argument, not merely for the arguments
 * its callers are expected to pass. Swept exhaustively because "expected to pass" is precisely the
 * assumption that failed. */
ZTEST(tof_mapping_authority, test_every_possible_argument_still_leaves_an_encodable_snapshot)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 7), au::commit_refusal::none);
    zassert_equal(au::current().enumerated_mask, 0x0F, "a proof must fill the masks, or the sweep "
                                                       "below would never reach the refusing case");

    for (unsigned flags = 0; flags <= 0xFF; ++flags) {
        for (unsigned pos = 0; pos <= 0xFF; ++pos) {
            const bool taken = au::note_chain_fault(static_cast<uint8_t>(flags), static_cast<uint8_t>(pos));
            const au::snapshot s{au::current()};
            zassert_equal(s.state, acq::mapping_state::fault, "flags=%u pos=%u left the authority "
                                                              "somewhere other than FAULT", flags, pos);
            zassert_true(the_publisher_could_send(s),
                         "flags=%u pos=%u published a frame the encoder refuses, which silences "
                         "the heartbeat", flags, pos);
            if (!taken) {
                zassert_equal(s.chain_flags, 0, "a rejected reason must not be half-published");
                zassert_equal(s.failing_position, au::kNoFailingPosition);
            }
        }
    }
}

/* Koko's case (#103 review): from PROVEN, a position named with no chain-fault bit. The masks are
 * 0xF by then, which is what makes the encoder refuse -- and is why the pre-existing chain-fault
 * test could not see this: it runs from a fresh authority, where the masks are still zero. */
ZTEST(tof_mapping_authority, test_naming_a_position_with_no_reason_degrades_to_a_generic_fault)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 3), au::commit_refusal::none);

    zassert_false(au::note_chain_fault(0x0, 3), "the arguments are contradictory and must be refused");
    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::fault);
    zassert_equal(s.chain_flags, 0);
    zassert_equal(s.failing_position, au::kNoFailingPosition);
    zassert_equal(s.enumerated_mask, 0x0F, "the masks are the last enumeration result and are not "
                                           "cleared to satisfy the encoder");
    zassert_equal(s.model_verified_mask, 0x0F);
    zassert_equal(s.epoch, 3, "the epoch is kept so the consumer can still correlate");
    zassert_true(the_publisher_could_send(s));
}

/* The other half: a well-formed fault after PROVEN must pass through unchanged, masks and all. If
 * the fix had simply cleared the masks to make everything encodable, this is the test that fails. */
ZTEST(tof_mapping_authority, test_a_well_formed_fault_after_proven_keeps_its_reason_and_its_masks)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 4), au::commit_refusal::none);

    zassert_true(au::note_chain_fault(0x2, 5));
    const au::snapshot s{au::current()};
    zassert_equal(s.chain_flags, 0x2);
    zassert_equal(s.failing_position, 5);
    zassert_equal(s.enumerated_mask, 0x0F);
    zassert_equal(s.model_verified_mask, 0x0F);
    zassert_true(the_publisher_could_send(s));
}

ZTEST(tof_mapping_authority, test_an_out_of_range_position_degrades_to_a_generic_fault)
{
    static constexpr uint8_t kBadPositions[]{0, 7, 0xFE};
    for (const uint8_t pos : kBadPositions) {
        fresh_authority();
        const transaction t;
        zassert_equal(prove(t, 2), au::commit_refusal::none);
        zassert_false(au::note_chain_fault(0x1, pos), "position %u is outside 1-6 and not NONE", pos);
        const au::snapshot s{au::current()};
        zassert_equal(s.failing_position, au::kNoFailingPosition);
        zassert_equal(s.chain_flags, 0);
        zassert_true(the_publisher_could_send(s));
    }
}

ZTEST(tof_mapping_authority, test_flags_outside_the_contract_bits_degrade_to_a_generic_fault)
{
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 2), au::commit_refusal::none);

    /* Previously masked with & 0x7, which silently turned a caller's bug into a different, legal
     * fault reason. A reason nobody asked for is worse than no reason. */
    zassert_false(au::note_chain_fault(0x18, 5));
    const au::snapshot s{au::current()};
    zassert_equal(s.chain_flags, 0);
    zassert_equal(s.failing_position, au::kNoFailingPosition);
    zassert_true(the_publisher_could_send(s));
}

ZTEST(tof_mapping_authority, test_a_chain_fault_clears_the_installed_mapping_even_when_refused)
{
    /* The arguments can be wrong; the fault is still real. Leaving a mapping installed because the
     * caller mislabelled its reason would be the worst outcome available. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 6), au::commit_refusal::none);
    zassert_not_equal(au::installed_mapping().positions, 0, "the proof must have installed one");

    zassert_false(au::note_chain_fault(0x0, 3));
    zassert_equal(au::installed_mapping().positions, 0,
                  "a refused reason must not leave the mapping installed");
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

    /* With no attempt open, a bench evaluation runs and gives a real answer. */
    const au::bench_result idle_run{au::evaluate_bench(bench.evidence())};
    zassert_true(idle_run.ran);
    zassert_true(idle_run.report.clean(), "reason %d", static_cast<int>(idle_run.report.reason));

    /* With one open, it refuses. Bench evidence can only have come from walking a chain, and
     * walking it now would re-address the very chain the attempt is about -- so the authority
     * refuses rather than trusting the commissioning command to remember. */
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened());
    const au::bench_result during{au::evaluate_bench(bench.evidence())};
    zassert_false(during.ran, "a bench run was allowed during a product attempt");

    /* And refusing left the attempt intact. */
    const transaction product;
    zassert_true(au::evaluate(product.evidence(), a.challenge).granted());
}

ZTEST(tof_mapping_authority, test_nothing_works_before_init)
{
    /* Not a courtesy check. A commit accepted before the runtime spec exists would compare the
     * proven chain against nothing at all. */
    au::config bad{};
    (void)au::init(bad); // leaves the authority uninitialised

    const transaction t;
    const au::attempt a{au::begin_proof()};
    zassert_false(a.opened());
    zassert_equal(a.reason, au::begin_refusal::not_initialised);
    zassert_false(au::evaluate(t.evidence(), pf::challenge{}).granted());
    pf::proof_token forged{};
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(forged), 3),
                  au::commit_refusal::not_initialised);
}

ZTEST(tof_mapping_authority, test_no_attempt_is_opened_while_acquisition_is_running)
{
    /* The check that has to happen at the START. A proof is two full enumerations, which drop
     * enable lines and re-address parts; finding out at commit time that acquisition was
     * running means that already happened underneath a live reader, and no refusal can undo
     * it. */
    fresh_authority();
    acquisition_is_idle = false;

    const au::attempt a{au::begin_proof()};
    zassert_false(a.opened());
    zassert_equal(a.reason, au::begin_refusal::acquisition_not_idle);
    zassert_false(a.challenge.valid());
    zassert_equal(au::attempt_nonce(), 0u, "no attempt may be left half-open");
}

ZTEST(tof_mapping_authority, test_a_mistimed_request_does_not_revoke_a_working_mapping)
{
    /* Deliberate: the refusal above must not be a punishment. A running acquisition under a
     * proven mapping is the normal state, and revoking it because someone asked to re-prove at
     * the wrong moment would turn a mistimed request into an outage. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 20), au::commit_refusal::none);

    acquisition_is_idle = false;
    zassert_false(au::begin_proof().opened());

    const au::snapshot s{au::current()};
    zassert_equal(s.state, acq::mapping_state::proven, "a refused request revoked the mapping");
    zassert_equal(s.epoch, 20);
}

ZTEST(tof_mapping_authority, test_re_initialising_does_not_hand_the_epoch_space_back)
{
    /* The bug this test exists for: init() used to clear the used-epoch bitmap, so
     * init -> epoch 7 -> init -> epoch 7 passed while breaking the contract outright. Every
     * test re-initialising was what hid it -- the suite's own harness was the alibi. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 7), au::commit_refusal::none);

    au::config cfg{};
    cfg.runtime_spec = &runtime_spec_storage;
    cfg.begin_epoch = fake_begin_epoch;
    cfg.acquisition_idle = fake_is_idle;
    cfg.install_mapping = fake_install;
    zassert_equal(au::init(cfg), 0);
    /* Note: no reset_epoch_history_for_test() here. That is the point. */

    zassert_equal(prove(t, 7), au::commit_refusal::epoch_reused,
                  "a re-init handed epoch 7 back");
    zassert_equal(prove(t, 8), au::commit_refusal::none, "unused values are still available");
}

ZTEST(tof_mapping_authority, test_re_initialising_does_close_any_open_attempt)
{
    /* The other side of the same question. The bitmap survives a re-init because it is about
     * the power cycle; an open attempt does not, because it is about a configuration that has
     * just been replaced. */
    fresh_authority();
    const transaction t;
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened());
    pf::verdict v{au::evaluate(t.evidence(), a.challenge)};
    zassert_true(v.granted());

    au::config cfg{};
    cfg.runtime_spec = &runtime_spec_storage;
    cfg.begin_epoch = fake_begin_epoch;
    cfg.acquisition_idle = fake_is_idle;
    cfg.install_mapping = fake_install;
    zassert_equal(au::init(cfg), 0);

    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 30),
                  au::commit_refusal::no_attempt);
}

ZTEST(tof_mapping_authority, test_a_refused_commit_still_closes_the_attempt)
{
    /* One attempt buys one commit. Otherwise a caller could re-present the same evidence until some
     * later check happened to pass, and each retry would be judged against a chain that is one
     * attempt older than the evidence describing it. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 3), au::commit_refusal::none);

    /* A commit refused for a reason discovered AFTER the token matched: the epoch is reused. */
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened());
    pf::verdict v{au::evaluate(t.evidence(), a.challenge)};
    zassert_true(v.granted());
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 3),
                  au::commit_refusal::epoch_reused);
    zassert_equal(au::attempt_nonce(), 0u, "a refused commit left the attempt open");
}

ZTEST(tof_mapping_authority, test_a_forged_token_cannot_close_someone_elses_attempt)
{
    /* The other half. Checks that run BEFORE the nonce comparison must not spend an attempt they
     * have no claim to -- otherwise anyone could cancel a commissioning run in progress by
     * presenting an empty token. */
    fresh_authority();
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened());

    pf::proof_token forged{};
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(forged), 5),
                  au::commit_refusal::invalid_token);
    zassert_equal(au::attempt_nonce(), a.challenge.nonce(), "a forged token closed the attempt");

    /* And the real attempt still works. */
    const transaction t;
    pf::verdict v{au::evaluate(t.evidence(), a.challenge)};
    zassert_true(v.granted());
    zassert_equal(au::commit_proof(static_cast<pf::proof_token &&>(v.token), 5),
                  au::commit_refusal::none);
}

ZTEST(tof_mapping_authority, test_abort_closes_an_attempt_that_will_not_be_committed)
{
    /* An abandoned attempt is not harmless: evaluate_bench() refuses while one is open, so giving
     * up without saying so silently disables diagnostics until somebody starts another proof. */
    fresh_authority();
    const transaction t;
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened());
    zassert_false(au::evaluate_bench(t.evidence()).ran, "an open attempt blocks bench runs");

    zassert_true(au::abort_proof(a.challenge));
    zassert_equal(au::attempt_nonce(), 0u);
    zassert_true(au::evaluate_bench(t.evidence()).ran, "aborting must give diagnostics back");
}

ZTEST(tof_mapping_authority, test_abort_is_bound_to_the_attempt_it_names)
{
    /* One caller must not be able to cancel another's run. */
    fresh_authority();
    const au::attempt a{au::begin_proof()};
    zassert_true(a.opened());

    zassert_false(au::abort_proof(pf::challenge{}), "a fabricated challenge aborted an attempt");
    zassert_equal(au::attempt_nonce(), a.challenge.nonce());

    const au::attempt b{au::begin_proof()};   // supersedes a
    zassert_false(au::abort_proof(a.challenge), "a stale challenge aborted the current attempt");
    zassert_equal(au::attempt_nonce(), b.challenge.nonce());
    zassert_true(au::abort_proof(b.challenge));
}

ZTEST(tof_mapping_authority, test_aborting_restores_nothing)
{
    /* begin_proof() revoked the mapping and the chain has since been walked. An abort says "this
     * attempt is over", not "put things back" -- there is nothing to put back that would be true. */
    fresh_authority();
    const transaction t;
    zassert_equal(prove(t, 9), au::commit_refusal::none);

    const au::attempt a{au::begin_proof()};
    zassert_equal(au::current().state, acq::mapping_state::lost);
    zassert_true(au::abort_proof(a.challenge));
    zassert_equal(au::current().state, acq::mapping_state::lost, "an abort restored PROVEN");
}

/* ------------------------------------------- the install step of the transaction ---- */

ZTEST(tof_mapping_authority, test_the_descriptors_are_keyed_before_proven_is_published)
{
    fresh_authority();
    const transaction t;

    zassert_equal(prove(t, 9), au::commit_refusal::none);

    zassert_equal(install_calls, 1, "the install callback was not part of the commit");
    zassert_equal(install_epoch_seen, 9, "the callback was not told which epoch it is keying");
    zassert_false(proven_when_installed,
                  "PROVEN was already published when the descriptors were keyed");
    zassert_equal(au::current().state, acq::mapping_state::proven);
}

ZTEST(tof_mapping_authority, test_a_refused_install_never_reaches_proven)
{
    /* The P1 this step exists for. Keying used to happen after the commit, so a refusal left a
     * PROVEN authority whose descriptors described a different chain -- and only the clamp stood
     * between that and a measurement published under another corner's source_id. */
    fresh_authority();
    const transaction t;
    install_rc = -EINVAL;

    zassert_equal(prove(t, 11), au::commit_refusal::mapping_install_failed);

    const au::snapshot s{au::current()};
    zassert_not_equal(s.state, acq::mapping_state::proven, "a failed install still published PROVEN");
    zassert_equal(au::installed_mapping().positions, 0u,
                  "the installed mapping survived a failed install");
    zassert_equal(s.enumerated_mask, 0);
    zassert_equal(s.model_verified_mask, 0);
}

ZTEST(tof_mapping_authority, test_an_epoch_refused_by_the_install_is_not_burned)
{
    /* It was never issued, so it must still be available -- the same rule a failed cycle reset
     * follows. Burning it would cost one of 256 for a mapping that never took effect. */
    fresh_authority();
    install_rc = -EINVAL;
    zassert_equal(prove(transaction{}, 12), au::commit_refusal::mapping_install_failed);

    install_rc = 0;
    zassert_equal(prove(transaction{}, 12), au::commit_refusal::none,
                  "the epoch was burned by a commit that never published");
    zassert_equal(au::current().epoch, 12);
}

ZTEST(tof_mapping_authority, test_a_refused_install_still_spends_the_attempt)
{
    /* One attempt buys one commit, whatever the outcome. Otherwise the same evidence could be
     * re-presented until some later check happened to pass. */
    fresh_authority();
    install_rc = -EINVAL;
    zassert_equal(prove(transaction{}, 13), au::commit_refusal::mapping_install_failed);
    zassert_equal(au::attempt_nonce(), 0u, "a failed install left the attempt open");
}
