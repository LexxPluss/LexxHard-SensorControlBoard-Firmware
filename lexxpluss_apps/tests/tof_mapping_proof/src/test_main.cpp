/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side tests for the cliff mapping proof.
 *
 * The evidence here is built by hand, and that is the point rather than a shortcut. Every
 * arrangement worth testing is one a healthy machine cannot produce: a tail that answers
 * its neighbour's address, a walk that claims `complete` while a position never verified,
 * two walks that disagree. Driving a real enumeration would give exactly one of these
 * cases -- the good one.
 *
 * What these tests must NOT do is fabricate a proof. A `proof_token` with a non-zero nonce
 * can only come out of `gate::evaluate`; the tests can inject any evidence they like, and
 * they still cannot mint one, which is the property the type is shaped for.
 */

#include <zephyr/ztest.h>

#include "tof_chain_spec.hpp"
#include "tof_mapping_proof.hpp"

namespace enm = lexxhard::tof_enum;
namespace pf = lexxhard::tof_proof;

/* The token cannot be copied, only moved. Compile-time, because a runtime test cannot
 * observe the absence of a copy constructor -- the program that tries does not build.
 *
 * Hand-rolled rather than <type_traits>: this build has no libstdc++ headers, and adding a
 * C++ standard library to a test image to ask one question about one class would be a
 * larger change than the question deserves. */
namespace detect {

template <bool B> struct answer { static constexpr bool value = B; };

template <typename T, typename = void> struct copy_constructible : answer<false> {};
template <typename T>
struct copy_constructible<T, decltype(void(new T(*static_cast<const T *>(nullptr))))>
    : answer<true> {};

template <typename T, typename = void> struct move_constructible : answer<false> {};
template <typename T>
struct move_constructible<T, decltype(void(new T(static_cast<T &&>(*static_cast<T *>(nullptr)))))>
    : answer<true> {};

} // namespace detect

static_assert(!detect::copy_constructible<pf::proof_token>::value,
              "a copyable token is a token that can be spent twice");
static_assert(detect::move_constructible<pf::proof_token>::value,
              "the authority takes the token by value");

namespace {

constexpr enm::id_bytes kL7Id{0xf0, 0x02};
constexpr enm::id_bytes kL4Id{0xeb, 0xaa};

/* The production chain plus the one thing production does not have yet: the frozen
 * mounting roles. Everything provable in this file is provable only because these four
 * lines exist here and not in tof_chain_spec.hpp. */
enm::chain_spec provable_spec()
{
    enm::chain_spec s{lexxhard::tof_chain::dasher_spec()};
    s.at[2].role = enm::l4_role::front_left;
    s.at[3].role = enm::l4_role::rear_left;
    s.at[4].role = enm::l4_role::rear_right;
    s.at[5].role = enm::l4_role::front_right;
    return s;
}

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
        r.at[i].enable_commanded_high = true;
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

/* One transaction that should pass, so every negative test differs from it in exactly one
 * respect. Anything that fails for two reasons at once proves nothing about either. */
struct transaction {
    enm::chain_spec spec{provable_spec()};
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

pf::refusal refused(const transaction &t)
{
    pf::gate g;
    const pf::challenge c{g.issue()};
    return g.evaluate(t.evidence(), c).reason;
}

/* The diagnostic path. Chain shapes that are not the commissioning profile can only be
 * checked here, because evaluate() refuses them on the topology before it looks at
 * anything else -- which is the point of the topology check. */
pf::refusal bench_refused(const transaction &t)
{
    pf::gate g;
    return g.evaluate_bench(t.evidence()).reason;
}

} // namespace

ZTEST_SUITE(tof_mapping_proof, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_mapping_proof, test_a_clean_transaction_is_proven)
{
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};
    const pf::verdict v{g.evaluate(t.evidence(), c)};

    zassert_true(v.granted(), "reason %d", static_cast<int>(v.reason));
    zassert_equal(v.reason, pf::refusal::none);
    zassert_true(v.token.valid());
}

ZTEST(tof_mapping_proof, test_the_token_is_bound_to_the_challenge_that_authorised_it)
{
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};
    const pf::verdict v{g.evaluate(t.evidence(), c)};

    zassert_true(v.granted());
    /* The authority compares these two before committing. If the token did not carry the
     * nonce there would be nothing to compare, and a token from any earlier attempt would
     * be as good as this one. */
    zassert_equal(v.token.nonce(), c.nonce());
    zassert_not_equal(c.nonce(), 0u, "nonce zero is the fabricated value");
}

ZTEST(tof_mapping_proof, test_the_token_carries_the_chain_that_was_proven)
{
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};
    const pf::verdict v{g.evaluate(t.evidence(), c)};

    zassert_true(v.granted());
    const pf::fingerprint &fp{v.token.proven()};
    zassert_equal(fp.positions, 6u);
    for (size_t i{0}; i < fp.positions; ++i) {
        zassert_equal(fp.at[i].position, static_cast<uint8_t>(i + 1));
        zassert_equal(fp.at[i].address, t.spec.at[i].target_addr, "position %u", (unsigned)i);
        zassert_true(fp.at[i].verified);
    }
    /* The roles travel with the proof: the authority fills masks keyed by source_id, and
     * re-deriving the role from the position would be the arithmetic guess the whole
     * mapping exists to avoid. */
    zassert_equal(fp.at[2].role, enm::l4_role::front_left);
    zassert_equal(fp.at[5].role, enm::l4_role::front_right);
}

ZTEST(tof_mapping_proof, test_an_l7_reported_retained_in_the_second_walk_still_proves)
{
    /* The normalisation that matters most, because getting it wrong produces a check that
     * fails on every healthy chain: the L7 keeps its address across an enable-low while
     * powered, so the second walk finds it already at its target. */
    transaction t;
    t.walk1 = clean_walk(t.spec, false); // enumerated
    t.walk2 = clean_walk(t.spec, true);  // retained
    zassert_equal(t.walk1.at[0].verdict, enm::outcome::enumerated);
    zassert_equal(t.walk2.at[0].verdict, enm::outcome::retained);
    zassert_equal(refused(t), pf::refusal::none);
}

ZTEST(tof_mapping_proof, test_an_l4_reported_retained_is_a_fault_not_a_normalisation)
{
    /* Same verdict, different model, opposite meaning. The L4 enable is reset-class, so a
     * retained L4 kept an address it should have lost. */
    transaction t;
    t.walk2.at[3].verdict = enm::outcome::retained;
    zassert_equal(refused(t), pf::refusal::l4_retained);
}

ZTEST(tof_mapping_proof, test_a_fabricated_challenge_authorises_nothing)
{
    const transaction t;
    pf::gate g;
    (void)g.issue();
    const pf::challenge forged{}; // the only challenge a caller can build
    const pf::verdict v{g.evaluate(t.evidence(), forged)};

    zassert_false(v.granted());
    zassert_equal(v.reason, pf::refusal::challenge_invalid);
    zassert_false(v.token.valid());
}

ZTEST(tof_mapping_proof, test_a_superseded_challenge_is_refused)
{
    const transaction t;
    pf::gate g;
    const pf::challenge first{g.issue()};
    const pf::challenge second{g.issue()};

    zassert_not_equal(first.nonce(), second.nonce());
    /* This is the replay defence: a second attempt kills the first attempt's challenge, so
     * evidence gathered before the chain was disturbed again cannot be presented after. */
    zassert_equal(g.evaluate(t.evidence(), first).reason, pf::refusal::challenge_stale);
    zassert_true(g.evaluate(t.evidence(), second).granted());
}

ZTEST(tof_mapping_proof, test_a_granted_challenge_cannot_be_used_twice)
{
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};

    zassert_true(g.evaluate(t.evidence(), c).granted());
    const pf::verdict again{g.evaluate(t.evidence(), c)};
    zassert_false(again.granted());
    zassert_equal(again.reason, pf::refusal::challenge_consumed);
    zassert_false(again.token.valid());
}

ZTEST(tof_mapping_proof, test_a_refused_attempt_also_consumes_the_challenge)
{
    /* One evaluation per challenge, pass or fail, and the failing case is the one that
     * matters. A refusal means the machine needs a repair; a repair moves the enable chain;
     * so the fixed machine's walk 2 belongs to a different physical chain state than the
     * refused attempt's walk 1 and isolation. Letting the same challenge take a second
     * submission is precisely how evidence from two different walks gets spliced into one
     * transaction. A retry is a new challenge and a fresh walk 1 -> isolation -> walk 2. */
    transaction bad;
    bad.isolation.prev_probe = enm::probe_state::ack;

    pf::gate g;
    const pf::challenge c{g.issue()};
    zassert_equal(g.evaluate(bad.evidence(), c).reason, pf::refusal::isolation_prev_answered);
    zassert_equal(g.outstanding_nonce(), 0u, "a refusal spends the challenge too");

    const transaction fixed;
    const pf::verdict spliced{g.evaluate(fixed.evidence(), c)};
    zassert_false(spliced.granted());
    zassert_equal(spliced.reason, pf::refusal::challenge_consumed);

    /* The legitimate retry path, and the only one. */
    const pf::challenge again{g.issue()};
    zassert_true(g.evaluate(fixed.evidence(), again).granted());
}

ZTEST(tof_mapping_proof, test_moving_a_token_empties_the_source)
{
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};
    pf::verdict v{g.evaluate(t.evidence(), c)};
    zassert_true(v.token.valid());

    const pf::proof_token taken{static_cast<pf::proof_token &&>(v.token)};
    zassert_true(taken.valid());
    zassert_equal(taken.nonce(), c.nonce());
    /* One-shot expressed in the type: whoever handed the token over no longer holds one. */
    zassert_false(v.token.valid());
    zassert_equal(v.token.nonce(), 0u);
}

ZTEST(tof_mapping_proof, test_the_production_spec_proves_with_the_frozen_role_table)
{
    /* This case used to assert the opposite -- role_unknown -- and its comment said: if this ever
     * starts passing without that table being frozen, something has guessed. The table is now frozen
     * from dasher_connectivity.png, so the assertion flips, and what it has to pin flips with it:
     * that the evaluator accepts the SHIPPED spec, and that the four roles it accepts are a
     * permutation of the four corners rather than any four values that happen to parse. */
    transaction t;
    t.spec = lexxhard::tof_chain::dasher_spec();
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);
    zassert_equal(refused(t), pf::refusal::none, "the shipped spec no longer proves");

    bool seen[4]{};
    for (size_t i{2}; i < t.spec.positions; ++i) {
        const int8_t src{pf::source_id_of(t.spec.at[i].role)};
        zassert_true(src >= 0, "position %zu has no source id", i + 1);
        zassert_false(seen[src], "source id %d claimed twice", src);
        seen[src] = true;
    }
    for (int i{0}; i < 4; ++i)
        zassert_true(seen[i], "source id %d unclaimed", i);
}

ZTEST(tof_mapping_proof, test_a_duplicated_role_is_refused)
{
    transaction t;
    t.spec.at[4].role = t.spec.at[2].role;
    zassert_equal(refused(t), pf::refusal::role_duplicate);
}

ZTEST(tof_mapping_proof, test_an_incomplete_first_walk_is_refused)
{
    transaction t;
    t.walk1.status = enm::chain_status::degraded;
    zassert_equal(refused(t), pf::refusal::walk1_not_complete);
}

ZTEST(tof_mapping_proof, test_an_incomplete_second_walk_is_refused)
{
    transaction t;
    t.walk2.status = enm::chain_status::failed;
    zassert_equal(refused(t), pf::refusal::walk2_not_complete);
}

ZTEST(tof_mapping_proof, test_a_position_that_never_verified_is_refused)
{
    /* `absent` is the pos6 failure class from the DS20001 bring-up: enable never arrived.
     * A walk carrying it cannot be `complete`, so this arrangement is doubly impossible on
     * hardware -- which is why the check cannot rest on the status alone. */
    transaction t;
    t.walk2.at[5].verdict = enm::outcome::absent;
    zassert_equal(refused(t), pf::refusal::position_not_verified);
}

ZTEST(tof_mapping_proof, test_a_walk_whose_spec_the_enumerator_rejected_is_refused)
{
    transaction t;
    t.walk2.spec = enm::spec_error::target_duplicate;
    zassert_equal(refused(t), pf::refusal::walk_spec_rejected);
}

ZTEST(tof_mapping_proof, test_the_live_chain_must_answer_on_its_assigned_addresses)
{
    transaction t;
    t.walk1.at[4].address = 0x40;
    t.walk2.at[4].address = 0x40;
    zassert_equal(refused(t), pf::refusal::address_mismatch);
}

ZTEST(tof_mapping_proof, test_two_positions_on_one_address_are_refused)
{
    /* The spec's targets are distinct, so this needs a spec the enumerator would have
     * rejected AND walks that claim it accepted -- a fabrication on both counts. It is
     * checked because a proof module that trusts its inputs is not a proof module. */
    transaction t;
    t.spec.at[5].target_addr = t.spec.at[4].target_addr;
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);
    zassert_equal(refused(t), pf::refusal::address_not_distinct);
}

ZTEST(tof_mapping_proof, test_the_live_chain_identity_must_match_the_model)
{
    transaction t;
    t.walk1.at[3].seen = kL7Id; // an L7 identity at an L4 position
    t.walk2.at[3].seen = kL7Id;
    zassert_equal(refused(t), pf::refusal::identity_mismatch);
}

ZTEST(tof_mapping_proof, test_the_two_walks_must_prove_the_same_chain)
{
    /* The check that carries the isolation evidence forward. Walk 2 is entirely consistent
     * with the spec here -- only walk 1 differs -- so nothing else can catch it. */
    transaction t;
    t.walk1.at[2].address = 0x41;
    zassert_equal(refused(t), pf::refusal::fingerprint_mismatch);
}

ZTEST(tof_mapping_proof, test_a_walk_that_read_different_identity_bytes_is_a_mismatch)
{
    transaction t;
    t.walk1.at[0].seen = enm::id_bytes{0xf0, 0x03}; // a different L7 revision
    zassert_equal(refused(t), pf::refusal::fingerprint_mismatch);
}

ZTEST(tof_mapping_proof, test_isolation_must_be_attempted)
{
    transaction t;
    t.isolation.attempted = false;
    zassert_equal(refused(t), pf::refusal::isolation_not_attempted);
}

ZTEST(tof_mapping_proof, test_a_transport_error_during_isolation_proves_nothing)
{
    transaction tail;
    tail.isolation.tail_probe = enm::probe_state::transport_error;
    zassert_equal(refused(tail), pf::refusal::isolation_transport_error);

    /* Separately on the neighbour's probe, because that is the half whose whole job is to
     * prove an absence -- and an absence is exactly what a transport error cannot prove. */
    transaction prev;
    prev.isolation.prev_probe = enm::probe_state::transport_error;
    zassert_equal(refused(prev), pf::refusal::isolation_transport_error);
}

ZTEST(tof_mapping_proof, test_a_silent_tail_is_refused)
{
    transaction t;
    t.isolation.tail_probe = enm::probe_state::nack;
    zassert_equal(refused(t), pf::refusal::isolation_no_answer);
}

ZTEST(tof_mapping_proof, test_the_tail_answering_its_neighbours_address_is_the_silent_merge)
{
    /* The failure this gate exists for, and the one observed 4/4 on DS20001 before the
     * 50 ohm series resistor: one clock pulse enabled two boards, both were written to one
     * address, and six identity reads still matched. */
    transaction t;
    t.isolation.answering_addr = t.spec.at[4].target_addr;
    zassert_equal(refused(t), pf::refusal::isolation_wrong_address);
}

ZTEST(tof_mapping_proof, test_the_isolated_tail_must_be_an_l4)
{
    transaction t;
    t.isolation.seen = kL7Id;
    zassert_equal(refused(t), pf::refusal::isolation_identity);

    transaction unread;
    unread.isolation.id_read_ok = false;
    zassert_equal(refused(unread), pf::refusal::isolation_identity);
}

ZTEST(tof_mapping_proof, test_a_neighbour_that_still_answers_means_isolation_did_not_take)
{
    transaction t;
    t.isolation.prev_probe = enm::probe_state::ack;
    zassert_equal(refused(t), pf::refusal::isolation_prev_answered);
}

ZTEST(tof_mapping_proof, test_silence_proven_at_the_wrong_address_is_refused)
{
    /* Otherwise the negative half is free: probe an address nobody owns, collect a clean
     * NACK, and prove nothing about the neighbour. */
    transaction t;
    t.isolation.prev_addr = 0x42;
    zassert_equal(refused(t), pf::refusal::isolation_prev_addr_wrong);
}

ZTEST(tof_mapping_proof, test_missing_evidence_is_refused)
{
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};

    pf::evidence ev{t.evidence()};
    ev.walk2 = nullptr;
    zassert_equal(g.evaluate(ev, c).reason, pf::refusal::missing_evidence);
    /* Spent, like every other outcome. Presenting a challenge is what spends it, so there is
     * no cheap probe that keeps an attempt alive. */
    zassert_equal(g.outstanding_nonce(), 0u);

    pf::evidence no_spec{t.evidence()};
    no_spec.spec = nullptr;
    const pf::challenge second{g.issue()};
    zassert_equal(g.evaluate(no_spec, second).reason, pf::refusal::missing_evidence);
    zassert_equal(g.outstanding_nonce(), 0u);
}

ZTEST(tof_mapping_proof, test_a_chain_too_short_to_isolate_is_not_provable)
{
    transaction t;
    t.spec.positions = 1;
    t.spec.at[0] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, false);
    zassert_equal(bench_refused(t), pf::refusal::spec_too_few_positions);
    /* And it is not provable by the other door either, for a blunter reason. */
    zassert_equal(refused(t), pf::refusal::spec_not_commissioning_profile);
}

ZTEST(tof_mapping_proof, test_a_chain_with_no_cliff_sensor_at_all_is_refused)
{
    transaction t;
    t.spec.positions = 2; // the two L7s only
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);
    zassert_equal(bench_refused(t), pf::refusal::spec_no_cliff);
    zassert_equal(refused(t), pf::refusal::spec_not_commissioning_profile);
}

ZTEST(tof_mapping_proof, test_a_chain_whose_tail_is_not_a_cliff_sensor_is_refused)
{
    /* Distinct from the case above, and the distinction is the diagnostic: this chain HAS
     * cliff sensors, they are just not where isolation needs one. */
    transaction t;
    t.spec.positions = 3;
    t.spec.at[0] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    t.spec.at[1] = {enm::model::l4cx, 0x2D, -1, enm::l4_role::rear_left};
    t.spec.at[2] = {enm::model::l7cx, 0x2A, 0, enm::l4_role::unknown};
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, false);
    t.isolation = clean_isolation(t.spec);
    zassert_equal(bench_refused(t), pf::refusal::spec_no_tail_l4);
    zassert_equal(refused(t), pf::refusal::spec_not_commissioning_profile);
}

ZTEST(tof_mapping_proof, test_a_walk_reporting_a_different_position_count_is_refused)
{
    /* Two different chains, not one chain to compare. The enumerator zeroes `positions`
     * when it rejects a spec, so this is also the shape a rejected walk takes if the
     * spec_error field is ever cleared by hand. */
    transaction t;
    t.walk2.positions = 5;
    zassert_equal(refused(t), pf::refusal::walk_position_count);
}

ZTEST(tof_mapping_proof, test_a_bench_chain_cannot_be_proven_through_the_commissioning_door)
{
    /* The P1 this test exists for: before it, a three-board bench chain produced the same
     * proof_token as the product, and a token is a token -- whatever the authority does with
     * it later, the object capable of opening PROVEN had already been minted. */
    transaction t;
    t.spec.positions = 3;
    t.spec.at[0] = {enm::model::l7cx, 0x2A, 0, enm::l4_role::unknown};
    t.spec.at[1] = {enm::model::l7cx, 0x2B, 1, enm::l4_role::unknown};
    t.spec.at[2] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);

    pf::gate g;
    const pf::challenge c{g.issue()};
    const pf::verdict v{g.evaluate(t.evidence(), c)};
    zassert_false(v.granted());
    zassert_equal(v.reason, pf::refusal::spec_not_commissioning_profile);
    zassert_false(v.token.valid());
}

ZTEST(tof_mapping_proof, test_the_same_bench_chain_still_gets_a_real_diagnostic_answer)
{
    /* Keeping the bench path useful is the other half of the P1: a chain that cannot be
     * proven should still be told whether it is wired correctly. This one is. */
    transaction t;
    t.spec.positions = 3;
    t.spec.at[0] = {enm::model::l7cx, 0x2A, 0, enm::l4_role::unknown};
    t.spec.at[1] = {enm::model::l7cx, 0x2B, 1, enm::l4_role::unknown};
    t.spec.at[2] = {enm::model::l4cx, 0x2C, -1, enm::l4_role::front_left};
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    t.isolation = clean_isolation(t.spec);

    pf::gate g;
    const pf::bench_report b{g.evaluate_bench(t.evidence())};
    zassert_true(b.clean(), "reason %d", static_cast<int>(b.reason));
    zassert_equal(b.proven.positions, 3u);
    zassert_equal(b.proven.at[2].address, 0x2C);
    /* And it reports faults, so it is a real check and not a rubber stamp. */
    transaction merged{t};
    merged.isolation.answering_addr = t.spec.at[1].target_addr;
    zassert_equal(bench_refused(merged), pf::refusal::isolation_wrong_address);
}

ZTEST(tof_mapping_proof, test_a_bench_evaluation_never_touches_the_challenge)
{
    /* It authorises nothing, so it has nothing to spend -- and it must not be able to burn
     * an attempt the operator is in the middle of. */
    const transaction t;
    pf::gate g;
    const pf::challenge c{g.issue()};
    (void)g.evaluate_bench(t.evidence());
    zassert_equal(g.outstanding_nonce(), c.nonce());
    zassert_true(g.evaluate(t.evidence(), c).granted());
}

ZTEST(tof_mapping_proof, test_the_profile_requires_the_grid_sensors_first)
{
    /* Six boards, four cliff roles, and still not the profile: the models are in the wrong
     * order. Enumeration would pass and the mapping would be someone else's. */
    transaction t;
    t.spec.at[0] = {enm::model::l4cx, 0x2A, -1, enm::l4_role::front_left};
    t.spec.at[2] = {enm::model::l7cx, 0x2C, 0, enm::l4_role::unknown};
    t.walk1 = clean_walk(t.spec, false);
    t.walk2 = clean_walk(t.spec, true);
    zassert_equal(refused(t), pf::refusal::spec_not_commissioning_profile);
}

ZTEST(tof_mapping_proof, test_the_role_diagnostics_survive_the_profile_check)
{
    /* The reason the profile is checked in two halves: a chain with the right topology but an
     * incomplete role table must be told THAT, not "not the commissioning profile", which would
     * send someone looking at the hardware.
     *
     * The unknown role is now injected explicitly. It used to come free from dasher_spec(), which
     * carried l4_role::unknown -- but that made this refusal reachable only for as long as the
     * shipped table stayed unfrozen, and it is frozen now. A safety refusal must not lose its
     * coverage the moment the product configuration stops happening to trigger it. */
    transaction unknown_roles;
    unknown_roles.spec = lexxhard::tof_chain::dasher_spec();
    unknown_roles.spec.at[4].role = enm::l4_role::unknown;
    unknown_roles.walk1 = clean_walk(unknown_roles.spec, false);
    unknown_roles.walk2 = clean_walk(unknown_roles.spec, true);
    unknown_roles.isolation = clean_isolation(unknown_roles.spec);
    zassert_equal(refused(unknown_roles), pf::refusal::role_unknown);

    transaction duplicated;
    duplicated.spec.at[4].role = duplicated.spec.at[2].role;
    zassert_equal(refused(duplicated), pf::refusal::role_duplicate);
}
