/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_mapping_proof.hpp"

namespace lexxhard::tof_proof {

namespace {

// Does this verdict normalise to "verified" for a position of this model?
//
// The asymmetry is the contract's, and it is not a convenience: an L7 keeps its assigned
// address across an enable-low while powered, so the second walk legitimately finds it
// already at its target and reports `retained`. An L4's enable is reset-class -- a
// `retained` L4 means a device kept an address it should have lost, which contradicts the
// chain state model and is a fault, not a normalisation case.
bool normalises_to_verified(enm::model m, enm::outcome v, refusal &why)
{
    if (v == enm::outcome::enumerated)
        return true;
    if (v == enm::outcome::retained) {
        if (m == enm::model::l7cx)
            return true;
        why = refusal::l4_retained;
        return false;
    }
    why = refusal::position_not_verified;
    return false;
}

bool same_position(const position_fingerprint &a, const position_fingerprint &b)
{
    return a.position == b.position && a.expected == b.expected && a.address == b.address &&
           a.id.first == b.id.first && a.id.second == b.id.second &&
           a.source_id == b.source_id && a.role == b.role && a.verified == b.verified;
}

} // namespace

bool same(const fingerprint &a, const fingerprint &b)
{
    if (a.positions != b.positions)
        return false;
    for (size_t i{0}; i < a.positions; ++i) {
        if (!same_position(a.at[i], b.at[i]))
            return false;
    }
    return true;
}

bool fingerprint_of(const enm::chain_spec &spec, const enm::chain_result &walk,
                    fingerprint &out, refusal &why)
{
    out = fingerprint{};

    /* A walk that reports a different number of positions from the spec it was run
     * against is not a chain to compare; it is two different chains. */
    if (walk.positions != spec.positions || spec.positions == 0 ||
        spec.positions > enm::chain_spec::kMaxPositions) {
        why = refusal::walk_position_count;
        return false;
    }

    for (size_t i{0}; i < spec.positions; ++i) {
        const enm::position_spec &ps{spec.at[i]};
        const enm::position_result &pr{walk.at[i]};

        if (!normalises_to_verified(ps.expected, pr.verdict, why))
            return false;

        /* Descriptive, deliberately. Nothing here is compared against the spec's target
         * address or expected identity -- `evaluate` does that, once, on the walk that
         * will actually produce data.
         *
         * The distinction is not stylistic. An earlier draft of this file validated every
         * position of every walk against the spec, and the effect was that both
         * fingerprints became functions of the spec alone: they could not differ, and the
         * equality check that carries the isolation evidence forward was silently always
         * true. A vacuous safety check is worse than a missing one, because it reads as
         * covered. */
        position_fingerprint &f{out.at[i]};
        f.position = static_cast<uint8_t>(i + 1);
        f.expected = ps.expected;
        f.address = pr.address;
        f.id = pr.seen;
        /* These two are spec-derived, so today they are equal between the two walks by
         * construction. They are in the fingerprint because the contract names them, and
         * because the day a caller passes a per-walk spec, that difference must fail the
         * comparison rather than pass unnoticed. */
        f.source_id = ps.source_id;
        f.role = ps.role;
        f.verified = true;
    }

    out.positions = spec.positions;
    why = refusal::none;
    return true;
}

challenge gate::issue()
{
    /* Never zero, on wrap or otherwise: zero is the value a fabricated challenge has, and
     * it must stay unusable. */
    if (next_ == 0)
        next_ = 1;
    current_ = next_++;
    consumed_ = false;
    return challenge{current_};
}

verdict gate::evaluate(const evidence &ev, const challenge &c)
{
    verdict v{};

    if (ev.spec == nullptr || ev.walk1 == nullptr || ev.walk2 == nullptr ||
        ev.spec->positions == 0) {
        v.reason = refusal::missing_evidence;
        return v;
    }

    /* The challenge first, before any evidence is looked at. Order matters for the
     * diagnostic: "you presented evidence for a superseded attempt" is a different
     * problem from anything about the chain, and reporting a chain fault for it would send
     * an operator to the hardware. */
    if (!c.valid()) {
        v.reason = refusal::challenge_invalid;
        return v;
    }
    if (c.nonce() != current_) {
        v.reason = refusal::challenge_stale;
        return v;
    }
    if (consumed_) {
        v.reason = refusal::challenge_consumed;
        return v;
    }

    const enm::chain_spec &spec{*ev.spec};

    /* Isolation's negative half needs a neighbour whose silence can be proven, so a
     * single-position chain is not provable by this definition. Refusing is right: the
     * alternative is a proof that quietly means less on some chains than on others. */
    if (spec.positions < 2) {
        v.reason = refusal::spec_too_few_positions;
        return v;
    }

    /* "No cliff sensor at all" comes before "the tail is not a cliff sensor", and the order
     * is the whole reason both refusals exist. Reversed, a chain of two grid sensors fails
     * the tail check and is told its tail is wrong -- true, and useless, since it has no
     * cliff sensor to put there; and the no-cliff branch was then unreachable, which is
     * what a refusal-coverage check over the tests caught. */
    size_t cliff_positions{0};
    for (size_t i{0}; i < spec.positions; ++i) {
        if (spec.at[i].expected == enm::model::l4cx)
            ++cliff_positions;
    }
    if (cliff_positions == 0) {
        v.reason = refusal::spec_no_cliff;
        return v;
    }

    /* Tail isolation is defined on the tail, and the tail of a cliff chain is an L4. A spec
     * ending in an L7 is not a chain this proof knows how to prove -- a refusal, not
     * something to work around by isolating a different position.
     *
     * Deliberately not a hard-coded six: the position count belongs to the injected spec,
     * so a bench chain with fewer boards stays provable and the product's length is not
     * baked into the safety logic. */
    const size_t tail{spec.positions - 1};
    if (spec.at[tail].expected != enm::model::l4cx) {
        v.reason = refusal::spec_no_tail_l4;
        return v;
    }

    /* Both walks must have been run against a spec the enumerator itself accepted. Cheap,
     * and it closes the one hole this module cannot close on its own: it does not validate
     * the spec (the validator is internal to enumerate()), so without this a caller could
     * present a spec the enumerator would have rejected outright. */
    if (ev.walk1->spec != enm::spec_error::none || ev.walk2->spec != enm::spec_error::none) {
        v.reason = refusal::walk_spec_rejected;
        return v;
    }

    if (ev.walk1->status != enm::chain_status::complete) {
        v.reason = refusal::walk1_not_complete;
        return v;
    }
    if (ev.walk2->status != enm::chain_status::complete) {
        v.reason = refusal::walk2_not_complete;
        return v;
    }

    /* The role table, before the electrical checks. A machine can be electrically perfect
     * and still unprovable: the masks a consumer reads are keyed by source_id, not by
     * chain position, so without the frozen mounting roles nothing downstream can be
     * filled honestly. Electrical enumeration proves the type sequence; it cannot prove
     * which of four identical carriers is mounted where. */
    for (size_t i{0}; i < spec.positions; ++i) {
        if (spec.at[i].expected != enm::model::l4cx)
            continue;
        if (spec.at[i].role == enm::l4_role::unknown) {
            v.reason = refusal::role_unknown;
            return v;
        }
        for (size_t j{i + 1}; j < spec.positions; ++j) {
            if (spec.at[j].expected == enm::model::l4cx &&
                spec.at[j].role == spec.at[i].role) {
                v.reason = refusal::role_duplicate;
                return v;
            }
        }
    }

    fingerprint fp1{}, fp2{};
    refusal why{refusal::none};

    if (!fingerprint_of(spec, *ev.walk1, fp1, why)) {
        v.reason = why;
        return v;
    }
    if (!fingerprint_of(spec, *ev.walk2, fp2, why)) {
        v.reason = why;
        return v;
    }

    /* The live chain against the spec: each position answers on the address the mapping
     * assigns it, with a type-appropriate identity. Walk 2 and not walk 1, because walk 2
     * is the chain the acquisition path will use; walk 1 is held to walk 2 by the
     * fingerprint comparison below, which is what transfers the isolation evidence.
     *
     * The identity check is belt and braces over the enumerator, deliberately: the
     * enumerator already refuses a wrong model, but "type-appropriate identity read at
     * each position" is one of the three results the contract names, and a criterion this
     * module claims to check must be checked here rather than inherited from a producer.
     *
     * Its limit, stated because it is easy to over-read: these are MODEL identity bytes,
     * not serial numbers. Four identical L4 carriers are indistinguishable to this check,
     * so it can never detect two of them being swapped between positions. That is exactly
     * why the mounting roles come from a frozen document and are refused when unknown. */
    for (size_t i{0}; i < fp2.positions; ++i) {
        if (fp2.at[i].address != spec.at[i].target_addr) {
            v.reason = refusal::address_mismatch;
            return v;
        }
        if (!enm::id_matches(spec.at[i].expected, fp2.at[i].id)) {
            v.reason = refusal::identity_mismatch;
            return v;
        }
    }

    /* Mutually distinct addresses. The enumerator's own spec validation rejects duplicate
     * targets, and the walks' `spec` field is checked above, so reaching this needs a
     * fabricated pair of results -- which is precisely the case a proof module should not
     * take on trust. */
    for (size_t i{0}; i < fp2.positions; ++i) {
        for (size_t j{i + 1}; j < fp2.positions; ++j) {
            if (fp2.at[i].address == fp2.at[j].address) {
                v.reason = refusal::address_not_distinct;
                return v;
            }
        }
    }

    if (!same(fp1, fp2)) {
        v.reason = refusal::fingerprint_mismatch;
        return v;
    }

    /* Isolation last, because it is the criterion whose meaning depends on everything
     * above: "the tail answers its own address" is only informative once we know which
     * address is the tail's and that both walks agree on it. */
    const isolation_observation &iso{ev.isolation};

    if (!iso.attempted) {
        v.reason = refusal::isolation_not_attempted;
        return v;
    }
    if (iso.tail_probe == enm::probe_state::transport_error ||
        iso.prev_probe == enm::probe_state::transport_error) {
        v.reason = refusal::isolation_transport_error;
        return v;
    }
    if (iso.tail_probe != enm::probe_state::ack || iso.answering_addr == 0) {
        v.reason = refusal::isolation_no_answer;
        return v;
    }
    if (iso.answering_addr != spec.at[tail].target_addr) {
        /* The silent merge this whole gate exists to catch: two devices were written to
         * one address, so the isolated tail answers on its neighbour's. */
        v.reason = refusal::isolation_wrong_address;
        return v;
    }
    if (!iso.id_read_ok || !enm::id_matches(enm::model::l4cx, iso.seen)) {
        v.reason = refusal::isolation_identity;
        return v;
    }
    /* The negative half, and it has to be proven at the RIGHT address. A caller that
     * probed some unrelated address would collect a clean NACK for free and the check
     * would pass while proving nothing about the neighbour. */
    if (iso.prev_addr != spec.at[tail - 1].target_addr) {
        v.reason = refusal::isolation_prev_addr_wrong;
        return v;
    }
    /* An address that still answers while its device is supposed to be disabled means the
     * isolation did not take, and then the positive half proves nothing -- the tail could
     * have been answering all along with its neighbour awake beside it. Only a clean NACK
     * proves silence. */
    if (iso.prev_probe != enm::probe_state::nack) {
        v.reason = refusal::isolation_prev_answered;
        return v;
    }

    /* Everything held. The challenge is spent here and not before: a refused attempt
     * leaves it outstanding so evidence can be re-presented after a repair, and a granted
     * one can never be repeated. */
    consumed_ = true;
    v.reason = refusal::none;
    v.token = proof_token{c.nonce(), fp2};
    return v;
}

} // namespace lexxhard::tof_proof
