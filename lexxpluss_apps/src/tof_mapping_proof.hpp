/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * The mapping proof: the only thing in this firmware that may decide the cliff
 * position-to-source mapping has been proven.
 *
 * It decides, and it does nothing else. It touches no bus, holds no lock, starts no
 * thread, and cannot make the subsystem publish anything. What it produces is a token,
 * and a token by itself changes no behaviour: the authority that will consume it does not
 * exist yet, and the PROVEN clamp in tof_acquisition is untouched by this file.
 *
 * WHAT THE CONTRACT REQUIRES, AND WHY IT IS A TRANSACTION
 *
 * `commissioning-2026-08-18c`, *What proves mapping_state == PROVEN* and *The proof is a
 * transaction*. The short version, because the shape is not obvious:
 *
 * Tail isolation needs every position but the tail disabled, and on the L4 carriers the
 * enable line is reset-class -- disabling a position returns that device to the default
 * address. So the chain that ends up holding the live addresses is ALWAYS enumerated
 * after the isolation, and can never itself hold an isolation result. The three pieces of
 * evidence the contract demands cannot come from one chain state, and an implementation
 * written as though they could will fail on every healthy machine. The tempting repair is
 * to drop a criterion.
 *
 * So the proof is one transaction -- walk 1, isolation, walk 2 -- and what carries the
 * isolation evidence forward to the chain that will actually produce data is an explicit
 * semantic fingerprint that both walks must produce identically. Not an unstated
 * assumption that nothing changed in between.
 *
 * WHAT IT REFUSES TO BE GIVEN
 *
 * The inputs are the enumerator's own `chain_result` and `chain_spec`, plus raw
 * observations from the isolation step. There is deliberately no digested "evidence
 * summary" input: a caller that could hand this module `tail_ok = true` would be the
 * component actually deciding, and this module would be decoration. Every verdict below
 * is computed here from observations.
 *
 * A bus-wide address scan count is not an input and must never become one. It was
 * observed on DS20001 to intermittently miss two addresses while the same devices passed
 * 200/200 probes and 100/100 register transfers; the cause is unexplained and an
 * unexplained intermittent measurement cannot gate a safety mapping claim.
 *
 * WHAT IS NOT CHECKED HERE, AND WHERE IT IS
 *
 * "Nothing left at the default address" is not a separate input, because a `complete`
 * walk already proves it: the enumerator re-runs its census after every successful
 * readdress and requires the default address to answer with a clean NACK, including at
 * the last position. Adding a field for it would invite a caller to assert it
 * independently of the walk that is supposed to establish it.
 *
 * The epoch is not here either. A proof says the mapping is proven; issuing an epoch and
 * resetting the cycle counter is one transaction owned by the authority, and under the
 * commissioning profile the epoch comes from the host. See the contract's
 * *Commissioning mapping_epoch issuance*.
 *
 * SYNCHRONISATION
 *
 * None, on purpose. `gate` is a plain object with no internal lock; the authority that
 * owns one is the single caller and serialises it under its own lock. A lock in here
 * would be a second authority over the same state.
 */

#include <stddef.h>
#include <stdint.h>

#include "tof_enumerator.hpp"

namespace lexxhard::tof_proof {

namespace enm = tof_enum;

// Why a proof was refused. One reason per defect, because "proof failed" is useless to a
// commissioning operator standing at the machine: the difference between "position 6
// answered position 5's address" and "position 4's role is unknown" is the difference
// between a hardware fault and a missing document.
enum class refusal : uint8_t {
    none = 0,
    missing_evidence,        // a null spec or walk pointer, or an empty spec
    challenge_invalid,       // the default-constructed challenge, which authorises nothing
    challenge_stale,         // not the challenge this gate most recently issued
    challenge_consumed,      // that challenge has already been evaluated, pass or fail
    spec_not_commissioning_profile,  // not L7,L7,L4,L4,L4,L4 with each cliff role once
    spec_no_tail_l4,         // the tail position is not an L4: isolation is undefined
    spec_no_cliff,           // the spec carries no L4 at all
    spec_too_few_positions,  // fewer than two: isolation has no neighbour to silence
    walk_position_count,     // a walk reports a different position count from the spec
    walk_spec_rejected,      // the enumerator rejected the spec this walk ran against
    walk1_not_complete,      // status != complete
    walk2_not_complete,
    position_not_verified,   // a verdict that does not normalise to verified
    l4_retained,             // an L4 reported retained: contradicts the reset-class enable
    address_mismatch,        // a verified position does not answer on its spec target
    address_not_distinct,    // two positions ended up on one address
    identity_mismatch,       // identity bytes are not the expected model's
    role_unknown,            // a cliff position with no frozen mounting role
    role_duplicate,          // two cliff positions claiming one role
    fingerprint_mismatch,    // the two walks did not prove the same chain
    isolation_not_attempted,
    isolation_transport_error,   // a probe that proves neither presence nor vacancy
    isolation_no_answer,         // the tail did not answer at all
    isolation_wrong_address,     // the tail answered on someone else's address: silent merge
    isolation_identity,          // the tail answered but is not the expected L4
    isolation_prev_addr_wrong,   // the silence was proven at some address other than the neighbour's
    isolation_prev_answered,     // the disabled neighbour still answers: isolation did not take
};

// One position's contribution to the fingerprint. The contract names these fields; this
// struct is not free to carry more, and in particular carries no pulse count and no
// control history -- two walks may reach the same proven configuration by different pulse
// counts, and comparing those manufactures failures with no bearing on the mapping.
struct position_fingerprint {
    uint8_t position{0};                       // 1-based, as the operator counts them
    enm::model expected{enm::model::l4cx};
    uint8_t address{0};
    enm::id_bytes id{};
    int8_t source_id{-1};                      // grid source, -1 for none
    enm::l4_role role{enm::l4_role::unknown};
    bool verified{false};
};

struct fingerprint {
    size_t positions{0};
    position_fingerprint at[enm::chain_spec::kMaxPositions]{};
};

bool same(const fingerprint &a, const fingerprint &b);

// Does this fingerprint describe the commissioning profile -- six positions, two grid
// sensors then four cliff sensors, each cliff role used exactly once?
//
// Exported because the authority must re-check it on the token it is handed rather than
// trust that whoever produced the token checked. The definition of the profile lives in
// this module, so the check belongs here too; a second copy in the authority is exactly how
// the two drift.
bool is_commissioning_profile(const fingerprint &fp);

// The wire contract's role -> source_id table: front_left 0, rear_left 1, rear_right 2,
// front_right 3. Returns -1 for unknown, which is not a source id but the absence of one.
//
// ONE definition, exported rather than copied. Two callers need it now -- the authority, to build
// the per-cycle masks, and the runtime, to fill source_desc::role_id from an installed mapping --
// and a second copy is how they drift apart while both still compile. Written as an explicit
// switch and never as arithmetic on the enum: `static_cast<int>(role) - 1` happens to agree today
// and would silently follow any reordering of an enum the contract has no say over, producing a
// source_id that points at a different corner of the machine.
int8_t source_id_of(enm::l4_role role);

// What was observed with only the tail position enabled. Raw observations, not verdicts:
// the evaluator decides what they mean.
struct isolation_observation {
    bool attempted{false};
    // "The isolated tail answered SOMEWHERE", and `answering_addr` is where. Not "the tail's own
    // address ACKed": in the failure this check exists for, the tail answers its NEIGHBOUR's
    // address because both were written to it, and reporting that as "the tail did not answer"
    // would send an operator looking for a dead board instead of a merge.
    enm::probe_state tail_probe{enm::probe_state::transport_error};
    // Which address actually answered -- possibly the previous position's. Zero when none did.
    uint8_t answering_addr{0};
    // Identity read AT THE ADDRESS THAT ANSWERED, not at the expected one: reading at the expected
    // one would fail in the merge case and lose the distinction the isolation is for.
    bool id_read_ok{false};
    enm::id_bytes seen{};
    // The previous position is disabled and must be silent. A clean NACK is the only
    // answer that proves it; a transport error proves nothing.
    enm::probe_state prev_probe{enm::probe_state::transport_error};
    uint8_t prev_addr{0};
};

struct evidence {
    const enm::chain_spec *spec{nullptr};
    const enm::chain_result *walk1{nullptr};
    const enm::chain_result *walk2{nullptr};
    isolation_observation isolation{};
};

// A one-shot challenge. The default-constructed value is deliberately valid C++ and
// deliberately worthless: nonce 0 authorises nothing, so a caller that fabricates a
// challenge gets a refusal rather than a proof.
class challenge {
public:
    challenge() = default;
    uint32_t nonce() const { return nonce_; }
    bool valid() const { return nonce_ != 0; }

private:
    explicit challenge(uint32_t n) : nonce_{n} {}
    uint32_t nonce_{0};
    friend class gate;
};

/*
 * The proof of one transaction, bound to the challenge that authorised it.
 *
 * Move-only, and the move empties the source. That is the one-shot property expressed in
 * the type system rather than in a comment: a consumer takes it by value or by rvalue
 * reference, and whoever held it before no longer has anything.
 *
 * The default constructor is public and yields nonce 0 -- an object that authorises
 * nothing. Only `gate` can construct one with a non-zero nonce, which is the property
 * that matters: a token that authorises anything came from an evaluation. There is no
 * set_proven(), no force flag and no constructor taking "trust me".
 */
class proof_token {
public:
    proof_token() = default;
    proof_token(const proof_token &) = delete;
    proof_token &operator=(const proof_token &) = delete;

    proof_token(proof_token &&other) noexcept
        : nonce_{other.nonce_}, proven_{other.proven_}
    {
        other.nonce_ = 0;
    }

    proof_token &operator=(proof_token &&other) noexcept
    {
        if (this != &other) {
            nonce_ = other.nonce_;
            proven_ = other.proven_;
            other.nonce_ = 0;
        }
        return *this;
    }

    bool valid() const { return nonce_ != 0; }
    uint32_t nonce() const { return nonce_; }

    // The chain this token proves. The authority needs it to fill the masks the wire
    // contract keys by source_id, and to bring up the sensors on the addresses that were
    // actually proven rather than on the ones the spec hoped for.
    const fingerprint &proven() const { return proven_; }

private:
    proof_token(uint32_t nonce, const fingerprint &fp) : nonce_{nonce}, proven_{fp} {}
    uint32_t nonce_{0};
    fingerprint proven_{};
    friend class gate;
};

struct verdict {
    refusal reason{refusal::missing_evidence};
    proof_token token{};
    bool granted() const { return reason == refusal::none && token.valid(); }
};

/*
 * The result of a diagnostic evaluation on a chain that is not the commissioning profile.
 *
 * It carries no token, and that is the whole design: a bench chain of three boards can be
 * checked against every rule this module knows, and still cannot produce anything that
 * could later open PROVEN. The commissioning profile is a fixed topology -- two grid
 * sensors then four cliff sensors, each cliff role used exactly once -- and a proof of some
 * other chain is not a weaker proof of the product, it is a proof of a different machine.
 *
 * Not enforced by a flag on the token, deliberately. A `scope` field would mean a bench run
 * does mint a PROVEN-capable object and something downstream is trusted to look at the
 * field. Here there is no object to check.
 */
struct bench_report {
    refusal reason{refusal::missing_evidence};
    fingerprint proven{};
    bool clean() const { return reason == refusal::none; }
};

/*
 * Issues challenges and evaluates evidence against them.
 *
 * The order is the safety property: a challenge is issued BEFORE the first enable line
 * moves, and a proof is only accepted against the challenge outstanding at that moment.
 * Evidence collected under an older challenge cannot be presented later, and a second
 * attempt invalidates the first attempt's challenge by issuing a new one -- so a token
 * from a previous, possibly stale, chain state can never be committed after a new attempt
 * has begun.
 */
class gate {
public:
    // Invalidates any outstanding challenge. Deliberately not called "reset": the point
    // is that the previous challenge dies here.
    challenge issue();

    // ONE EVALUATION PER CHALLENGE, pass or fail. Presenting a challenge spends it, before
    // any evidence is looked at.
    //
    // The earlier version left a refused challenge outstanding so an operator could repair
    // the machine and re-present evidence. That was wrong, and not subtly: a repair means
    // the enable chain moved, so the second attempt's walk 2 belongs to a different
    // physical chain state than the first attempt's walk 1 and isolation. Accepting
    // evidence piecemeal against one challenge is exactly how evidence from two different
    // walks gets spliced into one transaction, which is the thing the transaction exists to
    // prevent. A retry is a new issue() and a fresh walk 1 -> isolation -> walk 2.
    //
    // Only the commissioning profile can be proven here; anything else is refused with
    // `spec_not_commissioning_profile`. Diagnostics use evaluate_bench().
    verdict evaluate(const evidence &ev, const challenge &c);

    // Diagnostic evaluation of any chain shape. Takes no challenge and returns no token,
    // because it authorises nothing -- there is no attempt to bind it to. Everything else
    // is checked exactly as evaluate() checks it, so a bench chain still gets a real answer
    // about its wiring.
    bench_report evaluate_bench(const evidence &ev);

    // For diagnostics and for the authority's own assertions. Not an authorisation path.
    uint32_t outstanding_nonce() const { return consumed_ ? 0 : current_; }

private:
    uint32_t next_{1};
    uint32_t current_{0};
    bool consumed_{true};
};

// Exposed for the authority and for tests: builds the semantic fingerprint of one walk,
// with the normalisation the contract defines (an L7 that kept its address across an
// enable-low reports `retained` where the first walk reported `enumerated`, and both mean
// verified; an L4 reporting `retained` does not normalise, it is a fault). Returns false
// with `why` set when the walk cannot produce a fingerprint at all.
//
// Descriptive only: it records what a walk observed and does NOT hold it against the
// spec's target addresses or expected identities. `evaluate` does that once, on the walk
// that will produce data. Validating both walks against the spec here would make the two
// fingerprints functions of the spec alone, and the equality check that carries the
// isolation evidence forward would be always true.
bool fingerprint_of(const enm::chain_spec &spec, const enm::chain_result &walk,
                    fingerprint &out, refusal &why);

}  // namespace lexxhard::tof_proof
