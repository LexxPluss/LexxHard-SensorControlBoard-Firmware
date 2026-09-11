/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * The mapping authority: the single owner of "is the cliff mapping proven, and under which
 * epoch".
 *
 * tof_mapping_proof decides whether a transaction is sound. This decides what the rest of
 * the system is told, and it is the only component allowed to say PROVEN. The split matters
 * because the two answer different questions: a sound proof of a chain that is not the one
 * acquisition is configured for must still be refused, and only something holding both the
 * proof and the runtime configuration can notice.
 *
 * IT STILL CANNOT PUBLISH A MEASUREMENT
 *
 * tof_acq::effective_mapping_state() clamps PROVEN unconditionally, and this commit does not
 * touch it. A committed proof here is visible in the health snapshot and nowhere else. The
 * clamp lifts in its own commit, once the role table is frozen and the cycle health frame
 * exists.
 *
 * THE GATE IS PRIVATE, ON PURPOSE
 *
 * The authority owns the tof_proof::gate and never hands it out. Everything must go through
 * begin_proof(): an external caller holding a gate could issue its own challenge, evaluate
 * evidence against it and arrive with a token that no revocation preceded -- which is the
 * whole safety property of the sequence, not a detail of it.
 *
 * ORDER OF A COMMIT, WHICH IS NORMATIVE
 *
 *   install the proven mapping -> issue the epoch -> reset cycle_seq -> publish PROVEN
 *
 * PROVEN is published last and atomically. Any earlier step failing leaves the state exactly
 * as it was, non-PROVEN, with nothing half-installed: a consumer that saw PROVEN before the
 * epoch was set would correlate against an epoch that no measurement will carry.
 *
 * WHY THE SNAPSHOT IS ONE ATOMIC WORD RATHER THAN A MUTEX
 *
 * The health path reads this state and MUST keep publishing while a proof is running -- and
 * a proof runs two full enumerations, which take seconds. A mutex held across a commit, or
 * across the walks, would stall the heartbeat exactly when a consumer most needs it. So the
 * whole snapshot is packed into one atomic word: readers do a single atomic_get and can
 * never observe a half-updated state, without ever blocking.
 */

#include <stdint.h>

#include "tof_enumerator.hpp"
#include "tof_mapping_proof.hpp"
#include "tof_mapping_state.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_authority {

namespace pf = tof_proof;

// Why a commit was refused. Every one of these leaves the authority non-PROVEN.
enum class commit_refusal : uint8_t {
    none = 0,
    not_initialised,
    no_attempt,                // commit without a begin_proof(), or after one was superseded
    invalid_token,             // the fabricated token: nonce 0, authorises nothing
    wrong_attempt,             // a token from an attempt that is no longer the current one
    // Re-checked here rather than trusted from the token. DELIBERATELY UNREACHABLE today and
    // therefore untested: the evaluator refuses anything but the commissioning profile, so
    // every valid token already describes one, and a token cannot be fabricated. It stays
    // because it is the lock that holds if the evaluator's profile check is ever loosened --
    // the alternative is that such a change opens the authority silently. Anything that makes
    // this reachable must come with a test that reaches it.
    not_commissioning_profile,
    runtime_mapping_mismatch,  // the proven chain is not the chain acquisition is configured for
    epoch_reused,              // used since power-on, so triples could alias
    epoch_space_exhausted,     // all 256 used this power cycle; wrapping would reuse
    acquisition_busy,          // begin_epoch() refused: cycles are still being produced
    epoch_install_failed,      // begin_epoch() failed for any other reason
    /* The runtime could not key its descriptors from this mapping. Inside the transaction on
     * purpose: keying used to happen after the commit had already published PROVEN, so a failure
     * left a PROVEN authority whose descriptors did not describe the proven chain -- and the only
     * thing standing between that and a measurement published under the wrong source_id was the
     * clamp. */
    mapping_install_failed,
};

// One consistent view of the authority's state. Decoded from a single atomic read.
struct snapshot {
    tof_acq::mapping_state state{tof_acq::mapping_state::not_ready};
    uint8_t epoch{0};
    // Keyed by source_id, from the chain a committed proof proved. Zero while non-PROVEN,
    // because nothing has proved a role; 0xF after a commit, because a proof cannot be granted
    // unless all four cliff positions enumerated to their own addresses with the expected model
    // ids and known, distinct roles.
    uint8_t enumerated_mask{0};
    uint8_t model_verified_mask{0};
    uint8_t chain_flags{0};        // contract flags bits 0-2
    uint8_t failing_position{0xFF}; // 1-6, or 0xFF for none
};

// Why an attempt could not be opened.
enum class begin_refusal : uint8_t {
    none = 0,
    not_initialised,
    acquisition_not_idle, // a proof moves enable lines; cycles must have stopped first
};

// What begin_proof() returns. The challenge is worthless unless `reason` is none, and pairing
// them means a caller cannot read the challenge without having been handed the reason.
struct attempt {
    pf::challenge challenge{};
    begin_refusal reason{begin_refusal::not_initialised};
    bool opened() const { return reason == begin_refusal::none && challenge.valid(); }
};

struct config {
    // The chain acquisition is configured for. A commit compares the proven fingerprint
    // against THIS, position by position, so a proof of some other chain cannot install
    // itself over the running configuration.
    const tof_enum::chain_spec *runtime_spec{nullptr};
    /* Keys the acquisition descriptors from the mapping being committed, and validates before it
     * writes. REQUIRED, not optional: a commit that published PROVEN without the descriptors that
     * make its source_ids mean anything would be publishing a mapping nothing acts on.
     *
     * Called with the chain lock already held by the commissioning session, which is the same lock
     * acquisition reads those descriptors under. Must return 0 only if EVERY position was keyed;
     * on any refusal it must leave none of them keyed, because a half-keyed table publishes some
     * corners correctly and others under the wrong name.
     */
    int (*install_mapping)(const pf::fingerprint &proven, uint8_t epoch){nullptr};
    // tof_acq::begin_epoch in production. Injected so the authority stays host-testable
    // without a bus, and so a test can make the cycle reset fail on demand.
    int (*begin_epoch)(){nullptr};
    // tof_acq::is_idle in production. Checked BEFORE a challenge is issued: a proof is about
    // to move enable lines, and discovering at commit time that acquisition was running means
    // the chain has already been re-addressed underneath a live reader.
    bool (*acquisition_idle)(){nullptr};
};

int init(const config &cfg);

// Opens an attempt, and revokes first.
//
// Revocation is immediate and unconditional: a chain about to be re-enumerated is a chain
// whose enable lines are about to move, so it stops being a chain that may produce
// measurements before the first line moves rather than after. A mapping that had been proven
// becomes LOST, which is not the same as UNKNOWN and must not be reported as it -- the
// consumer's recovery path differs.
//
// Refuses outright unless acquisition is idle. Checking here rather than at commit time is
// the difference between refusing to start and discovering the problem after two walks have
// already re-addressed the chain underneath a running reader.
//
// Any previously issued challenge and any token minted from it stop being committable here.
attempt begin_proof();

// Evaluates evidence against an attempt's challenge. A proxy, because the gate must not
// leave this module: a caller holding its own gate could issue a challenge that no
// revocation preceded.
//
// The token does exist briefly outside the authority, between this call and commit_proof().
// That is deliberate -- the caller needs the refusal detail to tell an operator what to fix,
// and folding the two calls together would collapse twenty-eight distinct refusals into
// "no". It costs nothing: a token authorises only the attempt it names, that attempt is
// already revoked, and commit_proof() empties it whether it accepts it or not.
pf::verdict evaluate(const pf::evidence &ev, const pf::challenge &c);

// Diagnostic evaluation of a chain that is not the commissioning profile. Mints nothing.
//
// REFUSED while an attempt is open, and `ran` is how that is reported. The evaluation itself is
// pure -- it reads evidence and touches no hardware -- but evidence for a bench chain can only
// have been collected by real walks, and walking a bench chain during a product attempt
// re-addresses the very chain that attempt is about. Rather than document a rule for the
// commissioning command to respect, the authority refuses: it can see whether an attempt is
// open and the command cannot forget to ask.
struct bench_result {
    bool ran{false};
    pf::bench_report report{};
};
bench_result evaluate_bench(const pf::evidence &ev);

// The only path to PROVEN. Takes the token by rvalue reference and empties it: a token is
// one attempt's worth of authority, and after this call the caller holds nothing.
//
// A token that MATCHES the open attempt closes it, whatever the outcome. One attempt buys one
// commit, exactly as one challenge buys one evaluation: retrying a refused commit against the same
// attempt would let a caller keep presenting the same evidence until some later check happened to
// pass. A forged or superseded token closes nothing -- it was never this attempt's.
commit_refusal commit_proof(pf::proof_token &&token, uint8_t host_epoch);

// Closes an attempt that will not be committed.
//
// Nonce-bound: it closes the attempt only if `c` IS the open attempt, so one caller cannot cancel
// another's. Returns true when it closed one.
//
// It exists because an attempt that stays open is not harmless. evaluate_bench() refuses while one
// is open -- deliberately, since a bench walk would re-address the chain the attempt is about -- so
// an abandoned attempt silently disables diagnostics until someone starts another proof. Every path
// that gives up after begin_proof() has to say so.
//
// It does NOT restore anything. begin_proof() revoked the mapping and the chain has since been
// walked; only a completed proof can make it PROVEN again.
bool abort_proof(const pf::challenge &c);

// A mapping that was proven and is not any more. Publishes LOST while keeping the epoch, so
// the consumer can still correlate the measurements it already accepted.
void note_mapping_lost();

// A fault that prevents any trustworthy mapping. `failing_position` is 1-6 or 0xFF.
void note_chain_fault(uint8_t chain_flags, uint8_t failing_position);

snapshot current();

// For tof_acq::config::mapping_state_provider. Reports what this authority believes; the
// acquisition layer's clamp is what decides whether anything acts on it.
tof_acq::mapping_state state_provider();

// The chain a committed proof installed. Empty (positions == 0) while non-PROVEN. The masks
// the contract keys by source_id will be filled from this once the role table exists.
//
// NOT protected by the atomic word, unlike everything in snapshot(). Only the commissioning
// context may read it. The health path must read snapshot() and nothing else -- that is what
// keeps the heartbeat lock-free while a proof runs.
const pf::fingerprint &installed_mapping();

// Diagnostics only. Never an authorisation path.
uint32_t attempt_nonce();
uint32_t epochs_used();

#ifdef CONFIG_ZTEST
// Clears the used-epoch bitmap.
//
// Test-only, and behind CONFIG_ZTEST so it cannot exist in a firmware image. It is here
// because the bitmap deliberately SURVIVES init(): "no epoch reused since power-on" has to
// mean the power cycle, not the last configuration, or a re-init would hand the whole space
// back. Tests need each case to start empty, and the honest way to give them that is an
// explicit test-only door rather than weakening what init() means in production.
void reset_epoch_history_for_test();
#endif

} // namespace lexxhard::tof_authority

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
