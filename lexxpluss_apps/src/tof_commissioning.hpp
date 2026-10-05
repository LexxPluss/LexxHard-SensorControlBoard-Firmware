/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/*
 * The commissioning proof, orchestrated: one call that carries out the contract's whole
 * transaction and either installs a proven mapping or leaves nothing changed.
 *
 * It owns no policy. The proof's rules live in tof_mapping_proof, the mapping and the epoch live in
 * tof_mapping_authority, the walk lives in tof_enumerator and the isolation in tof_tail_isolation.
 * What lives here is the ORDER, because the order is where the mistakes are.
 *
 * THE ORDER, AND WHY EACH STEP IS WHERE IT IS
 *
 *   1. quiesce acquisition and wait for it to stop -- but NOT teardown()
 *   2. take the chain lock with K_NO_WAIT
 *   3. begin the attempt WHILE HOLDING the lock
 *   4. walk 1 -> isolation -> walk 2, then evaluate and commit, all in the same session
 *   5. release, and only then may acquisition be restarted
 *
 * Step 1 is stop(), never teardown(). They differ in exactly the thing that matters here: teardown()
 * stops the health timer, cancels the pending work and clears the configuration, and commissioning
 * is precisely when a consumer needs the heartbeat most -- it is the only channel saying "the
 * subsystem is alive, the mapping is being re-proven, do not trust position data". A commissioning
 * run that silenced health would be indistinguishable from a crashed producer for its whole
 * duration.
 *
 * Step 2 before step 3, which is the part that is easy to get backwards. If the attempt were begun
 * first and the lock then turned out to be busy, the mapping would already have been revoked and an
 * attempt left open, for a proof that never took a single step. Taking the lock first means a busy
 * chain costs nothing at all.
 *
 * Step 3 while holding the lock closes the other window: begin_proof() checks that acquisition is
 * idle, and between an unlocked check and a later lock acquisition acquisition could start again.
 * The check is only worth anything if nothing can intervene between it and the walk. This relies on
 * Zephyr's k_mutex being RECURSIVE for its owner (kernel/mutex.c: `owner == _current` increments
 * lock_count), because begin_proof() -> is_idle() and commit_proof() -> begin_epoch() both take the
 * same mutex from inside the session. That is a documented property of the API, not a coincidence,
 * and a test holds the session while driving both calls.
 *
 * Step 4 keeps evaluation and installation inside the session. Releasing between walk 2 and the
 * commit would open a window for anything else to re-address the chain the proof just described.
 *
 * WALK 2 RUNS EVEN WHEN THE ISOLATION FAILED
 *
 * The isolation darkens positions 1..N-1, and on the L4 carriers that returns them to the default
 * address. Returning early after a failed observation would leave the chain in that half-addressed
 * state, which nothing else recovers -- a fresh enumeration IS the recovery. So once the isolation
 * has been started, walk 2 is attempted whatever it reported, and the evaluator refuses on the
 * isolation's own evidence rather than on a missing walk.
 *
 * WHAT IT CANNOT DO YET
 *
 * It does not start acquisition after a successful proof; there is no acquisition thread yet. And
 * on the product spec as it stands it cannot succeed at all: the four cliff roles are still
 * `unknown`, so the evaluator refuses by rule. Both are true by design at this point, and the
 * PROVEN clamp is untouched either way.
 */

#include <stdint.h>

#include <zephyr/kernel.h>

#include "tof_enumerator.hpp"
#include "tof_mapping_authority.hpp"
#include "tof_mapping_proof.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_commissioning {

namespace au = tof_authority;
namespace pf = tof_proof;

// Where a run stopped. One value per step, because "commissioning failed" tells an operator
// standing at the machine nothing: a busy chain, a frozen walk and a missing role table need three
// different actions.
enum class stage : uint8_t {
    none = 0,         // it succeeded
    not_configured,   // no chain ops, spec or lock was injected
    epoch_out_of_range, // the host offered something outside 0-255
    quiesce_failed,   // acquisition would not stop
    chain_busy,       // the lock was held: another enumeration, or acquisition still on the chain
    attempt_refused,  // the authority would not open an attempt
    evidence_refused, // the transaction was carried out and the proof refused it
    /* The bus would not go to 100 kHz for the walks. Refused rather than proceeding at whatever the
     * bus happened to be left at: a walk at an uncharacterised speed produces a mapping whose
     * evidence means nothing, while reporting success. */
    proof_speed_refused,
    // The proof held, but the bus would not go to 400 kHz afterwards.
    product_speed_refused,
    // The proof held and the bus retimed, but a position did not answer as itself at 400 kHz.
    identity_recheck_failed,
    commit_refused,   // the proof held and the authority refused to install it
};

/* The two speeds this transaction uses, and there are exactly two. 100 kHz is the only speed the
 * commissioning walk has ever completed at; 400 kHz is the only speed the acquisition schedule fits
 * in. Naming them rather than passing a frequency keeps the pair a decision somebody made once. */
enum class bus_speed : uint8_t {
    proof_100k,
    product_400k,
};

enum class bus_state : uint8_t {
    // A set_bus_speed call failed and nothing is known about what the driver left behind.
    unknown,
    proof_100k,
    product_400k,
};

/* Which position disagreed, and how. Populated only for identity_recheck_failed. */
struct identity_recheck {
    // 1-based, as the operator counts them. 0 when nothing failed.
    uint8_t position{0};
    uint8_t address{0};
    /* The probe did not cleanly ACK. Distinguished from an identity mismatch because the two send an
     * operator to different places: silence is a bus or a part, a wrong id is the WRONG part. The
     * wire carries the distinction too -- position_silent against identity_disagreed. */
    bool silent{false};
    int read_rc{0};
    tof_enum::id_bytes seen{};
};

struct outcome {
    stage failed_at{stage::not_configured};
    // Populated for the stage that failed, and left at its neutral value otherwise. Kept as three
    // separate fields rather than one merged code: they come from three components with three
    // vocabularies, and flattening them would need a translation table that could only lose detail.
    int rc{0};                                                       // quiesce errno
    au::begin_refusal begin{au::begin_refusal::none};
    pf::refusal proof{pf::refusal::none};
    au::commit_refusal commit{au::commit_refusal::none};

    // The transaction's own results, for the operator's transcript. A failed run is exactly when
    // these are worth printing.
    tof_enum::chain_result walk1{};
    tof_enum::chain_result walk2{};
    pf::isolation_observation isolation{};
    int isolation_rc{0};

    /* What either set_bus_speed() call returned, for proof_speed_refused and
     * product_speed_refused. */
    int speed_rc{0};
    /* WHERE THE BUS WAS LEFT, which matters most on the paths that failed. `unknown` is a real
     * answer and not a missing one: a driver that refused a speed change may have left the
     * peripheral anywhere, and claiming otherwise would send the next proof -- or an operator --
     * off a guess. */
    bus_state final_bus{bus_state::unknown};
    /* Whether a failure after the retime tried to put the bus back, and what that attempt said.
     * Best effort by design: the restore does not change WHICH failure is reported, so a proof that
     * failed its identity re-check and then failed to restore still reports the re-check. */
    bool restore_attempted{false};
    int restore_rc{0};
    // Populated only for identity_recheck_failed.
    identity_recheck recheck{};

    bool proven() const { return failed_at == stage::none; }
};

struct config {
    // The chain control lines and the bus as one resource. Production passes
    // &tof_chain_controller::chain_lock(); the tests pass a real k_mutex, because "recursive locking
    // does not deadlock" is not a property a fake can demonstrate.
    k_mutex *chain{nullptr};
    tof_enum::chain_ops *ops{nullptr};
    // The chain being proven. The authority holds its own pointer to the RUNTIME spec and compares
    // the two, so passing a different one here cannot install itself.
    const tof_enum::chain_spec *spec{nullptr};
    // Stops acquisition and does not return until it has stopped. Production: stop the thread and
    // join it. Must NOT tear the subsystem down -- the heartbeat has to keep running.
    int (*quiesce)(){nullptr};
    /* Sets the I2C bitrate and does not return until it has. Production drives the chain
     * controller's runtime reconfigure; the tests inject one so the ORDER of the two speed changes
     * against the walks is observable, which is the whole point of the dual-rate transaction. */
    int (*set_bus_speed)(bus_speed speed){nullptr};
};

int init(const config &cfg);

// Carries out the whole transaction. `host_epoch` is mandatory and has no default: under the
// commissioning profile the host owns the epoch, and a firmware-invented one would be a value no
// operator recorded. Taken as uint32_t so that a value outside 0-255 is REFUSED rather than
// truncated into a different epoch.
outcome prove(uint32_t host_epoch);

}  // namespace lexxhard::tof_commissioning

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
