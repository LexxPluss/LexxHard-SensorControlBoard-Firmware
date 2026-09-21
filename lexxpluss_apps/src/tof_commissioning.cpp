/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commissioning.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

#include "tof_tail_isolation.hpp"

namespace lexxhard::tof_commissioning {

namespace {

namespace enm = tof_enum;
namespace iso = tof_isolate;

config cfg_{};
bool ready_{false};

/* Holds the chain for the whole transaction and releases it on every exit path.
 *
 * RAII rather than an unlock at each return: this function has eight of them, and the one that
 * forgets is the one that parks the chain for the rest of the boot -- after which every later
 * commissioning run reports a busy chain and acquisition can never take it either. */
struct session {
    k_mutex *lock{nullptr};

    explicit session(k_mutex *m)
    {
        /* K_NO_WAIT, deliberately. The alternative parks the caller -- a shell thread, in
         * practice -- behind whatever holds the chain, which may be an acquisition cycle that is
         * itself stuck on a sensor. A commissioning command that never returns is worse than one
         * that says the chain is busy. */
        if (m != nullptr && k_mutex_lock(m, K_NO_WAIT) == 0)
            lock = m;
    }

    ~session()
    {
        if (lock != nullptr)
            k_mutex_unlock(lock);
    }

    bool held() const { return lock != nullptr; }

    session(const session &) = delete;
    session &operator=(const session &) = delete;
};

} // namespace

int init(const config &cfg)
{
    /* set_bus_speed is required, with no fallback to "leave the bus alone". A proof that silently
     * ran at whatever the bus happened to be set to would be the exact failure the transaction
     * exists to prevent, and it would report success. */
    if (cfg.chain == nullptr || cfg.ops == nullptr || cfg.spec == nullptr ||
        cfg.quiesce == nullptr || cfg.set_bus_speed == nullptr || cfg.spec->positions == 0)
        return -EINVAL;
    cfg_ = cfg;
    ready_ = true;
    return 0;
}

#ifdef CONFIG_ZTEST
void reset_for_test()
{
    cfg_ = config{};
    ready_ = false;
}
#endif

/* Puts the bus back to the speed a walk needs, after a failure past the point where it was raised.
 *
 * Best effort, and recorded rather than escalated: there is nothing useful to do about a bus that
 * will not retime in either direction, and turning it into a different failure would bury the one
 * that actually happened. What matters is that the outcome says so, and that the next proof sets
 * the speed itself rather than trusting this. */
void restore_proof_speed(outcome &r)
{
    const int rc{cfg_.set_bus_speed(bus_speed::proof_100k)};

    r.restore_attempted = true;
    r.restore_rc = rc;
    r.final_bus = rc == 0 ? bus_state::proof_100k : bus_state::unknown;
}

/* NON-DISTURBING re-verification at the product speed: every position must still ACK at the address
 * the enumeration assigned it, and still read back an identity appropriate to the model the spec
 * says sits there.
 *
 * "Non-disturbing", not "read-only", and the distinction is load-bearing: reading an L7's id means
 * selecting register page 0, reading, and putting page 2 back, so this does write. What it does not
 * do is readdress anything, move an enable line, or start a device ranging -- and those are the
 * three that would change the chain this step exists to inspect. A full open would do all of them,
 * leaving six sensors in a state nobody asked for on a path whose whole purpose is deciding whether
 * to proceed at all.
 *
 * The page write is only safe because read_id() puts the page back unconditionally, including when
 * the read itself fails. It did not always: a failed read used to leave the device parked on page 0,
 * where the next caller silently gets different registers than it asked for -- which on this path
 * would have meant a 400 kHz failure poisoning the following proof.
 *
 * WHAT THIS DOES AND DOES NOT ESTABLISH, stated because the difference is easy to lose: it shows
 * that the chain proven at 100 kHz still answers as itself at 400 kHz. It is not a second proof.
 * It cannot distinguish two devices of the same model that have swapped addresses -- that is what
 * the two walks and the tail isolation are for, and they have already run. */
bool recheck_identities(outcome &r)
{
    for (size_t i{0}; i < cfg_.spec->positions; ++i) {
        const enm::position_spec &ps{cfg_.spec->at[i]};

        r.recheck = identity_recheck{};
        r.recheck.position = static_cast<uint8_t>(i + 1);
        r.recheck.address = ps.target_addr;

        /* Only a clean ACK counts. A transport error is not an answer, and treating it as one would
         * let a bus that has stopped working at 400 kHz read as a chain that is present. */
        if (const enm::probe_result pr{cfg_.ops->probe(ps.target_addr)};
            pr.state != enm::probe_state::ack) {
            r.recheck.silent = true;
            return false;
        }

        enm::id_bytes seen{};
        if (const int rc{cfg_.ops->read_id(ps.expected, ps.target_addr, seen)}; rc != 0) {
            r.recheck.read_rc = rc;
            return false;
        }

        r.recheck.seen = seen;
        if (!enm::id_matches(ps.expected, seen))
            return false;
    }

    r.recheck = identity_recheck{};
    return true;
}

outcome prove(uint32_t host_epoch)
{
    outcome r{};

    if (!ready_) {
        r.failed_at = stage::not_configured;
        return r;
    }

    /* Before anything is touched. The epoch is a uint8 on the wire, and silently truncating 256 to 0
     * would install an epoch the host never issued and never recorded -- while telling the operator
     * the run succeeded. */
    if (host_epoch > 0xFF) {
        r.failed_at = stage::epoch_out_of_range;
        return r;
    }

    /* STEP 1: stop acquisition and wait. Not teardown(): the heartbeat must keep running for the
     * whole run, because it is the only channel telling a consumer that the subsystem is alive and
     * that its mapping is being re-proven. */
    if (const int rc{cfg_.quiesce()}; rc != 0) {
        r.failed_at = stage::quiesce_failed;
        r.rc = rc;
        return r;
    }

    /* STEP 2: take the chain BEFORE opening the attempt. Backwards, a busy chain would leave the
     * mapping revoked and an attempt open for a proof that never took a step. */
    const session held{cfg_.chain};
    if (!held.held()) {
        r.failed_at = stage::chain_busy;
        return r;
    }

    /* STEP 3: open the attempt while holding the chain. begin_proof() checks that acquisition is
     * idle, and that check is only worth something if nothing can start acquisition between it and
     * the first walk. Its is_idle() takes this same mutex recursively, which Zephyr permits for the
     * owning thread. */
    const au::attempt attempt{au::begin_proof()};
    if (!attempt.opened()) {
        r.failed_at = stage::attempt_refused;
        r.begin = attempt.reason;
        return r;
    }

    /* STEP 4: SET the proof speed. Set, not confirm and not assume.
     *
     * The bus can be at 400 kHz when this runs for reasons that have nothing to do with a previous
     * proof succeeding: an earlier run may have switched to the product speed, failed the identity
     * re-check, and failed again trying to put it back. Depending on that restore would make every
     * later proof inherit one earlier failure, and the walks would run at a speed nobody
     * characterised them at -- producing a mapping whose evidence means nothing while reporting
     * success. So the entry owns the speed unconditionally. */
    if (const int rc{cfg_.set_bus_speed(bus_speed::proof_100k)}; rc != 0) {
        r.failed_at = stage::proof_speed_refused;
        r.speed_rc = rc;
        /* Deliberately NOT retried here, and left `unknown`. The call that just failed is the one
         * that would be repeated, and a driver that refused the speed once is not made trustworthy
         * by asking twice -- it would only turn one honest "nobody knows" into a second guess. */
        (void)au::abort_proof(attempt.challenge);
        return r;
    }
    r.final_bus = bus_state::proof_100k;

    /* STEP 5: the transaction, all of it inside the session. */
    r.walk1 = enm::enumerate(*cfg_.ops, *cfg_.spec);

    /* The isolation's precondition is a chain where every position is enabled and addressed, which
     * only a `complete` walk provides. Running it on a frozen chain would produce an observation of
     * something else entirely -- and, unlike the case below, nothing has been darkened yet, so there
     * is nothing to compensate for. The evaluator refuses on walk1 and the operator gets the walk's
     * own diagnosis. */
    if (r.walk1.status == enm::chain_status::complete) {
        r.isolation_rc = iso::observe_tail(*cfg_.ops, *cfg_.spec, r.isolation);

        /* Walk 2 runs WHATEVER the isolation reported, including a control failure. The isolation
         * has darkened positions 1..N-1 and their addresses are gone with them; returning here would
         * leave the chain in that state, and nothing but a fresh enumeration recovers it. So the
         * recovery is not optional and not the operator's job. */
        r.walk2 = enm::enumerate(*cfg_.ops, *cfg_.spec);
    }

    pf::evidence ev{};
    ev.spec = cfg_.spec;
    ev.walk1 = &r.walk1;
    ev.walk2 = &r.walk2;
    ev.isolation = r.isolation;

    pf::verdict v{au::evaluate(ev, attempt.challenge)};
    if (!v.granted()) {
        r.failed_at = stage::evidence_refused;
        r.proof = v.reason;
        /* Say so, rather than walking away. An attempt left open blocks bench diagnostics until
         * somebody starts another proof, and "the operator gave up" is exactly when a diagnostic is
         * wanted. */
        (void)au::abort_proof(attempt.challenge);
        return r;
    }

    /* STEP 6: THE SPEED THE CHAIN WILL ACTUALLY BE READ AT, before anything is published.
     *
     * The proof establishes what the chain is, at 100 kHz. Nothing in it says the same chain still
     * answers at 400 kHz, which is the only speed the acquisition schedule fits in -- and a mapping
     * published on evidence gathered at a speed the product never uses is a mapping proven for a
     * machine that does not exist.
     *
     * It happens HERE, before commit_proof(), and that ordering is the whole design. Committing
     * first and re-checking afterwards would publish PROVEN and then withdraw it, and the health
     * timer runs on its own cadence: a consumer can sample that window and act on a PROVEN it was
     * never meant to see. There must be no such window, so the authority is never told anything
     * until the chain has answered at the speed it will be read at. */
    if (const int rc{cfg_.set_bus_speed(bus_speed::product_400k)}; rc != 0) {
        r.failed_at = stage::product_speed_refused;
        r.speed_rc = rc;
        r.final_bus = bus_state::unknown;
        restore_proof_speed(r);
        (void)au::abort_proof(attempt.challenge);
        return r;
    }

    r.final_bus = bus_state::product_400k;

    if (!recheck_identities(r)) {
        r.failed_at = stage::identity_recheck_failed;
        restore_proof_speed(r);
        (void)au::abort_proof(attempt.challenge);
        return r;
    }

    /* STEP 7: only now. commit_proof() is what writes the descriptor roles, consumes the epoch and
     * publishes PROVEN -- one transaction, so there is no state in which some corners are keyed and
     * the rest are not. Nothing above it wrote a role. */
    const au::commit_refusal cr{
        au::commit_proof(static_cast<pf::proof_token &&>(v.token), static_cast<uint8_t>(host_epoch))};
    if (cr != au::commit_refusal::none) {
        r.failed_at = stage::commit_refused;
        r.commit = cr;
        /* The bus is at 400 kHz and nothing was installed, so this leaves exactly the state every
         * other failure path leaves: a chain at the product speed with no proven mapping. It used
         * to walk away from here without restoring, which is the one failure that ends at 400 kHz
         * with an operator told nothing about it -- and a reused epoch is the ordinary way to reach
         * it. Best effort, and it does not become a different failure: the commit refusal is what
         * happened and is what is returned. */
        restore_proof_speed(r);
        /* No abort here: a token that matched the attempt spends it inside commit_proof(), whatever
         * the outcome. Calling abort would be harmless -- it is nonce-bound and the attempt is
         * already closed -- but it would suggest the attempt were still open, which it is not. */
        return r;
    }

    r.failed_at = stage::none;
    return r;
}

} // namespace lexxhard::tof_commissioning

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
