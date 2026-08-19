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
    if (cfg.chain == nullptr || cfg.ops == nullptr || cfg.spec == nullptr ||
        cfg.quiesce == nullptr || cfg.spec->positions == 0)
        return -EINVAL;
    cfg_ = cfg;
    ready_ = true;
    return 0;
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

    /* STEP 4: the transaction, all of it inside the session. */
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

    const au::commit_refusal cr{
        au::commit_proof(static_cast<pf::proof_token &&>(v.token), static_cast<uint8_t>(host_epoch))};
    if (cr != au::commit_refusal::none) {
        r.failed_at = stage::commit_refused;
        r.commit = cr;
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
