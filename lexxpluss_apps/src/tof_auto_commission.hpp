/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*
 * The unattended prove-then-start sequence: default off, bounded, and incapable of putting a
 * measurement frame on the bus by any path except a proof that actually succeeded.
 *
 * WHAT IT IS AND IS NOT. It sequences; it decides nothing about mappings. The proof itself is the
 * existing transaction -- `tof_commissioning::prove()`, which quiesces acquisition, walks the chain
 * at 100 kHz, isolates the tail, walks again, moves to 400 kHz and commits, all under the chain
 * lock. None of that is reimplemented here, and it must not be: a second copy of the proof would be
 * a second thing to keep in step with the contract, and the one that drifted would be the one
 * running unattended.
 *
 * DEFAULT OFF, IN THE TYPE. `config::enabled` is false when default-constructed, so a machine that
 * nobody configured does nothing and calls none of its hooks. Being switched on is an act, not an
 * absence.
 *
 * EVERY INPUT IS INJECTED, AND TWO OF THEM FOR REASONS THAT OUTLIVE TESTING:
 *
 *   enumeration_permitted  whether the machine may re-enumerate the chain right now. A trustworthy
 *                          stationary condition does not exist yet -- it is an open item against
 *                          safety, not something this module may infer from what it can see. So it
 *                          is asked, never derived, and a machine whose hook says no does nothing.
 *   acquire_epoch          where the epoch comes from. Which side issues it is undecided: the
 *                          firmware with a persistent store, or the host over a downlink. Hiding
 *                          that behind one hook is what lets this sequence be written and tested
 *                          while the route is still with release and safety, and it is why this
 *                          module does not call tof_epoch_issuer directly.
 *
 * THE INVARIANT THIS EXISTS TO HOLD, stated as narrowly as it is actually true: **this module calls
 * `start()` only after a `prove()` that returned success.** That is a property of this module's own
 * behaviour and nothing more.
 *
 * An earlier version of this comment claimed more -- that a failure here means no measurement frames
 * exist at all. It cannot claim that. Acquisition may already be running because an operator started
 * it, and this module neither knows nor changes that. What it guarantees is that it will not be the
 * thing that starts it: starting acquisition is the only step between a proven mapping and 0x216 on
 * the wire, which the shell's own `tof cliff start` says in as many words, so a failure here adds
 * nothing to the bus. Health is published by the runtime for its own reasons; this module fabricates
 * no health state and suppresses none.
 *
 * MUTUAL EXCLUSION WITH THE OPERATOR is not implemented here, because it already exists lower down:
 * the transaction takes the chain with K_NO_WAIT and refuses if an operator's command holds it. A
 * busy chain therefore arrives as an ordinary proof failure, is retried, and is bounded like any
 * other. Adding a second lock here would mean two things claiming to serialise the same chain.
 *
 * BOUNDED, NOT PERSISTENT. `max_attempts` is counted within one power-on. There is no unbounded
 * retry, and there is no memory of attempts across a reset -- a board that comes up again is a board
 * whose conditions may have changed, and pretending otherwise would be inventing the persistence
 * whose absence is the open decision.
 */

#include <stdint.h>

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_auto_commission {

struct hooks {
    /* May the chain be re-enumerated right now? Asked, never inferred. */
    bool (*enumeration_permitted)(void *ctx);
    /* 0 and an epoch, or negative for "none available". Negative is not a failure of the sequence:
     * no epoch means no attempt is made and none is counted. */
    int (*acquire_epoch)(void *ctx, uint32_t *out_epoch);
    /* The real transaction. 0 means a proven, keyed mapping; anything else means nothing was
     * installed. */
    int (*prove)(void *ctx, uint32_t epoch);
    /* Acquisition. 0 means the thread is running. */
    int (*start)(void *ctx);
    void *ctx;
};

struct config {
    bool enabled{false};
    /* Proof attempts within one power-on. Zero with `enabled` set is a configuration fault, not a
     * quiet nothing: it describes a machine that can never reach `started`. */
    uint8_t max_attempts{0};
    /* Acquisition starts, counted separately. A start that refuses is a different failure from a
     * proof that refuses -- the mapping is installed and the epoch is spent -- so it gets its own
     * budget rather than borrowing the proof's, and it is bounded for the same reason the proof is:
     * an unbounded retry keeps taking the chain from whoever is trying to look at the machine. */
    uint8_t max_start_attempts{0};
};

enum class state : uint8_t {
    disabled,      /* not configured, or configured off -- a choice, not a fault */
    misconfigured, /* switched on and unable to finish: a hook or a budget is missing. A fault */
    waiting,   /* on, and no mapping proven yet */
    proven,    /* a proof succeeded; acquisition not yet running */
    started,   /* acquisition running; terminal for this power-on */
    exhausted, /* attempts spent without a proof; terminal, and deliberately so */
};

enum class step_result : uint8_t {
    disabled,
    misconfigured,      /* enabled with a hook or a budget missing; nothing is attempted, and it will not resolve */
    not_permitted,      /* the stationary condition said no; nothing was attempted or counted */
    no_epoch,           /* no epoch available; nothing was attempted or counted */
    proof_failed,       /* an attempt was spent and nothing was installed */
    attempts_exhausted, /* the last attempt is spent; no further proof will be attempted */
    start_failed,       /* the mapping is proven and acquisition refused to start */
    start_attempts_exhausted, /* the start budget is spent; the mapping stays proven, unstarted */
    started,
    already_started,
};

void init(const config &cfg, const hooks &h);

/* One attempt at most. Returns what happened; the caller decides when to call again. */
step_result step();

state current();
uint8_t attempts_used();
uint8_t start_attempts_used();

} // namespace lexxhard::tof_auto_commission

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
