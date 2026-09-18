/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_auto_commission.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_auto_commission {

namespace {

config cfg_{};
hooks hooks_{};
state state_{state::disabled};
uint8_t attempts_{0};

bool wired()
{
    return hooks_.enumeration_permitted != nullptr && hooks_.acquire_epoch != nullptr &&
           hooks_.prove != nullptr && hooks_.start != nullptr;
}

} // namespace

void init(const config &cfg, const hooks &h)
{
    cfg_ = cfg;
    hooks_ = h;
    attempts_ = 0;
    /* A machine that is off, or not fully wired, reports disabled rather than waiting. The
     * difference matters to anything asking why nothing is happening: "switched off" and "on and
     * unable to act" are different answers and only one of them is a fault. */
    state_ = (cfg_.enabled && wired()) ? state::waiting : state::disabled;
}

step_result step()
{
    if (state_ == state::disabled)
        return step_result::disabled;
    if (state_ == state::started)
        return step_result::already_started;
    if (state_ == state::exhausted)
        return step_result::attempts_exhausted;

    /* Already proven, only the start remains. Deliberately not re-proved: a proof that succeeded
     * installed a mapping and spent an epoch, and re-proving because acquisition refused to start
     * would spend another for a failure that has nothing to do with the mapping. */
    if (state_ == state::proven) {
        if (hooks_.start(hooks_.ctx) != 0)
            return step_result::start_failed;
        state_ = state::started;
        return step_result::started;
    }

    /* The budget is checked BEFORE anything is reached for, not after a proof has already failed.
     * An earlier version checked it only on the failure path, so max_attempts = 0 -- a
     * configuration that switches the machine on without saying how many attempts it may make --
     * still made one. Zero means zero, for the same reason `enabled` defaults to false: the
     * permissive reading of an unstated limit is the one nobody chose. */
    if (attempts_ >= cfg_.max_attempts) {
        state_ = state::exhausted;
        return step_result::attempts_exhausted;
    }

    /* Asked, not inferred, and asked before the epoch: a machine that may not re-enumerate must not
     * reach for an epoch, because acquiring one may spend it. */
    if (!hooks_.enumeration_permitted(hooks_.ctx))
        return step_result::not_permitted;

    uint32_t epoch{0};
    if (hooks_.acquire_epoch(hooks_.ctx, &epoch) != 0) {
        /* Not a failed attempt. There is nothing to attempt WITH, and counting it would spend the
         * retry budget on a condition no retry can change. */
        return step_result::no_epoch;
    }

    ++attempts_;

    if (hooks_.prove(hooks_.ctx, epoch) != 0) {
        /* Nothing was installed, so nothing may start. Exhaustion is terminal on purpose: a
         * sequence that kept trying would re-enumerate the chain indefinitely on a machine whose
         * problem is not going to resolve itself, and each attempt takes the chain away from the
         * operator trying to look at it. */
        if (attempts_ >= cfg_.max_attempts) {
            state_ = state::exhausted;
            return step_result::attempts_exhausted;
        }
        return step_result::proof_failed;
    }

    /* The only path to start(), and it is reached only from a prove() that returned success. */
    state_ = state::proven;
    if (hooks_.start(hooks_.ctx) != 0)
        return step_result::start_failed;

    state_ = state::started;
    return step_result::started;
}

state current()
{
    return state_;
}

uint8_t attempts_used()
{
    return attempts_;
}

} // namespace lexxhard::tof_auto_commission

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
