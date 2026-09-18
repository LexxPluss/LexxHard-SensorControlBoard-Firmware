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
uint8_t start_attempts_{0};

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
    start_attempts_ = 0;
    /* Three outcomes, not two. Off is a choice; on-and-unwired is a fault and is reported as one,
     * because anything asking why nothing is happening needs to tell them apart -- and because a
     * missing hook will not resolve itself, so reporting it as "disabled" would describe a broken
     * deployment as a deliberate configuration. */
    if (!cfg_.enabled)
        state_ = state::disabled;
    else if (!wired())
        state_ = state::misconfigured;
    else
        state_ = state::waiting;
}

step_result step()
{
    if (state_ == state::disabled)
        return step_result::disabled;
    if (state_ == state::misconfigured)
        return step_result::misconfigured;
    if (state_ == state::started)
        return step_result::already_started;
    if (state_ == state::exhausted)
        return step_result::attempts_exhausted;

    /* Already proven, only the start remains. Deliberately not re-proved: a proof that succeeded
     * installed a mapping and spent an epoch, and re-proving because acquisition refused to start
     * would spend another for a failure that has nothing to do with the mapping. */
    if (state_ == state::proven) {
        if (start_attempts_ >= cfg_.max_start_attempts)
            return step_result::start_attempts_exhausted;
        ++start_attempts_;
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

    /* The only path to start(), and it is reached only from a prove() that returned success. The
     * state moves to proven BEFORE the start is tried, so a start that refuses is retried as a start
     * and never re-proved: the mapping is installed and its epoch is spent, and spending another on
     * a failure that has nothing to do with the mapping would be waste with a cost. */
    state_ = state::proven;
    if (start_attempts_ >= cfg_.max_start_attempts)
        return step_result::start_attempts_exhausted;
    ++start_attempts_;
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

uint8_t start_attempts_used()
{
    return start_attempts_;
}

} // namespace lexxhard::tof_auto_commission

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
