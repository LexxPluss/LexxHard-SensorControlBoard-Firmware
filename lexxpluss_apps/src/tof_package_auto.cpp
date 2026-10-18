/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>

#include "tof_package_auto.hpp"

#if defined(ENABLE_TOF_L7_ULD)

namespace lexxhard::tof_package_auto {

const char *action_name(action a)
{
    switch (a) {
    case action::bind:
        return "bind";
    case action::claim_with_quiesce:
        return "claim_with_quiesce";
    case action::claim_by_takeover:
        return "claim_by_takeover";
    case action::rebind_to_retry:
        return "rebind_to_retry";
    case action::acquire:
        return "acquire";
    case action::pay_stop:
        return "pay_stop";
    case action::report_only:
        return "report_only";
    }
    return "unnamed";
}

int policy::init(const limits &l)
{
    if (l.claims == 0U || l.takeovers > l.claims)
        return -EINVAL;
    limits_ = l;
    counters_ = counters{};
    initialised_ = true;
    return 0;
}

action policy::next(const observation &o)
{
    if (!initialised_)
        return counters_.last = action::report_only;

    /* MEASURING COMES FIRST, before any budget is consulted. A device that is ranging has already
     * been claimed, and a pass that re-examined the budget here would be able to stop a working
     * stream because an earlier claim had failed. */
    if (o.current == tof_package::state::ranging)
        return counters_.last = action::acquire;

    /* A DEBT IS PAID BEFORE ANYTHING ELSE IS ATTEMPTED, and it is not a claim: no budget is spent,
     * because the lifecycle's own stop_attempts bounds it and a second bound here would decide
     * when to abandon a session that may still be measuring. */
    if (o.current == tof_package::state::stop_owed)
        return counters_.last = action::pay_stop;

    if (!o.bound)
        return counters_.last = action::bind;

    if (o.current == tof_package::state::blocked) {
        /* WHICH BLOCKED STEP IT WAS IS REMEMBERED HERE, because a rebind resets it. The lifecycle
         * clears `failed_step` on its way back to idle, so by the time a takeover could be chosen
         * the reason for choosing it is gone. Recorded before the discardable check so that a
         * quiesce failure this policy cannot act on is still part of the record. */
        if (o.failed_step == tof_package::step::survivor)
            counters_.quiesce_refused = true;

        /* The lifecycle's answer, not a second copy of it: discardable() is false for exactly the
         * steps a live session can be behind, and discarding one of those loses the only record
         * that the device may still be measuring. */
        if (!o.discardable)
            return counters_.last = action::report_only;
        if (counters_.claims_used >= limits_.claims)
            return counters_.last = action::report_only;
        return counters_.last = action::rebind_to_retry;
    }

    /* idle: the only state a claim may start from. */
    if (counters_.claims_used >= limits_.claims)
        return counters_.last = action::report_only;

    /* A TAKEOVER IS FOR A QUIESCE THAT WAS REFUSED, not for a claim that failed somewhere else. A
     * claim that stopped at the probe or at open says nothing about whether the device can be
     * stopped, so repeating it with the stop defence removed would skip a defence to fix an
     * unrelated fault. The one exception is a configuration that has already seen this geometry
     * refuse the quiesce and said so: `quiesce_first` false authorises the first claim to skip it. */
    const bool skip_the_quiesce{counters_.quiesce_refused ||
                                (!limits_.quiesce_first && counters_.claims_used == 0U)};
    if (skip_the_quiesce && counters_.takeovers_used < limits_.takeovers) {
        ++counters_.claims_used;
        ++counters_.takeovers_used;
        return counters_.last = action::claim_by_takeover;
    }
    /* No takeover left, and the quiesce has already been refused once: repeating the attempt that
     * failed would spend the remaining claims on the same refusal. */
    if (skip_the_quiesce)
        return counters_.last = action::report_only;

    ++counters_.claims_used;
    return counters_.last = action::claim_with_quiesce;
}

}  // namespace lexxhard::tof_package_auto

#endif  // ENABLE_TOF_L7_ULD
