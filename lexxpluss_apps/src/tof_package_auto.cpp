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

const char *report_name(report r)
{
    switch (r) {
    case report::publish_grid:
        return "publish_grid";
    case report::publish_diagnostic:
        return "publish_diagnostic";
    case report::say_nothing:
        return "say_nothing";
    }
    return "unnamed";
}

report what_to_send(const pass &p, const cadence &c)
{
    /* A GRID IS NEVER RATE LIMITED. Its rate is the sensor's, and holding one back to satisfy a
     * diagnostic cadence would drop a measurement to make room for a message about measurements. */
    if (p.read_rc == 0 && p.fresh)
        return report::publish_grid;

    /* A poll before the next grid is due is not a publishable event. */
    if (p.read_rc == 0 && p.now_ms - p.last_grid_ms < c.grid_overdue_ms)
        return report::say_nothing;

    /* What is left is a failed read, or a silence long enough to be a stall. Both persist, so both
     * go out on the cadence rather than on every pass. */
    if (p.now_ms - p.last_diagnostic_ms < c.diagnostic_interval_ms)
        return report::say_nothing;
    return report::publish_diagnostic;
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
        /* Save the failed step before rebind clears it, even if takeover is unavailable. */
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

    /* Only a failed quiesce authorizes takeover. quiesce_first=false is an explicit
     * exception for a board with a previously observed stop failure. */
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
