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

/*
 * THE UNATTENDED RUNTIME'S SEQUENCE, with no device, no bus and no clock.
 *
 * Every case here is a question about ORDER and BUDGET, which is the whole of what this policy
 * decides: when the stop defence is attempted, when it may be skipped, how many times the device
 * may be claimed before the runtime stops touching it, and what it does afterwards. None of them
 * needs hardware, and none of them would be answerable from a board: on a board the answer arrives
 * as a sensor that did or did not come up, with the reasoning invisible.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "tof_package_auto.hpp"

namespace pkg = lexxhard::tof_package;
namespace sut = lexxhard::tof_package_auto;

namespace {

sut::observation unbound()
{
    sut::observation o{};
    o.bound = false;
    o.current = pkg::state::idle;
    return o;
}

sut::observation idle()
{
    sut::observation o{};
    o.bound = true;
    o.current = pkg::state::idle;
    return o;
}

sut::observation ranging()
{
    sut::observation o{};
    o.bound = true;
    o.current = pkg::state::ranging;
    return o;
}

sut::observation blocked_at(pkg::step where, bool discardable)
{
    sut::observation o{};
    o.bound = true;
    o.current = pkg::state::blocked;
    o.failed_step = where;
    o.discardable = discardable;
    return o;
}

/* The whole sequence a board follows when the quiesce cannot be answered: the policy attempts it,
 * is refused, discards the harmless blocked state, and takes over once. Written as one case
 * because the rules only mean anything in that order. */
sut::policy configured(const sut::limits &l)
{
    sut::policy p;
    zassert_equal(0, p.init(l));
    return p;
}

}  // namespace

ZTEST_SUITE(tof_package_auto, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_package_auto, test_a_policy_that_was_never_configured_touches_nothing)
{
    /* No budget exists, so there is none to spend. Reporting is the only thing left that cannot
     * damage a sensor. */
    sut::policy p;
    zassert_equal(sut::action::report_only, p.next(idle()));
    zassert_equal(sut::action::report_only, p.next(ranging()));
    zassert_equal(0, p.current().claims_used);
}

ZTEST(tof_package_auto, test_a_zero_claim_budget_and_more_takeovers_than_claims_are_refused)
{
    sut::policy p;
    sut::limits none{};
    none.claims = 0;
    zassert_equal(-EINVAL, p.init(none));

    sut::limits more{};
    more.claims = 2;
    more.takeovers = 3;
    zassert_equal(-EINVAL, p.init(more));

    sut::limits ok{};
    ok.claims = 2;
    ok.takeovers = 2;
    zassert_equal(0, p.init(ok));
}

ZTEST(tof_package_auto, test_an_unbound_lifecycle_is_bound_before_anything_is_claimed)
{
    sut::policy p{configured(sut::limits{})};
    zassert_equal(sut::action::bind, p.next(unbound()));
    /* Binding opens nothing, so it is not a claim and costs nothing. */
    zassert_equal(0, p.current().claims_used);
}

ZTEST(tof_package_auto, test_the_first_claim_attempts_the_quiesce)
{
    sut::policy p{configured(sut::limits{})};
    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(1, p.current().claims_used);
    zassert_equal(0, p.current().takeovers_used);
}

ZTEST(tof_package_auto, test_a_ranging_device_is_read_and_never_reclaimed)
{
    sut::policy p{configured(sut::limits{})};
    for (int i = 0; i < 100; ++i)
        zassert_equal(sut::action::acquire, p.next(ranging()));
    /* A working stream spends no budget: an earlier failure must not be able to stop it later. */
    zassert_equal(0, p.current().claims_used);
}

ZTEST(tof_package_auto, test_a_stop_debt_is_paid_before_anything_else_and_costs_no_budget)
{
    sut::policy p{configured(sut::limits{})};
    sut::observation owed{};
    owed.bound = true;
    owed.current = pkg::state::stop_owed;
    zassert_equal(sut::action::pay_stop, p.next(owed));
    zassert_equal(0, p.current().claims_used);

    /* Even unbound, and even with the budget gone: the lifecycle bounds its own stop attempts, and
     * a second bound here would decide when to abandon a session that may still be measuring. */
    owed.bound = false;
    zassert_equal(sut::action::pay_stop, p.next(owed));
}

ZTEST(tof_package_auto, test_a_refused_quiesce_is_discarded_and_then_taken_over_once)
{
    sut::policy p{configured(sut::limits{})};

    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    /* bring_up() left it blocked at the survivor step, which discardable() allows: nothing was
     * opened, so no measurement can be behind it. */
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::survivor, true)));
    zassert_true(p.current().quiesce_refused);
    /* The rebind cleared the step, and the policy remembered what it was for. */
    zassert_equal(sut::action::claim_by_takeover, p.next(idle()));
    zassert_equal(1, p.current().takeovers_used);
    zassert_equal(2, p.current().claims_used);
}

ZTEST(tof_package_auto, test_the_takeover_budget_is_spent_whether_or_not_it_worked)
{
    sut::limits l{};
    l.claims = 6;
    l.takeovers = 1;
    sut::policy p{configured(l)};

    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::survivor, true)));
    zassert_equal(sut::action::claim_by_takeover, p.next(idle()));
    /* The takeover failed at open this time, which is discardable. */
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::open, true)));
    /* The quiesce has already been refused once, so repeating the claim that failed would spend
     * the remaining claims on the same refusal -- and no takeover is left. */
    zassert_equal(sut::action::report_only, p.next(idle()));
    zassert_equal(1, p.current().takeovers_used);
}

ZTEST(tof_package_auto, test_a_failure_that_is_not_the_quiesce_does_not_authorise_a_takeover)
{
    sut::policy p{configured(sut::limits{})};

    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    /* The probe did not complete: that says nothing about whether the device can be stopped, so
     * removing the stop defence would be skipping a defence to fix an unrelated fault. */
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::probe, true)));
    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(0, p.current().takeovers_used);
    zassert_false(p.current().quiesce_refused);
}

ZTEST(tof_package_auto, test_a_blocked_state_a_session_could_be_behind_is_never_discarded)
{
    sut::policy p{configured(sut::limits{})};
    /* step::stop with a spent budget: a device that may still be measuring. The lifecycle says it
     * is not discardable, and the policy takes that answer rather than deriving its own. */
    zassert_equal(sut::action::report_only, p.next(blocked_at(pkg::step::stop, false)));
    zassert_equal(sut::action::report_only, p.next(blocked_at(pkg::step::close, false)));
    zassert_equal(0, p.current().claims_used);
}

ZTEST(tof_package_auto, test_a_quiesce_refusal_is_recorded_even_when_it_cannot_be_acted_on)
{
    sut::limits l{};
    l.claims = 6;
    l.takeovers = 0;
    sut::policy p{configured(l)};

    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::survivor, true)));
    zassert_true(p.current().quiesce_refused);
    /* No takeover is authorised at all, so the stop defence stays absolute and the device is
     * reported rather than taken over. */
    zassert_equal(sut::action::report_only, p.next(idle()));
    zassert_equal(0, p.current().takeovers_used);
}

ZTEST(tof_package_auto, test_quiesce_first_false_takes_over_on_the_very_first_claim)
{
    sut::limits l{};
    l.claims = 6;
    l.takeovers = 1;
    l.quiesce_first = false;
    sut::policy p{configured(l)};

    zassert_equal(sut::action::claim_by_takeover, p.next(idle()));
    zassert_equal(1, p.current().takeovers_used);
}

ZTEST(tof_package_auto, test_quiesce_first_false_applies_to_the_first_claim_only)
{
    sut::limits l{};
    l.claims = 6;
    l.takeovers = 2;
    l.quiesce_first = false;
    sut::policy p{configured(l)};

    zassert_equal(sut::action::claim_by_takeover, p.next(idle()));
    /* The takeover failed at open, which says nothing about the quiesce. The next claim therefore
     * goes back to attempting it: the configuration authorised skipping the FIRST one, not every
     * one. */
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::open, true)));
    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(1, p.current().takeovers_used);
}

ZTEST(tof_package_auto, test_the_claim_budget_ends_in_reporting_and_stays_there)
{
    sut::limits l{};
    l.claims = 2;
    l.takeovers = 0;
    sut::policy p{configured(l)};

    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::probe, true)));
    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    /* Budget gone. The rebind is refused too, because a rebind exists only to make another claim
     * possible and there is none left. */
    zassert_equal(sut::action::report_only, p.next(blocked_at(pkg::step::probe, true)));
    zassert_equal(sut::action::report_only, p.next(idle()));
    /* And it is a destination: a device that starts ranging on its own would still be read, but
     * nothing in this policy claims the device again. */
    for (int i = 0; i < 10; ++i)
        zassert_equal(sut::action::report_only, p.next(idle()));
    zassert_equal(2, p.current().claims_used);
}

ZTEST(tof_package_auto, test_reporting_never_hides_a_device_that_is_measuring)
{
    sut::limits l{};
    l.claims = 1;
    l.takeovers = 0;
    sut::policy p{configured(l)};

    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(sut::action::report_only, p.next(idle()));
    /* The claim did start it after all -- the start was slow rather than failed. Measuring is
     * decided before any budget is consulted, so the stream is read. */
    zassert_equal(sut::action::acquire, p.next(ranging()));
}

ZTEST(tof_package_auto, test_init_resets_the_budget_and_what_it_remembered)
{
    sut::policy p{configured(sut::limits{})};
    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
    zassert_equal(sut::action::rebind_to_retry, p.next(blocked_at(pkg::step::survivor, true)));
    zassert_true(p.current().quiesce_refused);

    zassert_equal(0, p.init(sut::limits{}));
    zassert_false(p.current().quiesce_refused);
    zassert_equal(0, p.current().claims_used);
    zassert_equal(sut::action::claim_with_quiesce, p.next(idle()));
}

ZTEST(tof_package_auto, test_every_action_has_a_name)
{
    const sut::action all[]{sut::action::bind,        sut::action::claim_with_quiesce,
                            sut::action::claim_by_takeover, sut::action::rebind_to_retry,
                            sut::action::acquire,     sut::action::pay_stop,
                            sut::action::report_only};
    for (const auto a : all)
        zassert_not_equal(0, strcmp(sut::action_name(a), "unnamed"), "%d", static_cast<int>(a));
}
