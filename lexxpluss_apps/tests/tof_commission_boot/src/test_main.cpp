/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The boot order of the cliff subsystem and the commissioning downlink.
 *
 * Three rules, and each of them is a thing that was previously only written down: the downlink
 * starts AFTER the cliff bootstrap has returned, it starts EVEN IF that bootstrap failed, and the
 * whole sequence runs ONCE.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "tof_commission_boot.hpp"

namespace boot = lexxhard::tof_commission_boot;

namespace {

struct recorder {
    int order[4]{};
    int steps{0};
    int cliff_rc{0};
    int downlink_state{0};

    void note(int what)
    {
        if (steps < 4)
            order[steps] = what;
        ++steps;
    }
};

constexpr int kCliff{1};
constexpr int kDownlink{2};

int bootstrap_cliff(void *ctx)
{
    recorder *r{static_cast<recorder *>(ctx)};

    r->note(kCliff);
    return r->cliff_rc;
}

int start_downlink(void *ctx)
{
    recorder *r{static_cast<recorder *>(ctx)};

    r->note(kDownlink);
    return r->downlink_state;
}

boot::steps both(recorder &r)
{
    boot::steps s{};

    s.bootstrap_cliff = bootstrap_cliff;
    s.start_downlink = start_downlink;
    s.ctx = &r;
    return s;
}

void before(void *)
{
    boot::reset_for_test();
}

} // namespace

ZTEST_SUITE(tof_commission_boot, NULL, NULL, before, NULL, NULL);

/* THE DOWNLINK IS LAST. Starting it installs a CAN receive filter and a worker whose two hooks are
 * the real proof and the real acquisition start, both of which reach the cliff runtime. A request
 * arriving before that runtime is configured would be answered by a worker calling into a subsystem
 * still being set up. */
ZTEST(tof_commission_boot, test_the_downlink_starts_after_the_cliff_bootstrap)
{
    recorder r{};

    const boot::report rep{boot::run(both(r))};

    zassert_equal(r.steps, 2, "both steps run exactly once each");
    zassert_equal(r.order[0], kCliff, "the cliff bootstrap did not go first");
    zassert_equal(r.order[1], kDownlink, "the downlink did not go last");
    zassert_true(rep.cliff_attempted, "");
    zassert_true(rep.downlink_attempted, "");
    zassert_false(rep.already_run, "");
}

/* AND IT STARTS ANYWAY WHEN THE BOOTSTRAP FAILED. This is the rule that looks wrong: a board whose
 * cliff runtime did not come up still has to ANSWER, because a host holding a durable pending
 * request from a previous boot retransmits into silence for ever otherwise. A status saying the
 * transaction failed is strictly better than a board that cannot be addressed at all. */
ZTEST(tof_commission_boot, test_a_failed_cliff_bootstrap_does_not_silence_the_downlink)
{
    recorder r{};

    r.cliff_rc = -EINVAL;

    const boot::report rep{boot::run(both(r))};

    zassert_equal(rep.cliff_rc, -EINVAL, "the failure is reported rather than swallowed");
    zassert_true(rep.downlink_attempted,
                 "a cliff bootstrap failure left the board unable to answer a commissioning "
                 "request, which is the worse failure");
    zassert_equal(r.order[1], kDownlink, "");
}

/* ONCE PER BOOT, and the second call must run NOTHING. What this holds is a bus resource: a second
 * can_add_rx_filter() on the request identifier does not fail, it delivers every request twice and
 * the board answers twice -- invisible from inside the runtime and silent on the wire. */
ZTEST(tof_commission_boot, test_a_second_boot_runs_nothing_and_says_so)
{
    recorder r{};

    zassert_false(boot::run(both(r)).already_run, "");
    zassert_equal(r.steps, 2, "");

    const boot::report second{boot::run(both(r))};

    zassert_true(second.already_run, "a second boot was not refused");
    zassert_equal(r.steps, 2, "a second boot ran a step: the receive filter would be installed "
                              "twice and every request answered twice");
    zassert_false(second.cliff_attempted, "");
    zassert_false(second.downlink_attempted, "");
}

/* An image with no downlink leaves that step null, and the sequence still runs the half it has. The
 * preprocessor decides what exists; it does not decide the order. */
ZTEST(tof_commission_boot, test_an_image_without_a_downlink_still_bootstraps_the_cliff)
{
    recorder r{};
    boot::steps s{both(r)};

    s.start_downlink = nullptr;

    const boot::report rep{boot::run(s)};

    zassert_true(rep.cliff_attempted, "");
    zassert_false(rep.downlink_attempted, "");
    zassert_equal(r.steps, 1, "");
    zassert_equal(rep.downlink_rc, 0, "an unattempted step reports no outcome");
}

/* The downlink's outcome is carried through unexamined. Deciding what `answering_only` means is the
 * caller's business, not an ordering question -- and a module that interpreted it would be the
 * second opinion this project keeps having to delete. */
ZTEST(tof_commission_boot, test_the_downlink_outcome_is_reported_not_interpreted)
{
    recorder r{};

    r.downlink_state = 1; /* answering_only */

    const boot::report rep{boot::run(both(r))};

    zassert_equal(rep.downlink_rc, 1, "");
    zassert_true(rep.downlink_attempted, "");
}
