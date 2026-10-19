/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * THE BOOT SEQUENCE, AND THE DEFECT IT WAS WRITTEN FOR.
 *
 * The commissioning transaction used to be configured inside the `tof cliff prove` shell command,
 * so it was ready only on the path that goes through a keyboard. The downlink worker calls
 * tof_commissioning::prove() directly, so on a board nobody had typed at, every request was answered
 * `misconfigured` -- observed on dasher2 on 2026-09-21, three frames, neither I2C nor the enable
 * chain touched. Two suites covered the protocol across that seam and neither could see it, because
 * both replaced the real prove() with a fake that succeeds.
 *
 * So the case that matters here is the one that proves A BOOT ALONE IS ENOUGH: prove() has to work
 * without any prior call that a shell command would have made.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "fake_chain.hpp"
#include "tof_chain_controller.hpp"
#include "tof_cliff_runtime.hpp"
#include "tof_commission_wiring.hpp"
#include "tof_commissioning.hpp"
#include "tof_mapping_authority.hpp"

namespace au = lexxhard::tof_authority;
namespace acq = lexxhard::tof_acq;
namespace cm = lexxhard::tof_commissioning;
namespace rt = lexxhard::tof_cliff_runtime;
namespace wiring = lexxhard::tof_commission_wiring;

namespace {

constexpr rt::config kBootTiming{50, 20, 400, K_PRIO_PREEMPT(5), 21000, 3, 7};

K_THREAD_STACK_DEFINE(wiring_acq_stack, 2048);

fake::fake_chain wiring_chain{};

int wiring_set_bus_speed(cm::bus_speed)
{
    return 0;
}

int wiring_quiesce()
{
    return acq::try_stop();
}

int downlink_calls{0};
int downlink_result{0};

int fake_start_downlink(void *)
{
    ++downlink_calls;
    return downlink_result;
}

wiring::inputs complete_inputs()
{
    wiring::inputs in{};

    in.chain = &lexxhard::tof_chain_controller::chain_lock();
    in.ops = &wiring_chain;
    in.set_bus_speed = wiring_set_bus_speed;
    in.quiesce = wiring_quiesce;
    in.start_downlink = fake_start_downlink;
    return in;
}

void before_wiring(void *)
{
    rt::reset_for_test();
    rt::set_thread_stack_for_test(wiring_acq_stack, K_THREAD_STACK_SIZEOF(wiring_acq_stack));
    au::reset_epoch_history_for_test();
    wiring::reset_for_test();
    /* The configuration is a static that outlives a case, and another suite in this binary
     * configures it directly. Without this the headline case below would be reading what a
     * neighbour left behind -- which is the shape of the defect it exists to catch. */
    cm::reset_for_test();
    wiring_chain = fake::fake_chain{};
    downlink_calls = 0;
    downlink_result = 0;
}

}  // namespace

ZTEST_SUITE(tof_commission_wiring, NULL, NULL, before_wiring, NULL, NULL);

/* THE ONE THAT WOULD HAVE CAUGHT IT. Nothing here calls cm::init(): the boot sequence is the only
 * thing that has run, and a proof must already be possible. Before this wiring existed the proof
 * below returned `not_configured` on every board nobody had typed at. */
ZTEST(tof_commission_wiring, test_a_boot_alone_configures_the_transaction)
{
    const wiring::report r{wiring::boot(kBootTiming, complete_inputs())};

    zassert_equal(r.bootstrap_rc, 0, "");
    zassert_equal(r.configure_rc, 0, "the boot did not configure the commissioning transaction");
    zassert_equal(wiring::configure_status(), 0, "");

    /* And the proof really runs, rather than refusing for want of a configuration. Its verdict is
     * not the point -- what is checked is that it got past the configuration gate at all, with no
     * shell command anywhere in this test. */
    const cm::outcome o{cm::prove(7)};
    zassert_not_equal(o.failed_at, cm::stage::not_configured,
                      "prove() refused as unconfigured after a complete boot: the transaction is "
                      "still being set up by the shell command and not by the boot");
}

/* A FAILED BOOTSTRAP MUST NOT LEAVE A TRANSACTION THAT LOOKS READY. The runtime spec has static
 * storage and already holds the six product positions before bootstrap, so cm::init() against a
 * partial runtime would succeed -- and the first request would be the first anyone heard of it. */
ZTEST(tof_commission_wiring, test_a_failed_bootstrap_does_not_configure_the_transaction)
{
    rt::config bad{kBootTiming};
    bad.cliff_timing_budget_us = 0; /* refused by bootstrap, which names the property */

    const wiring::report r{wiring::boot(bad, complete_inputs())};

    zassert_not_equal(r.bootstrap_rc, 0, "");
    zassert_equal(r.configure_rc, r.bootstrap_rc,
                  "a failed bootstrap configured the transaction anyway");
    zassert_equal(cm::prove(7).failed_at, cm::stage::not_configured,
                  "the transaction was configured against a runtime that did not come up");
}

/* AND IT STILL STARTS THE DOWNLINK. This is the rule that looks wrong and is not: a board whose
 * bootstrap failed still has to ANSWER, or a host holding a durable pending request from a previous
 * boot retransmits into silence for ever. `misconfigured` is terminal and true. */
ZTEST(tof_commission_wiring, test_a_failed_bootstrap_does_not_silence_the_downlink)
{
    rt::config bad{kBootTiming};
    bad.cliff_timing_budget_us = 0;

    const wiring::report r{wiring::boot(bad, complete_inputs())};

    zassert_true(r.downlink_attempted,
                 "a failed bootstrap left the board unable to answer a commissioning request, "
                 "which is the worse failure");
    zassert_equal(downlink_calls, 1, "");
}

/* A missing piece is refused HERE, by name, rather than handed to init() as a config full of null
 * pointers -- which would also be refused, with one -EINVAL for any of five reasons. */
ZTEST(tof_commission_wiring, test_an_incomplete_wiring_is_refused_before_the_transaction)
{
    wiring::inputs in{complete_inputs()};
    in.set_bus_speed = nullptr;

    const wiring::report r{wiring::boot(kBootTiming, in)};

    zassert_equal(r.bootstrap_rc, 0, "the bootstrap is not what failed");
    zassert_equal(r.configure_rc, -EINVAL, "");
    zassert_equal(cm::prove(7).failed_at, cm::stage::not_configured, "");
    /* Still answering, for the same reason as above. */
    zassert_equal(downlink_calls, 1, "");
}

/* ONCE PER BOOT, and the second call must run NOTHING. What this holds is a bus resource: a second
 * can_add_rx_filter() on the request identifier does not fail, it delivers every request twice and
 * the board answers twice -- invisible inside the runtime and silent on the wire. */
ZTEST(tof_commission_wiring, test_a_second_boot_runs_nothing_and_says_so)
{
    zassert_false(wiring::boot(kBootTiming, complete_inputs()).already_run, "");
    zassert_equal(downlink_calls, 1, "");

    const wiring::report second{wiring::boot(kBootTiming, complete_inputs())};

    zassert_true(second.already_run, "a second boot was not refused");
    zassert_equal(downlink_calls, 1,
                  "a second boot started the downlink again: the receive filter would be installed "
                  "twice and every request answered twice");
    zassert_false(second.downlink_attempted, "");
    /* The status the first boot established is what a reader still gets. */
    zassert_equal(second.configure_rc, 0, "");
    zassert_equal(wiring::configure_status(), 0, "");
}

/* An image with no downlink leaves that step null and the sequence runs the half it has. The
 * preprocessor decides what exists; it does not decide the order. */
ZTEST(tof_commission_wiring, test_an_image_without_a_downlink_still_configures_the_transaction)
{
    wiring::inputs in{complete_inputs()};
    in.start_downlink = nullptr;

    const wiring::report r{wiring::boot(kBootTiming, in)};

    zassert_equal(r.configure_rc, 0, "");
    zassert_false(r.downlink_attempted, "");
    zassert_equal(downlink_calls, 0, "");
    zassert_equal(r.downlink_rc, 0, "an unattempted step reports no outcome");
}

/* The downlink's outcome is carried through unexamined. Deciding what the binding's three outcomes
 * mean is the caller's business, not an ordering question -- and a module that interpreted one here
 * would be the second opinion this project keeps having to delete. */
ZTEST(tof_commission_wiring, test_the_downlink_outcome_is_reported_not_interpreted)
{
    downlink_result = 1; /* answering_only */

    const wiring::report r{wiring::boot(kBootTiming, complete_inputs())};

    zassert_equal(r.downlink_rc, 1, "");
    zassert_true(r.downlink_attempted, "");
    /* And it did not change the transaction's verdict. */
    zassert_equal(r.configure_rc, 0, "");
}

/* tof_cliff_runtime.cpp marks the watchdog's baseline from inside the keying commit, so linking it
 * needs this symbol. Stubbed rather than satisfied by linking the real feeder, which would pull the
 * task watchdog, the tombstone and its DTCM reservation into a suite about commissioning. What the
 * feeder does with the flag is tests/tof_watchdog_feeder's business; that it is set at the commit
 * is a property of the call site, which is one line in a file this suite already links. */
namespace lexxhard::tof_watchdog_feeder {

bool baseline_point_set{false};

void set_baseline_point(bool ready)
{
    baseline_point_set = ready;
}

/* Recorded rather than ignored, because the sequence is the claim: the quiesce says stopped and a
 * successful start says expected again, and a suite that only counted the calls could not tell
 * those two apart. Starts at `true` because the feeder's own default is expected. */
bool acquisition_expected{true};

void set_acquisition_expected(bool expected)
{
    acquisition_expected = expected;
}

}  // namespace lexxhard::tof_watchdog_feeder
