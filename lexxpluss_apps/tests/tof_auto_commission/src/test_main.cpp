/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The unattended prove-then-start sequence, with every hook injected.
 *
 * The hooks are fakes and the sequencing is real. The one invariant these tests exist for is the
 * one that decides whether a failure can put measurements on the bus: start() is reached only from
 * a prove() that returned success. Every failure case below asserts the start counter, not just the
 * returned status, because a status is what the module says and the counter is what it did.
 *
 * The success path is driven by an injected epoch on purpose. Where a real epoch comes from is a
 * route decision still with release and safety, so a success path that looked field-reachable here
 * would be asserting something this branch cannot deliver.
 */

#include <zephyr/ztest.h>

#include "tof_auto_commission.hpp"

namespace ac = lexxhard::tof_auto_commission;

namespace {

struct fakes {
    bool permitted{true};
    bool epoch_available{true};
    uint32_t epoch{7};
    int prove_rc{0};
    int start_rc{0};

    int permitted_calls{0};
    int acquire_calls{0};
    int prove_calls{0};
    int start_calls{0};
    uint32_t last_epoch_proved{0};
};

fakes f_{};

bool fake_permitted(void *ctx)
{
    auto *f{static_cast<fakes *>(ctx)};
    ++f->permitted_calls;
    return f->permitted;
}

int fake_acquire(void *ctx, uint32_t *out)
{
    auto *f{static_cast<fakes *>(ctx)};
    ++f->acquire_calls;
    if (!f->epoch_available)
        return -1;
    *out = f->epoch;
    return 0;
}

int fake_prove(void *ctx, uint32_t epoch)
{
    auto *f{static_cast<fakes *>(ctx)};
    ++f->prove_calls;
    f->last_epoch_proved = epoch;
    return f->prove_rc;
}

int fake_start(void *ctx)
{
    auto *f{static_cast<fakes *>(ctx)};
    ++f->start_calls;
    return f->start_rc;
}

ac::hooks wired()
{
    return ac::hooks{ fake_permitted, fake_acquire, fake_prove, fake_start, &f_ };
}

void before(void *)
{
    f_ = fakes{};
}

void enable(uint8_t attempts, uint8_t start_attempts = 3)
{
    ac::config c{};
    c.enabled = true;
    c.max_attempts = attempts;
    c.max_start_attempts = start_attempts;
    ac::init(c, wired());
}

} // namespace

ZTEST_SUITE(tof_auto_commission, NULL, NULL, before, NULL, NULL);

/* ---- default off ---- */

ZTEST(tof_auto_commission, test_a_default_configuration_is_off_and_calls_nothing)
{
    ac::config c{}; /* enabled defaults false */
    ac::init(c, wired());

    zassert_equal(ac::current(), ac::state::disabled, "a default configuration is off");
    zassert_equal(ac::step(), ac::step_result::disabled, "and stepping it does nothing");
    zassert_equal(f_.permitted_calls, 0, "it does not even ask whether it may run");
    zassert_equal(f_.prove_calls, 0, "nothing is proved");
    zassert_equal(f_.start_calls, 0, "and nothing is started");
}

ZTEST(tof_auto_commission, test_an_unwired_machine_reports_a_configuration_error)
{
    ac::config c{};
    c.enabled = true;
    c.max_attempts = 3;
    c.max_start_attempts = 3;
    ac::hooks half{};
    half.enumeration_permitted = fake_permitted; /* the rest left null */
    half.ctx = &f_;
    ac::init(c, half);

    /* Not `disabled`: switched off is a choice and a missing hook is a fault, and a missing hook
     * will not resolve itself. Reporting it as disabled would describe a broken deployment as a
     * deliberate configuration. */
    zassert_equal(ac::current(), ac::state::misconfigured, "on and unable to act is a fault");
    zassert_equal(ac::step(), ac::step_result::misconfigured, "and it says which");
    zassert_equal(f_.permitted_calls, 0, "no hook is called at all");
    zassert_equal(f_.start_calls, 0, "and nothing is started");

    /* It does not resolve by being stepped again. */
    zassert_equal(ac::step(), ac::step_result::misconfigured, "still a fault");
}

/* ---- the permission condition ---- */

ZTEST(tof_auto_commission, test_it_does_not_run_while_enumeration_is_not_permitted)
{
    enable(3);
    f_.permitted = false;

    zassert_equal(ac::step(), ac::step_result::not_permitted, "it asked and was told no");
    zassert_equal(f_.permitted_calls, 1, "it asked");
    zassert_equal(f_.acquire_calls, 0, "and did not reach for an epoch, which could spend one");
    zassert_equal(f_.prove_calls, 0, "nothing is proved");
    zassert_equal(f_.start_calls, 0, "nothing is started");
    zassert_equal(ac::attempts_used(), 0, "and no attempt is spent on a condition it cannot change");
}

ZTEST(tof_auto_commission, test_no_epoch_available_spends_no_attempt)
{
    enable(3);
    f_.epoch_available = false;

    zassert_equal(ac::step(), ac::step_result::no_epoch, "there is nothing to attempt with");
    zassert_equal(f_.prove_calls, 0, "so nothing is attempted");
    zassert_equal(f_.start_calls, 0, "and nothing is started");
    zassert_equal(ac::attempts_used(), 0, "the retry budget is not spent on it");

    /* And it stays available for the moment an epoch does arrive. */
    f_.epoch_available = true;
    zassert_equal(ac::step(), ac::step_result::started, "an epoch arriving is all it was waiting for");
}

/* ---- failure paths never start acquisition ---- */

ZTEST(tof_auto_commission, test_a_failed_proof_never_starts_acquisition)
{
    enable(3);
    f_.prove_rc = -5;

    zassert_equal(ac::step(), ac::step_result::proof_failed, "nothing was installed");
    zassert_equal(f_.prove_calls, 1, "one attempt");
    zassert_equal(f_.start_calls, 0,
                  "and acquisition is not started -- this is what keeps 0x216 off the bus");
    zassert_equal(ac::current(), ac::state::waiting, "still waiting, not proven");
}

ZTEST(tof_auto_commission, test_retries_are_bounded_and_exhaustion_is_terminal)
{
    enable(3);
    f_.prove_rc = -5;

    zassert_equal(ac::step(), ac::step_result::proof_failed, "1");
    zassert_equal(ac::step(), ac::step_result::proof_failed, "2");
    zassert_equal(ac::step(), ac::step_result::attempts_exhausted, "3 spends the budget");
    zassert_equal(ac::current(), ac::state::exhausted, "and that is terminal");

    /* Terminal means terminal: the chain is left alone for whoever is trying to look at it. */
    zassert_equal(ac::step(), ac::step_result::attempts_exhausted, "no further attempt");
    zassert_equal(ac::step(), ac::step_result::attempts_exhausted, "still none");
    zassert_equal(f_.prove_calls, 3, "exactly max_attempts proofs, ever");
    zassert_equal(f_.start_calls, 0, "and acquisition never started");
}

ZTEST(tof_auto_commission, test_a_chain_held_by_an_operator_is_an_ordinary_bounded_failure)
{
    /* The transaction takes the chain with K_NO_WAIT and refuses when a shell command holds it, so
     * an operator at the console arrives here as a proof failure. It must be bounded like any
     * other, not retried until they give up. */
    enable(2);
    f_.prove_rc = -16; /* -EBUSY shaped */

    zassert_equal(ac::step(), ac::step_result::proof_failed, "refused while the operator holds it");
    zassert_equal(ac::step(), ac::step_result::attempts_exhausted, "and bounded");
    zassert_equal(f_.prove_calls, 2, "it does not keep taking the chain away");
    zassert_equal(f_.start_calls, 0, "nothing started");
}

/* ---- the success path, and what follows it ---- */

ZTEST(tof_auto_commission, test_a_successful_proof_starts_acquisition_once)
{
    enable(3);

    zassert_equal(ac::step(), ac::step_result::started, "proved and started");
    zassert_equal(f_.prove_calls, 1, "one proof");
    zassert_equal(f_.last_epoch_proved, 7u, "with the epoch the source supplied");
    zassert_equal(f_.start_calls, 1, "one start");
    zassert_equal(ac::current(), ac::state::started, "terminal for this power-on");

    zassert_equal(ac::step(), ac::step_result::already_started, "and it does not run again");
    zassert_equal(f_.prove_calls, 1, "no second proof");
    zassert_equal(f_.start_calls, 1, "no second start");
}

ZTEST(tof_auto_commission, test_a_failed_start_retries_the_start_and_not_the_proof)
{
    enable(3);
    f_.start_rc = -1;

    zassert_equal(ac::step(), ac::step_result::start_failed, "the mapping is proven, acquisition is not");
    zassert_equal(ac::current(), ac::state::proven, "and that is remembered");
    zassert_equal(f_.prove_calls, 1, "one proof so far");

    /* Re-proving here would spend another epoch for a failure that has nothing to do with the
     * mapping. */
    f_.start_rc = 0;
    zassert_equal(ac::step(), ac::step_result::started, "the second step starts it");
    zassert_equal(f_.prove_calls, 1, "and did NOT prove again");
    zassert_equal(f_.start_calls, 2, "it retried only the start");
}

ZTEST(tof_auto_commission, test_a_zero_proof_budget_is_a_configuration_fault)
{
    /* max_attempts defaults to 0, so a configuration that switches the machine on without saying how
     * many attempts it may make describes a machine that can never reach started. That is reported
     * as a fault rather than as a quiet nothing. */
    enable(0, 3);

    zassert_equal(ac::current(), ac::state::misconfigured, "on and unable to finish");
    zassert_equal(ac::step(), ac::step_result::misconfigured, "and it says which");
    zassert_equal(f_.acquire_calls, 0, "no epoch is reached for");
    zassert_equal(f_.prove_calls, 0, "nothing is proved");
    zassert_equal(f_.start_calls, 0, "and nothing is started");
}

/* ---- the start budget ---- */

ZTEST(tof_auto_commission, test_start_retries_are_bounded_too)
{
    /* A refusing start must not be retried forever either: the mapping is proven and the epoch is
     * spent, and an unbounded retry keeps taking the chain from whoever is looking at the machine. */
    enable(3, 2);
    f_.start_rc = -1;

    zassert_equal(ac::step(), ac::step_result::start_failed, "first start refused");
    zassert_equal(ac::step(), ac::step_result::start_failed, "second refused");
    zassert_equal(ac::step(), ac::step_result::start_attempts_exhausted, "and the budget is spent");
    zassert_equal(f_.start_calls, 2, "exactly max_start_attempts starts, ever");
    zassert_equal(f_.prove_calls, 1, "and it never re-proved");
    zassert_equal(ac::current(), ac::state::proven, "the mapping stays proven and unstarted");

    zassert_equal(ac::step(), ac::step_result::start_attempts_exhausted, "still spent");
    zassert_equal(f_.start_calls, 2, "no further start");
}

ZTEST(tof_auto_commission, test_a_zero_start_budget_is_caught_before_an_epoch_is_spent)
{
    /* This is the one that mattered. With no start budget the sequence used to acquire an epoch and
     * run the whole proof -- installing a mapping and spending an ordinal -- before refusing to
     * start, so a misconfiguration cost a real epoch and a real change to the chain on every step. */
    enable(3, 0);

    zassert_equal(ac::current(), ac::state::misconfigured, "caught at configuration time");
    zassert_equal(ac::step(), ac::step_result::misconfigured, "and reported as a fault");
    zassert_equal(f_.acquire_calls, 0, "no epoch is acquired, so none is spent");
    zassert_equal(f_.prove_calls, 0, "and the mapping is not touched");
    zassert_equal(f_.start_calls, 0, "nothing is started");
    zassert_equal(ac::attempts_used(), 0, "no proof attempt is counted either");
}
