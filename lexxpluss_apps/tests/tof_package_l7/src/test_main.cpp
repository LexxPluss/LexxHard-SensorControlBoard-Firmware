/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The Package lifecycle's ordering rules. Every hook is a spy, so each case asserts both the state
 * that was reached and WHAT WAS NOT CALLED -- which for this module is the more important half:
 * its reason to exist is that one class of failure must not be followed by an open.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_l7_quiesce.hpp"
#include "tof_l7_step_record.hpp"
#include "tof_package_l7.hpp"

namespace pkg = lexxhard::tof_package;
namespace steps = lexxhard::tof_l7_steps;
namespace quiesce = lexxhard::tof_l7_quiesce;
namespace tof_l7 = lexxhard::tof_l7;

namespace {

/* THE FAKE OBEYS THE REAL SENSOR'S LIFECYCLE, and that is the point of it.
 *
 * The first version of these fakes only returned scripted codes. Every case passed, and the module
 * still could not run `up -> stop -> up` on hardware: a confirmed stop leaves tof_l7's sensor
 * `configured`, open() requires `empty`, and there was no close in between. A fake that answers 0
 * to any open cannot show that, so it hid the one defect the suite was there to catch.
 *
 * So this mirrors tof_l7_sensor.cpp's transitions exactly -- open requires empty; configure
 * requires opened; start requires configured and lands in stop_unconfirmed on failure; stop accepts
 * running and stop_unconfirmed and returns to configured; close refuses running and
 * stop_unconfirmed; read requires running -- and refuses with -EPERM as the real one does. The
 * scripted return codes still apply, but only to a call the state would have allowed. */
enum class sensor_state : uint8_t {
    empty,
    opened,
    configured,
    running,
    stop_unconfirmed,
};

struct spy {
    int verified_rc{0};
    int probe_rc{0};
    bool probe_present{false};
    bool probe_wrong_identity{false};
    int quiesce_rc{0};
    int close_rc{0};
    int open_rc{0};
    int configure_rc{0};
    int start_rc{0};
    int read_rc{0};
    bool read_fresh{true};
    int stop_rc{0};

    int verified_calls{0};
    int probe_calls{0};
    int quiesce_calls{0};
    int close_calls{0};
    int open_calls{0};
    int configure_calls{0};
    int start_calls{0};
    int read_calls{0};
    int stop_calls{0};

    uint8_t probed_addr{0};
    uint8_t opened_addr{0};
    uint8_t configured_hz{0};

    sensor_state device{sensor_state::empty};
    /* Set by any fake that was called in a state the real sensor would have refused. Asserted at
     * the end of the ordering cases, because such a call is a defect even when the lifecycle's own
     * bookkeeping came out right. */
    int illegal_calls{0};
};

spy spy_;

int fake_verified()
{
    ++spy_.verified_calls;
    return spy_.verified_rc;
}

int fake_probe(void *, uint8_t addr, bool *present, bool *wrong_identity)
{
    ++spy_.probe_calls;
    spy_.probed_addr = addr;
    /* Deliberately writes both flags even on a failure, because a port that leaves stale values
     * behind is exactly what the lifecycle must not rely on. */
    *present = spy_.probe_present;
    *wrong_identity = spy_.probe_wrong_identity;
    return spy_.probe_rc;
}

int fake_quiesce(void *, uint8_t)
{
    ++spy_.quiesce_calls;
    return spy_.quiesce_rc;
}

int fake_close(void *)
{
    ++spy_.close_calls;
    if (spy_.device == sensor_state::running || spy_.device == sensor_state::stop_unconfirmed) {
        ++spy_.illegal_calls;
        return -EPERM;
    }
    if (spy_.close_rc != 0)
        return spy_.close_rc;
    spy_.device = sensor_state::empty;
    return 0;
}

int fake_open(void *, uint8_t addr)
{
    ++spy_.open_calls;
    spy_.opened_addr = addr;
    if (spy_.device != sensor_state::empty) {
        ++spy_.illegal_calls;
        return -EPERM;
    }
    if (spy_.open_rc != 0)
        return spy_.open_rc;
    spy_.device = sensor_state::opened;
    return 0;
}

int fake_configure(void *, uint8_t hz)
{
    ++spy_.configure_calls;
    spy_.configured_hz = hz;
    if (spy_.device != sensor_state::opened) {
        ++spy_.illegal_calls;
        return -EPERM;
    }
    if (spy_.configure_rc != 0)
        return spy_.configure_rc;
    spy_.device = sensor_state::configured;
    return 0;
}

int fake_start(void *)
{
    ++spy_.start_calls;
    if (spy_.device != sensor_state::configured) {
        ++spy_.illegal_calls;
        return -EPERM;
    }
    /* A FAILED START IS NOT A DEVICE THAT STAYED PUT. The command may have gone out, which is why
     * the real sensor lands in stop_unconfirmed, and why the lifecycle above owes a stop. */
    spy_.device = spy_.start_rc == 0 ? sensor_state::running : sensor_state::stop_unconfirmed;
    return spy_.start_rc;
}

int fake_read(void *, bool *fresh)
{
    ++spy_.read_calls;
    if (spy_.device != sensor_state::running) {
        ++spy_.illegal_calls;
        *fresh = false;
        return -EPERM;
    }
    *fresh = spy_.read_fresh;
    return spy_.read_rc;
}

int fake_stop(void *)
{
    ++spy_.stop_calls;
    if (spy_.device != sensor_state::running && spy_.device != sensor_state::stop_unconfirmed) {
        ++spy_.illegal_calls;
        return -EPERM;
    }
    spy_.device = spy_.stop_rc == 0 ? sensor_state::configured : sensor_state::stop_unconfirmed;
    return spy_.stop_rc;
}

pkg::hooks all_hooks()
{
    pkg::hooks h{};
    h.ensure_verified = fake_verified;
    h.probe = fake_probe;
    h.quiesce = fake_quiesce;
    h.close = fake_close;
    h.open = fake_open;
    h.configure = fake_configure;
    h.start = fake_start;
    h.read = fake_read;
    h.stop = fake_stop;
    h.ctx = &spy_;
    return h;
}

pkg::config good_config()
{
    pkg::config c{};
    c.address_7bit = 0x29;
    c.frequency_hz = 10;
    c.stop_attempts = 3;
    return c;
}

pkg::lifecycle lc_;

/* THE BENCH'S HOOKS, in the shape the shell adapter gives them: each one runs the step and then
 * records what it reported. The five below delegate to the fakes above, so the ordering cases and
 * the round-boundary cases are driven by the same device model.
 *
 * The status is built here the way the adapter builds it -- out of what the call reported, with no
 * second opinion -- because what is under test is WHEN the records are cleared, not what the
 * sensor layer puts in them. */
tof_l7::operation_status reported(int rc)
{
    tof_l7::operation_status st{};
    st.failed_stage = rc == 0 ? tof_l7::stage::none : tof_l7::stage::initialise;
    st.port_errno = rc;
    return st;
}

int recording_close(void *c)
{
    const int rc{fake_close(c)};
    steps::remember(steps::which::close, rc, reported(rc));
    return rc;
}

int recording_open(void *c, uint8_t addr)
{
    const int rc{fake_open(c, addr)};
    steps::remember(steps::which::open, rc, reported(rc));
    return rc;
}

int recording_configure(void *c, uint8_t hz)
{
    const int rc{fake_configure(c, hz)};
    steps::remember(steps::which::configure, rc, reported(rc));
    return rc;
}

int recording_start(void *c)
{
    const int rc{fake_start(c)};
    steps::remember(steps::which::start, rc, reported(rc));
    return rc;
}

int recording_read(void *c, bool *fresh)
{
    const int rc{fake_read(c, fresh)};
    steps::remember(steps::which::read, rc, reported(rc));
    return rc;
}

pkg::hooks recording_hooks()
{
    pkg::hooks h{all_hooks()};
    h.close = recording_close;
    h.open = recording_open;
    h.configure = recording_configure;
    h.start = recording_start;
    h.read = recording_read;
    return h;
}

/* WHAT cmd_up() DOES, in its order and with nothing else between. The shell command adds only the
 * printing; the gate beside the quiesce suite pins that this pair is what it runs. */
int an_up()
{
    steps::begin_round(lc_.can_begin_round());
    return lc_.bring_up();
}

void before(void *)
{
    spy_ = spy{};
    lc_ = pkg::lifecycle{};
    /* The records are file-scope state in their module, so a case that did not clear them would be
     * reading the previous case's run. */
    steps::forget_all();
    quiesce::forget_all();
}

}  // namespace

ZTEST_SUITE(tof_package_l7, NULL, NULL, before, NULL, NULL);

/* ---- the ordering, when everything works ------------------------------------------------- */

ZTEST(tof_package_l7, test_a_cold_bring_up_runs_the_steps_in_order)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = false;

    zassert_ok(lc_.bring_up());

    zassert_equal(spy_.verified_calls, 1, "the one verification is consulted, not repeated");
    zassert_equal(spy_.probe_calls, 1);
    zassert_equal(spy_.quiesce_calls, 0, "nothing answered, so there was nothing to quiesce");
    zassert_equal(spy_.open_calls, 1);
    zassert_equal(spy_.configure_calls, 1);
    zassert_equal(spy_.start_calls, 1);

    zassert_equal(spy_.probed_addr, 0x29);
    zassert_equal(spy_.opened_addr, 0x29);
    zassert_equal(spy_.configured_hz, 10);

    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::ranging);
    zassert_equal(s.failed_step, pkg::step::none);
    zassert_false(s.survivor_found);
    zassert_equal(spy_.illegal_calls, 0, "every call was one the real sensor would have allowed");
}

/* UP -> STOP -> UP, THE ORDINARY BENCH SEQUENCE, which did not work before the close hook existed.
 * A confirmed stop leaves the device `configured`, and open() requires `empty`; without a close in
 * between the second open is refused and the sensor is a one-session-per-boot object. The fake
 * enforces exactly those transitions, so this case fails if the close is dropped again. */
ZTEST(tof_package_l7, test_a_full_cycle_returns_to_idle_and_can_run_again)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());

    bool fresh{false};
    zassert_ok(lc_.read_once(&fresh));
    zassert_true(fresh);

    zassert_ok(lc_.stop());
    zassert_equal(lc_.current().current, pkg::state::idle);
    zassert_equal(spy_.device, sensor_state::configured, "a confirmed stop does not empty the sensor");

    zassert_ok(lc_.bring_up(), "idle is the state an open may start from");
    zassert_equal(spy_.open_calls, 2);
    zassert_equal(spy_.close_calls, 2, "closed before each open, including the first");
    zassert_equal(spy_.illegal_calls, 0, "and no call was made in a state the sensor refuses");
}

/* The close runs before EVERY open, not only the one that needs it. An unconditional close is
 * harmless from `empty` and is the only version that cannot be got wrong by a caller reasoning
 * about which previous failure left what behind. */
ZTEST(tof_package_l7, test_the_close_runs_before_the_first_open_too)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());

    zassert_equal(spy_.close_calls, 1);
    zassert_equal(spy_.open_calls, 1);
}

/* A CLOSE THAT REFUSES STOPS THE RUN BEFORE THE OPEN. The real close refuses exactly when the
 * device may be ranging, so continuing past it is the one thing that must not happen. */
ZTEST(tof_package_l7, test_a_refused_close_stops_the_run_before_the_open)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.close_rc = -EPERM;

    zassert_equal(lc_.bring_up(), -EPERM);

    zassert_equal(spy_.open_calls, 0, "no open after a close that would not give up the session");
    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::blocked);
    zassert_equal(s.failed_step, pkg::step::close);
}

/* ---- the rule this module exists for ------------------------------------------------------ */

/* AN UNCLASSIFIABLE PROBE IS NOT AN EMPTY ADDRESS. The chain can record a probe failure and carry
 * on to other positions; here there is no other position and no enumerator, so continuing would
 * mean opening a device that may already be ranging -- and the only way out of that on this sensor
 * is a machine power cycle. */
ZTEST(tof_package_l7, test_a_probe_that_did_not_complete_must_not_be_followed_by_an_open)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_rc = -EIO;
    spy_.probe_present = false;  /* a stale false, which must not be trusted */

    zassert_equal(lc_.bring_up(), -EIO);

    zassert_equal(spy_.open_calls, 0, "no open");
    zassert_equal(spy_.configure_calls, 0);
    zassert_equal(spy_.start_calls, 0);
    zassert_equal(spy_.quiesce_calls, 0, "and nothing was stopped on a guess either");

    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::blocked);
    zassert_equal(s.failed_step, pkg::step::probe);
    zassert_equal(s.errno_value, -EIO);
    zassert_false(s.survivor_found, "presence is unknown, so it is not recorded as found");
}

/* And a probe that DID complete and found nobody is the ordinary cold boot, which must not be
 * confused with the case above. The two differ only in the probe's return value. */
ZTEST(tof_package_l7, test_a_completed_probe_finding_nobody_is_not_a_failure)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_rc = 0;
    spy_.probe_present = false;

    zassert_ok(lc_.bring_up());
    zassert_equal(lc_.current().current, pkg::state::ranging);
}

/* ---- the warm-reset case ------------------------------------------------------------------ */

/* A SURVIVOR IS QUIESCED FIRST, then initialised in full. An SCB reset does not reset this device
 * and there is no reset line, so it may still be powered and measuring. */
ZTEST(tof_package_l7, test_a_survivor_is_quiesced_before_anything_is_opened)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = true;

    zassert_ok(lc_.bring_up());

    zassert_equal(spy_.quiesce_calls, 1);
    zassert_equal(spy_.open_calls, 1);
    zassert_true(lc_.current().survivor_found, "the warm-reset signal is recorded");
}

/* A SUCCESSFUL QUIESCE DOES NOT LET THE RUN SKIP INITIALISATION. Which device state survives an
 * SCB reset is not established, and the host-side configuration object is destroyed by the reset
 * regardless, so there is nothing to resume into. */
ZTEST(tof_package_l7, test_a_quiesced_survivor_is_still_opened_and_configured_in_full)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = true;

    zassert_ok(lc_.bring_up());

    zassert_equal(spy_.open_calls, 1, "the firmware download is not skipped");
    zassert_equal(spy_.configure_calls, 1);
    zassert_equal(spy_.start_calls, 1);
}

/* A SURVIVOR THAT CANNOT BE QUIESCED IS TERMINAL FOR THE RUN. Opening on top of a live session is
 * the one thing this ordering prevents. */
ZTEST(tof_package_l7, test_a_survivor_that_cannot_be_quiesced_blocks_the_run)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = true;
    spy_.quiesce_rc = -ETIMEDOUT;

    zassert_equal(lc_.bring_up(), -ETIMEDOUT);

    zassert_equal(spy_.open_calls, 0, "no open on top of a device that may still be ranging");
    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::blocked);
    zassert_equal(s.failed_step, pkg::step::survivor);
    zassert_true(s.survivor_found);
}

ZTEST(tof_package_l7, test_a_stop_after_a_survivor_refusal_is_a_state_refusal_and_asks_no_device)
{
    /* THE MISREPORT THIS CASE EXISTS FOR. On 2026-10-12 a bench run refused at the survivor step and
     * the operator then ran `stop`, which printed "the retry budget is spent. The device may still
     * be measuring and this tool cannot stop it". None of that had happened: stop() returned -EPERM
     * at its state guard, the device was never asked, and the budget was untouched. The shell had
     * inferred the reason from `current == blocked`, and `blocked` is reached by every refusal
     * earlier than a stop as well as by a spent budget.
     *
     * So this pins the three things the message got wrong, at the layer that knows them. */
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = true;
    spy_.quiesce_rc = -ETIMEDOUT;
    zassert_equal(lc_.bring_up(), -ETIMEDOUT);
    zassert_equal(lc_.current().current, pkg::state::blocked);

    const int stop_calls_before{spy_.stop_calls};
    const uint8_t budget_before{lc_.current().stop_attempts_used};

    zassert_equal(lc_.stop(), -EPERM, "nothing here owes a stop");
    zassert_equal(spy_.stop_calls, stop_calls_before, "the device must not be asked");
    zassert_equal(lc_.current().stop_attempts_used, budget_before, "the budget must not move");
    zassert_equal(lc_.current().last_stop_refusal, pkg::stop_refusal::wrong_state,
                  "and the refusal must say which one it was, not leave it to be guessed from the "
                  "state");

    /* Repeating it does not accumulate anything either: still a state refusal, still no device
     * call. A reader who tries twice must not end up with a spent-looking budget. */
    zassert_equal(lc_.stop(), -EPERM);
    zassert_equal(spy_.stop_calls, stop_calls_before);
    zassert_equal(lc_.current().stop_attempts_used, budget_before);
    zassert_equal(lc_.current().last_stop_refusal, pkg::stop_refusal::wrong_state);
}

ZTEST(tof_package_l7, test_a_spent_budget_is_told_apart_by_the_step_not_by_the_refusal)
{
    /* THE OTHER SIDE OF THE SAME DISTINCTION, and it does not live where the first attempt at this
     * case assumed. `stop_refusal::budget_spent` is unreachable: the attempt that spends the budget
     * also sets the state to `blocked`, so the next stop() answers at the state guard. Writing the
     * case that expected `budget_spent` is what surfaced that -- see stop_refusal for why the
     * guards are not reordered to make it reachable.
     *
     * What IS observable, and what the shell has to report from, is the pair: `blocked` with
     * `failed_step == step::stop` is the device-was-asked case. Here the device really is asked, as
     * many times as configured. */
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.start_rc = -EIO;
    zassert_equal(lc_.bring_up(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::stop_owed, "a failed start owes a stop");

    spy_.stop_rc = -ETIMEDOUT;
    int asked{0};
    while (lc_.stop() == -ETIMEDOUT)
        ++asked;

    zassert_true(asked > 0, "the device was asked at least once");
    zassert_equal(spy_.stop_calls, asked, "every attempt reached the device");
    zassert_equal(lc_.current().current, pkg::state::blocked);
    zassert_equal(lc_.current().failed_step, pkg::step::stop,
                  "and the step says the stop is what gave up -- this is the pair that means a "
                  "device may still be measuring");

    /* The refusal after that is a state refusal, reaching no device -- so the reason an operator
     * needs is in the step, not in this field. */
    const int reached{spy_.stop_calls};
    zassert_equal(lc_.stop(), -EPERM);
    zassert_equal(spy_.stop_calls, reached, "the refused attempt after exhaustion reached nothing");
    zassert_equal(lc_.current().last_stop_refusal, pkg::stop_refusal::wrong_state);
    zassert_equal(lc_.current().failed_step, pkg::step::stop, "the step still names the stop");
}

ZTEST(tof_package_l7, test_the_two_blocked_cases_are_distinguishable_from_what_is_recorded)
{
    /* The whole point, stated as one comparison: both runs end `blocked` with a `wrong_state`
     * refusal, and only `failed_step` separates "asked and gave up" from "never asked". A reporter
     * that looks at the state alone -- which is what printed the wrong message on 2026-10-12 --
     * cannot tell these two apart. */
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = true;
    spy_.quiesce_rc = -ETIMEDOUT;
    zassert_equal(lc_.bring_up(), -ETIMEDOUT);
    zassert_equal(lc_.stop(), -EPERM);

    zassert_equal(lc_.current().current, pkg::state::blocked);
    zassert_equal(lc_.current().last_stop_refusal, pkg::stop_refusal::wrong_state);
    zassert_equal(lc_.current().failed_step, pkg::step::survivor, "never asked for a stop");
    zassert_equal(spy_.stop_calls, 0, "and the device confirms it");
}

ZTEST(tof_package_l7, test_the_attempt_that_spends_the_budget_is_not_reported_as_retryable)
{
    /* REVIEW'S REPRODUCTION, AT THE LAYER THAT DECIDES THE MESSAGE. With a budget of one, a device
     * that refuses the stop leaves `blocked` at step `stop` with NO refusal recorded -- the attempt
     * was made, so nothing refused it -- and the shell's inline branch printed "Retry it". The next
     * stop() is then certain to be refused: the operator is told to do the one thing that cannot
     * work, on the one path where the device may still be measuring.
     *
     * So the report is asserted on the attempt itself, not on the refusal after it. */
    pkg::config one_attempt{good_config()};
    one_attempt.stop_attempts = 1;
    zassert_ok(lc_.init(all_hooks(), one_attempt));
    spy_.start_rc = -EIO;
    zassert_equal(lc_.bring_up(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::stop_owed);

    spy_.stop_rc = -ETIMEDOUT;
    zassert_equal(lc_.stop(), -ETIMEDOUT, "the device was asked and refused");
    zassert_equal(spy_.stop_calls, 1);
    zassert_equal(lc_.current().current, pkg::state::blocked, "and that was the only attempt allowed");
    zassert_equal(lc_.current().failed_step, pkg::step::stop);
    zassert_equal(lc_.current().last_stop_refusal, pkg::stop_refusal::none,
                  "nothing REFUSED this one -- which is exactly why the state has to be consulted");

    const pkg::stop_report what{pkg::report_for(lc_.current())};
    zassert_false(what.retryable, "no further stop can reach the device, so it must not say retry");
    zassert_not_null(what.text);
    zassert_is_null(strstr(what.text, "Retry it"), "the text said: %s", what.text);
    zassert_not_null(strstr(what.text, "may still be measuring"),
                     "and it has to say the thing that matters: %s", what.text);
}

ZTEST(tof_package_l7, test_a_refused_stop_with_budget_left_is_the_only_retryable_report)
{
    /* The other side: more than one attempt allowed, so a retry really can reach the device, and
     * this is the one case where saying so is correct. */
    pkg::config several{good_config()};
    several.stop_attempts = 3;
    zassert_ok(lc_.init(all_hooks(), several));
    spy_.start_rc = -EIO;
    zassert_equal(lc_.bring_up(), -EIO);

    spy_.stop_rc = -ETIMEDOUT;
    zassert_equal(lc_.stop(), -ETIMEDOUT);
    zassert_equal(lc_.current().current, pkg::state::stop_owed, "the debt stands and the budget has "
                                                                "something left");

    const pkg::stop_report what{pkg::report_for(lc_.current())};
    zassert_true(what.retryable);
    zassert_not_null(strstr(what.text, "Retry it"), "the text said: %s", what.text);
}

ZTEST(tof_package_l7, test_the_report_for_a_survivor_refusal_says_the_device_was_not_asked)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_present = true;
    spy_.quiesce_rc = -ETIMEDOUT;
    zassert_equal(lc_.bring_up(), -ETIMEDOUT);
    zassert_equal(lc_.stop(), -EPERM);

    const pkg::stop_report what{pkg::report_for(lc_.current())};
    zassert_false(what.retryable);
    zassert_not_null(strstr(what.text, "NOT asked"), "the text said: %s", what.text);
    zassert_is_null(strstr(what.text, "budget is spent"),
                    "and it must not claim a budget that was never touched: %s", what.text);
}

ZTEST(tof_package_l7, test_the_report_after_an_exhausted_budget_still_names_the_device)
{
    /* One call later than the case above: the refusal is now `wrong_state`, and only `failed_step`
     * keeps the fact that the device WAS asked. */
    pkg::config one_attempt{good_config()};
    one_attempt.stop_attempts = 1;
    zassert_ok(lc_.init(all_hooks(), one_attempt));
    spy_.start_rc = -EIO;
    zassert_equal(lc_.bring_up(), -EIO);
    spy_.stop_rc = -ETIMEDOUT;
    zassert_equal(lc_.stop(), -ETIMEDOUT);
    zassert_equal(lc_.stop(), -EPERM);

    const pkg::stop_report what{pkg::report_for(lc_.current())};
    zassert_false(what.retryable);
    zassert_not_null(strstr(what.text, "may still be measuring"), "the text said: %s", what.text);
    zassert_is_null(strstr(what.text, "NOT asked"),
                    "the device was asked, one call earlier: %s", what.text);
}

ZTEST(tof_package_l7, test_a_stop_before_init_names_that_rather_than_a_budget)
{
    pkg::lifecycle fresh;
    zassert_equal(fresh.stop(), -EPERM);
    zassert_equal(fresh.current().last_stop_refusal, pkg::stop_refusal::not_initialised);
}

/* ---- something answered and is not an L7 -------------------------------------------------- */

/* A WRONG IDENTITY IS NOT A COLD DEVICE, and this is the case that separates the Package sensor from
 * the chain. On the chain, "transport fine, identity mismatch" is a census anomaly the enumerator
 * reports and routes around, and the classifier used to collapse it into the same answer it gives
 * for an empty address. Here the address is fixed at 0x29, the device is always powered, and there
 * is no enumerator -- so that collapse ended in an open, and an 86 KB firmware download, onto a
 * device nobody had identified.
 *
 * Note what a NORMAL Package cold boot looks like: the sensor is powered and DOES answer at 0x29
 * with a matching identity. The chain's "cold boot means nobody answers" reasoning is a property of
 * the enable chain holding LPn low, and it does not carry over to this bus. */
ZTEST(tof_package_l7, test_a_wrong_identity_is_refused_and_nothing_is_opened)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_rc = 0;              /* the probe itself completed */
    spy_.probe_present = false;     /* but not as an L7 */
    spy_.probe_wrong_identity = true;

    zassert_equal(lc_.bring_up(), -ENOTSUP);

    zassert_equal(spy_.quiesce_calls, 0,
                  "an L7 stop sequence does not belong to a device that is not one");
    zassert_equal(spy_.close_calls, 0);
    zassert_equal(spy_.open_calls, 0, "and neither does an 86 KB firmware download");
    zassert_equal(spy_.configure_calls, 0);
    zassert_equal(spy_.start_calls, 0);

    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::blocked);
    zassert_equal(s.failed_step, pkg::step::identity);
    zassert_false(s.survivor_found,
                  "a survivor is an L7 that outlived a reset; this is a different finding");
}

/* The refusal is terminal for the run: what is on the bus is a question for a person, and retrying
 * cannot answer it. */
ZTEST(tof_package_l7, test_a_wrong_identity_is_not_retried_away)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.probe_wrong_identity = true;
    zassert_equal(lc_.bring_up(), -ENOTSUP);

    spy_.probe_wrong_identity = false;
    zassert_equal(lc_.bring_up(), -EPERM);
    zassert_equal(spy_.probe_calls, 1, "and the bus is not probed again");
}

/* ---- what may be thrown away -------------------------------------------------------------- */

/* THE QUESTION A RE-BIND HAS TO ASK. A caller that resets this object wholesale -- which is what the
 * bench's `bind` did -- erases a stop debt that only a successful stop may clear, and on this sensor
 * the way back from a forgotten live session is a machine power cycle. So the object answers it
 * directly, and the answer turns on whether anything was ever opened, not on whether the state looks
 * final. */
ZTEST(tof_package_l7, test_only_a_lifecycle_that_opened_nothing_may_be_discarded)
{
    /* Before init, and from idle. */
    zassert_true(lc_.discardable());
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_true(lc_.discardable());

    /* Ranging: no. */
    zassert_ok(lc_.bring_up());
    zassert_false(lc_.discardable(), "a device that is measuring is not bookkeeping");

    /* A stop debt: no. */
    spy_.stop_rc = -EIO;
    zassert_equal(lc_.stop(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::stop_owed);
    zassert_false(lc_.discardable());

    /* And a blocked reached AFTER the budget ran out still is not, because the device may be
     * ranging and this object is the only record of it. */
    zassert_equal(lc_.stop(), -EIO);
    zassert_equal(lc_.stop(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::blocked);
    zassert_false(lc_.discardable(), "blocked at the stop step is the worst case, not a clean one");

    /* Back to idle after a confirmed stop: yes. */
    pkg::lifecycle clean{};
    zassert_ok(clean.init(all_hooks(), good_config()));
    spy_.stop_rc = 0;
    spy_.device = sensor_state::empty;
    zassert_ok(clean.bring_up());
    zassert_ok(clean.stop());
    zassert_true(clean.discardable());
}

ZTEST(tof_package_l7, test_a_block_that_opened_nothing_may_be_discarded)
{
    /* Each of these stops before the open, so there is no session to forget. Listed one at a time
     * rather than as a loop, because the point is which steps are on the list. */
    const int steps{4};

    for (int i{0}; i < steps; ++i) {
        spy_ = spy{};
        pkg::lifecycle l{};
        zassert_ok(l.init(all_hooks(), good_config()));
        switch (i) {
        case 0: spy_.verified_rc = -ENOENT; break;                      /* firmware */
        case 1: spy_.probe_rc = -EIO; break;                            /* probe */
        case 2: spy_.probe_wrong_identity = true; break;                /* identity */
        case 3: spy_.probe_present = true; spy_.quiesce_rc = -EIO; break; /* survivor */
        }
        zassert_true(l.bring_up() != 0, "case %d was meant to fail", i);
        zassert_equal(l.current().current, pkg::state::blocked, "case %d", i);
        zassert_equal(spy_.open_calls, 0, "case %d opened something", i);
        zassert_true(l.discardable(), "case %d opened nothing, so it may be re-bound", i);
    }
}

/* AND THE TWO BLOCKED STEPS THAT MAY NOT BE DISCARDED. A close only refuses when the sensor is
 * running or owes a stop, so a block there means one of those was true; a spent stop budget is a
 * device this tool has given up on stopping. Both are the case where throwing the object away loses
 * the only record of a live session. */
ZTEST(tof_package_l7, test_a_block_behind_a_possible_session_may_not_be_discarded)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.close_rc = -EPERM;
    zassert_equal(lc_.bring_up(), -EPERM);
    zassert_equal(lc_.current().failed_step, pkg::step::close);
    zassert_false(lc_.discardable(), "a close refuses exactly when the device may be ranging");

    /* And the spent stop budget, which is the worst case of all. */
    spy_ = spy{};
    pkg::config c{good_config()};
    c.stop_attempts = 1;
    pkg::lifecycle l{};
    zassert_ok(l.init(all_hooks(), c));
    zassert_ok(l.bring_up());
    spy_.stop_rc = -EIO;
    zassert_equal(l.stop(), -EIO);
    zassert_equal(l.current().current, pkg::state::blocked);
    zassert_equal(l.current().failed_step, pkg::step::stop);
    zassert_false(l.discardable());
}

/* RE-INIT OVER A LIVE SESSION IS REFUSED, AND REFUSED BEFORE ANYTHING IS WRITTEN. init() otherwise
 * resets the state wholesale, so without this the one call a caller reaches for when it wants to
 * start over is also the one that erases the debt. */
ZTEST(tof_package_l7, test_re_init_is_refused_while_a_session_is_owned)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());

    pkg::config other{good_config()};
    other.frequency_hz = 5;
    zassert_equal(lc_.init(all_hooks(), other), -EPERM);

    /* Nothing was changed by the refusal: still ranging, and still at the original rate. */
    zassert_equal(lc_.current().current, pkg::state::ranging);
    spy_.stop_rc = 0;
    zassert_ok(lc_.stop());
    zassert_ok(lc_.bring_up());
    zassert_equal(spy_.configured_hz, 10, "the refused config was not half-applied");

    /* And once the session is accounted for, re-init is allowed. */
    zassert_ok(lc_.stop());
    zassert_ok(lc_.init(all_hooks(), other));
    zassert_equal(lc_.current().current, pkg::state::idle);
}

/* ---- the firmware gate -------------------------------------------------------------------- */

/* Without a verified payload there is nothing to download, so the run stops before touching the
 * bus at all: probing a device we could not then open would only disturb it. */
ZTEST(tof_package_l7, test_an_unverified_blob_stops_the_run_before_the_bus_is_touched)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.verified_rc = -ENOENT;

    zassert_equal(lc_.bring_up(), -ENOENT);

    zassert_equal(spy_.probe_calls, 0, "the bus is not touched");
    zassert_equal(spy_.open_calls, 0);
    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::blocked);
    zassert_equal(s.failed_step, pkg::step::firmware);
}

/* A VERIFICATION STILL IN FLIGHT MUST NOT BLOCK THE LIFECYCLE. It is not a refusal and not a
 * go-ahead: another consumer holds the one verification, so the answer is simply not in yet.
 *
 * The first version sent it to `blocked` along with every other non-zero code, which turned a few
 * milliseconds of contention between two sensors into a permanent refusal that only a reboot
 * cleared. The state has to stay exactly where it was -- idle, nothing opened, no device disturbed --
 * so that asking again is all it takes. */
ZTEST(tof_package_l7, test_a_verification_in_flight_leaves_the_run_retryable)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.verified_rc = -EINPROGRESS;

    zassert_equal(lc_.bring_up(), -EINPROGRESS);
    zassert_equal(spy_.probe_calls, 0, "the bus is not touched this round");
    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::idle, "NOT blocked: the answer is pending, not negative");
    zassert_equal(s.failed_step, pkg::step::firmware);
    zassert_equal(s.errno_value, -EINPROGRESS,
                  "distinguishable from a terminal refusal by the caller");

    /* And the retry simply runs. */
    spy_.verified_rc = 0;
    zassert_ok(lc_.bring_up());
    zassert_equal(lc_.current().current, pkg::state::ranging);
    zassert_equal(spy_.illegal_calls, 0);
}

/* Every OTHER verification errno is a verdict, and a verdict blocks. The pair matters: one of them
 * must stay retryable and the other must not, and they are distinguished only by the errno. */
ZTEST(tof_package_l7, test_a_refused_verification_still_blocks)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.verified_rc = -EAGAIN;  /* refusal_errno(uncommitted): terminal, despite how it reads */

    zassert_equal(lc_.bring_up(), -EAGAIN);
    zassert_equal(lc_.current().current, pkg::state::blocked);

    spy_.verified_rc = 0;
    zassert_equal(lc_.bring_up(), -EPERM, "a verdict is not retried away");
}

/* ---- stop debt and bounded retries -------------------------------------------------------- */

/* A FAILED START OWES A STOP rather than blocking. The command may have gone out and the device
 * may or may not have acted on it, so landing in blocked would strand a possibly-ranging device
 * with no way to stop it. */
ZTEST(tof_package_l7, test_a_failed_start_owes_a_stop_rather_than_blocking)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.start_rc = -EIO;

    zassert_equal(lc_.bring_up(), -EIO);

    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::stop_owed);
    zassert_equal(s.failed_step, pkg::step::start);

    /* And the debt is payable. */
    spy_.stop_rc = 0;
    zassert_ok(lc_.stop());
    zassert_equal(lc_.current().current, pkg::state::idle);
}

/* READS AND REOPENS ARE REFUSED WHILE THE DEBT STANDS, and retrying the stop is the only way out. */
ZTEST(tof_package_l7, test_a_stop_debt_refuses_reads_and_reopens_but_not_another_stop)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());
    spy_.stop_rc = -EIO;

    zassert_equal(lc_.stop(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::stop_owed);

    bool fresh{true};
    zassert_equal(lc_.read_once(&fresh), -EPERM, "a session we cannot account for is not trusted");
    zassert_false(fresh);
    zassert_equal(lc_.bring_up(), -EPERM, "and reopening would forget a device that may be ranging");

    spy_.stop_rc = 0;
    zassert_ok(lc_.stop(), "the retry is the only exit, and it works");
    zassert_equal(lc_.current().current, pkg::state::idle);
}

/* THE RETRY BUDGET IS FINITE, and running it out is reported rather than silently treated as
 * stopped: a device that may still be measuring, that we have given up on stopping, is exactly
 * what an operator has to be told about. */
ZTEST(tof_package_l7, test_the_stop_budget_is_bounded_and_exhaustion_is_reported)
{
    pkg::config c{good_config()};
    c.stop_attempts = 2;
    zassert_ok(lc_.init(all_hooks(), c));
    zassert_ok(lc_.bring_up());
    spy_.stop_rc = -EIO;

    zassert_equal(lc_.stop(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::stop_owed, "one attempt left");

    zassert_equal(lc_.stop(), -EIO);
    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::blocked, "budget spent");
    zassert_equal(s.stop_attempts_used, 2);

    zassert_equal(lc_.stop(), -EPERM, "and no further attempts are made");
    zassert_equal(spy_.stop_calls, 2, "the budget counts attempts, not calls that were refused");
}

/* ---- a read error is not a stop ----------------------------------------------------------- */

/* The way out of a read error still goes through a stop the device acknowledges. Dropping to idle
 * here would let a later bring_up() start a second session on a device that never stopped. */
ZTEST(tof_package_l7, test_a_read_error_leaves_the_sensor_ranging)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());
    spy_.read_rc = -EIO;

    bool fresh{true};
    zassert_equal(lc_.read_once(&fresh), -EIO);
    zassert_false(fresh, "no sample is claimed");

    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::ranging, "still ranging as far as anyone knows");
    zassert_equal(s.failed_step, pkg::step::read);

    spy_.stop_rc = 0;
    zassert_ok(lc_.stop());
    zassert_equal(lc_.current().current, pkg::state::idle);
}

/* A not-ready poll is not an error and not a sample. */
ZTEST(tof_package_l7, test_a_not_ready_poll_is_neither_an_error_nor_a_sample)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());
    spy_.read_fresh = false;

    bool fresh{true};
    zassert_ok(lc_.read_once(&fresh));
    zassert_false(fresh);
    zassert_equal(lc_.current().current, pkg::state::ranging);
    zassert_equal(lc_.current().failed_step, pkg::step::none);
}

/* ---- arguments ---------------------------------------------------------------------------- */

/* EVERY HOOK IS REQUIRED. An absent one is not a feature a caller left out: with one device and no
 * fallback, a missing probe or stop removes the only check between an unknown device and an open. */
ZTEST(tof_package_l7, test_every_hook_is_required)
{
    const pkg::hooks full{all_hooks()};

    for (size_t i{0}; i < 9; ++i) {
        pkg::hooks h{full};
        switch (i) {
        case 0: h.ensure_verified = nullptr; break;
        case 1: h.probe = nullptr; break;
        case 2: h.quiesce = nullptr; break;
        case 3: h.close = nullptr; break;
        case 4: h.open = nullptr; break;
        case 5: h.configure = nullptr; break;
        case 6: h.start = nullptr; break;
        case 7: h.read = nullptr; break;
        case 8: h.stop = nullptr; break;
        }
        pkg::lifecycle fresh_lc{};
        zassert_equal(fresh_lc.init(h, good_config()), -EINVAL, "hook %zu is not optional", i);
    }
}

ZTEST(tof_package_l7, test_the_configuration_is_checked_rather_than_defaulted)
{
    pkg::lifecycle l{};

    pkg::config no_rate{good_config()};
    no_rate.frequency_hz = 0;
    zassert_equal(l.init(all_hooks(), no_rate), -EINVAL, "a forgotten rate is not a default");

    pkg::config too_fast{good_config()};
    too_fast.frequency_hz = 16;
    zassert_equal(l.init(all_hooks(), too_fast), -EINVAL);

    pkg::config bad_addr{good_config()};
    bad_addr.address_7bit = 0x80;
    zassert_equal(l.init(all_hooks(), bad_addr), -EINVAL);

    /* A budget of zero would make stop_owed inescapable, which is the one state this lifecycle
     * must always be able to leave. */
    pkg::config no_budget{good_config()};
    no_budget.stop_attempts = 0;
    zassert_equal(l.init(all_hooks(), no_budget), -EINVAL);
}

ZTEST(tof_package_l7, test_nothing_works_before_init)
{
    bool fresh{true};

    zassert_equal(lc_.bring_up(), -EPERM);
    zassert_equal(lc_.read_once(&fresh), -EPERM);
    zassert_equal(lc_.stop(), -EPERM);
    zassert_equal(spy_.probe_calls, 0);
    zassert_equal(spy_.open_calls, 0);
}

/* ---- the states that refuse a bring_up ---------------------------------------------------- */

ZTEST(tof_package_l7, test_bring_up_is_refused_unless_the_state_is_idle)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());

    zassert_equal(lc_.bring_up(), -EPERM, "ranging would stack a session");
    zassert_equal(spy_.open_calls, 1, "and nothing was opened by the refusal");

    /* blocked is waiting on a decision this module does not make, so it refuses too. */
    pkg::lifecycle other{};
    zassert_ok(other.init(all_hooks(), good_config()));
    spy_.probe_rc = -EIO;
    zassert_equal(other.bring_up(), -EIO);
    zassert_equal(other.current().current, pkg::state::blocked);
    spy_.probe_rc = 0;
    zassert_equal(other.bring_up(), -EPERM, "blocked is not cleared by trying again");
}

/* ---- the round boundary: which `up` may clear the step records ----------------------------- */

/* THE DEFECT REVIEW REPRODUCED ON 2026-10-13. A first `up` failed at open and recorded it; a second
 * `up` was refused outright -- the state was `blocked`, so bring_up() returned at its guard having
 * called nothing -- and yet the records came back empty, because the shell cleared them before
 * asking. The call that performed no operation destroyed the only evidence of the one that did. */
ZTEST(tof_package_l7, test_an_up_refused_from_blocked_keeps_the_previous_rounds_evidence)
{
    zassert_ok(lc_.init(recording_hooks(), good_config()));
    spy_.open_rc = -EIO;

    zassert_equal(an_up(), -EIO);
    zassert_equal(lc_.current().current, pkg::state::blocked);
    zassert_true(steps::last(steps::which::open).attempted);
    zassert_equal(steps::last(steps::which::open).rc, -EIO);

    zassert_false(lc_.can_begin_round(), "blocked is not a state a round may start from");
    zassert_equal(an_up(), -EPERM, "and the second up is refused");

    zassert_equal(spy_.open_calls, 1, "the refused call opened nothing");
    zassert_equal(spy_.verified_calls, 1, "and consulted nothing");
    const steps::record &r{steps::last(steps::which::open)};
    zassert_true(r.attempted, "so it must not have cleared the failure that is still the evidence");
    zassert_equal(r.rc, -EIO);
    zassert_equal(r.port_errno, -EIO);
    zassert_equal(r.failed_stage, tof_l7::stage::initialise);
}

/* THE SAME RULE FROM THE OTHER TWO REFUSING STATES. `ranging` and `stop_owed` reach the identical
 * guard, and a bench operator who runs `up` again out of habit must not pay for it with the record
 * of what is still running or still owed. */
ZTEST(tof_package_l7, test_an_up_refused_while_ranging_keeps_the_round_that_is_running)
{
    zassert_ok(lc_.init(recording_hooks(), good_config()));
    zassert_ok(an_up());
    zassert_equal(lc_.current().current, pkg::state::ranging);

    zassert_false(lc_.can_begin_round());
    zassert_equal(an_up(), -EPERM);

    zassert_equal(spy_.open_calls, 1, "no second session was stacked");
    zassert_true(steps::last(steps::which::start).attempted, "the running round is still readable");
    zassert_equal(steps::last(steps::which::start).rc, 0);
}

ZTEST(tof_package_l7, test_an_up_refused_while_a_stop_is_owed_keeps_the_failed_start)
{
    zassert_ok(lc_.init(recording_hooks(), good_config()));
    spy_.start_rc = -ETIMEDOUT;

    zassert_equal(an_up(), -ETIMEDOUT);
    zassert_equal(lc_.current().current, pkg::state::stop_owed);

    zassert_false(lc_.can_begin_round());
    zassert_equal(an_up(), -EPERM);

    zassert_equal(spy_.start_calls, 1);
    const steps::record &r{steps::last(steps::which::start)};
    zassert_true(r.attempted, "the start that owes the stop is the record that matters most here");
    zassert_equal(r.rc, -ETIMEDOUT);
}

/* AND AN `up` BEFORE init() IS THE FOURTH REFUSAL, reached by a different member and on the same
 * terms: nothing ran, so nothing is cleared. */
ZTEST(tof_package_l7, test_an_up_before_init_clears_nothing_either)
{
    steps::remember(steps::which::open, -ENODEV, tof_l7::operation_status{});

    zassert_false(lc_.can_begin_round(), "an uninitialised lifecycle accepts no round");
    zassert_equal(an_up(), -EPERM);

    zassert_true(steps::last(steps::which::open).attempted);
    zassert_equal(steps::last(steps::which::open).rc, -ENODEV);
}

/* THE OTHER HALF OF THE RULE: an accepted round must not inherit. Clearing only after a successful
 * bring-up would leave a failing round showing the previous round's later steps -- here, a round
 * that stops at the close would still report an open, a configure and a start that belong to the
 * run before it. */
ZTEST(tof_package_l7, test_an_accepted_round_shows_its_own_steps_and_not_the_last_rounds)
{
    zassert_ok(lc_.init(recording_hooks(), good_config()));
    zassert_ok(an_up());
    zassert_ok(lc_.stop());
    zassert_equal(lc_.current().current, pkg::state::idle);
    zassert_true(steps::last(steps::which::start).attempted, "the first round reached the start");

    spy_.close_rc = -EPERM;
    zassert_true(lc_.can_begin_round(), "idle accepts a round");
    zassert_equal(an_up(), -EPERM, "which then fails at its first step");

    zassert_true(steps::last(steps::which::close).attempted);
    zassert_equal(steps::last(steps::which::close).rc, -EPERM);
    zassert_false(steps::last(steps::which::open).attempted,
                  "the open belongs to the previous round and must not be read as this one's");
    zassert_false(steps::last(steps::which::configure).attempted);
    zassert_false(steps::last(steps::which::start).attempted);
}

/* THE PREDICATE IS THE GUARD, which is what keeps the two from drifting: the caller clears on the
 * same answer bring_up() refuses on. Checked by consequence rather than by reading the code -- a
 * false answer must be a call that reached no hook at all. */
ZTEST(tof_package_l7, test_can_begin_round_answers_exactly_what_bring_up_accepts)
{
    zassert_false(lc_.can_begin_round(), "before init");
    zassert_equal(lc_.bring_up(), -EPERM);
    zassert_equal(spy_.verified_calls, 0, "a refused round consults nothing");

    zassert_ok(lc_.init(recording_hooks(), good_config()));
    zassert_true(lc_.can_begin_round(), "idle");
    zassert_ok(lc_.bring_up());
    zassert_equal(spy_.verified_calls, 1, "an accepted round begins with the verification");

    zassert_false(lc_.can_begin_round(), "ranging");
    zassert_equal(lc_.bring_up(), -EPERM);
    zassert_equal(spy_.verified_calls, 1);

    zassert_ok(lc_.stop());
    zassert_true(lc_.can_begin_round(), "idle again after a confirmed stop");
}

/* A ROUND BEGINS WHEN IT IS ACCEPTED, NOT WHEN THE VERIFICATION PASSES, which is the boundary
 * review accepted on 2026-10-13 rather than pushing the clearing down into bring_up(). A round that
 * is accepted and then refused by an in-flight verification has begun: it consulted something, its
 * own steps are the truthful "none", and the state it leaves is still idle, so the next `up` runs.
 *
 * What it must NOT do is take the quiesce record with it. That record is a different module and a
 * different lifetime -- the observation the last window needed survives rounds. */
ZTEST(tof_package_l7, test_a_round_refused_by_an_in_flight_verification_still_counts_as_begun)
{
    zassert_ok(lc_.init(recording_hooks(), good_config()));

    /* A survivor refusal from an earlier run, recorded the way the bench records it. */
    quiesce::inputs observed{};
    observed.port_errno = 0;
    observed.completed = 511U;
    observed.uld_status = quiesce::kUldTimeout;
    quiesce::remember(observed, 5429600U, quiesce::go2_observation{});
    const quiesce::record kept{quiesce::last()};
    zassert_true(kept.taken);

    const unsigned rounds_before{steps::rounds_begun()};
    spy_.verified_rc = -EINPROGRESS;

    zassert_equal(an_up(), -EINPROGRESS);

    zassert_equal(steps::rounds_begun(), rounds_before + 1U,
                  "the round was accepted, so it is counted -- `up #N` names what the records below "
                  "belong to even when N reached no step");
    for (uint8_t i = 0; i < static_cast<uint8_t>(steps::which::count); ++i)
        zassert_false(steps::last(static_cast<steps::which>(i)).attempted,
                      "step %u: nothing ran, and `not called` is the truthful answer for THIS round",
                      i);

    const pkg::status s{lc_.current()};
    zassert_equal(s.current, pkg::state::idle,
                  "an in-flight verification is `ask again`, not a verdict, so nothing is blocked");
    zassert_equal(s.failed_step, pkg::step::firmware);
    zassert_equal(spy_.probe_calls, 0, "and the bus was not touched");
    zassert_true(lc_.can_begin_round(), "so the next up runs");

    const quiesce::record &now{quiesce::last()};
    zassert_true(now.taken, "the quiesce record is a different lifetime and must survive the round");
    zassert_equal(now.sequence, kept.sequence);
    zassert_equal(now.rc, kept.rc);
    zassert_equal(now.which, kept.which);
    zassert_equal(now.raw.completed, kept.raw.completed);
    zassert_equal(now.elapsed_us, kept.elapsed_us);
}
