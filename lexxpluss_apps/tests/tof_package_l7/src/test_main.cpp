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

#include "tof_package_l7.hpp"

namespace pkg = lexxhard::tof_package;

namespace {

struct spy {
    int verified_rc{0};
    int probe_rc{0};
    bool probe_present{false};
    int quiesce_rc{0};
    int open_rc{0};
    int configure_rc{0};
    int start_rc{0};
    int read_rc{0};
    bool read_fresh{true};
    int stop_rc{0};

    int verified_calls{0};
    int probe_calls{0};
    int quiesce_calls{0};
    int open_calls{0};
    int configure_calls{0};
    int start_calls{0};
    int read_calls{0};
    int stop_calls{0};

    uint8_t probed_addr{0};
    uint8_t opened_addr{0};
    uint8_t configured_hz{0};
};

spy spy_;

int fake_verified()
{
    ++spy_.verified_calls;
    return spy_.verified_rc;
}

int fake_probe(void *, uint8_t addr, bool *present)
{
    ++spy_.probe_calls;
    spy_.probed_addr = addr;
    /* Deliberately writes `present` even on a failure, because a port that leaves a stale value
     * behind is exactly what the lifecycle must not rely on. */
    *present = spy_.probe_present;
    return spy_.probe_rc;
}

int fake_quiesce(void *, uint8_t)
{
    ++spy_.quiesce_calls;
    return spy_.quiesce_rc;
}

int fake_open(void *, uint8_t addr)
{
    ++spy_.open_calls;
    spy_.opened_addr = addr;
    return spy_.open_rc;
}

int fake_configure(void *, uint8_t hz)
{
    ++spy_.configure_calls;
    spy_.configured_hz = hz;
    return spy_.configure_rc;
}

int fake_start(void *)
{
    ++spy_.start_calls;
    return spy_.start_rc;
}

int fake_read(void *, bool *fresh)
{
    ++spy_.read_calls;
    *fresh = spy_.read_fresh;
    return spy_.read_rc;
}

int fake_stop(void *)
{
    ++spy_.stop_calls;
    return spy_.stop_rc;
}

pkg::hooks all_hooks()
{
    pkg::hooks h{};
    h.ensure_verified = fake_verified;
    h.probe = fake_probe;
    h.quiesce = fake_quiesce;
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

void before(void *)
{
    spy_ = spy{};
    lc_ = pkg::lifecycle{};
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
}

ZTEST(tof_package_l7, test_a_full_cycle_returns_to_idle_and_can_run_again)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    zassert_ok(lc_.bring_up());

    bool fresh{false};
    zassert_ok(lc_.read_once(&fresh));
    zassert_true(fresh);

    zassert_ok(lc_.stop());
    zassert_equal(lc_.current().current, pkg::state::idle);

    zassert_ok(lc_.bring_up(), "idle is the state an open may start from");
    zassert_equal(spy_.open_calls, 2);
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

/* A verification still in flight is not a refusal, but it is not a go-ahead either: the run stops
 * and the errno says to ask again rather than naming a defect. */
ZTEST(tof_package_l7, test_a_verification_in_flight_stops_the_run_without_calling_it_a_defect)
{
    zassert_ok(lc_.init(all_hooks(), good_config()));
    spy_.verified_rc = -EINPROGRESS;

    zassert_equal(lc_.bring_up(), -EINPROGRESS);
    zassert_equal(spy_.probe_calls, 0);
    zassert_equal(lc_.current().errno_value, -EINPROGRESS,
                  "distinguishable from a terminal refusal by the caller");
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

    for (size_t i{0}; i < 8; ++i) {
        pkg::hooks h{full};
        switch (i) {
        case 0: h.ensure_verified = nullptr; break;
        case 1: h.probe = nullptr; break;
        case 2: h.quiesce = nullptr; break;
        case 3: h.open = nullptr; break;
        case 4: h.configure = nullptr; break;
        case 5: h.start = nullptr; break;
        case 6: h.read = nullptr; break;
        case 7: h.stop = nullptr; break;
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
