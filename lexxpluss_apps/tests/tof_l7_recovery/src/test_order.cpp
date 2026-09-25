/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The boot order, which is the one part of this work whose content IS the order.
 *
 * A surviving L7 can only be told anything while it is still enabled, and the first
 * gpio_pin_configure_dt on the data line ends that: the chain goes to a known state, the sensor
 * goes silent, and it keeps every bit of the state that made recovery necessary while losing the
 * only channel that could clear it. So recovery before the first control-line change is not a
 * preference; it is the difference between a recovery that can run and one that cannot, and that
 * is the sort of constraint which survives in a comment for a while and then quietly stops being
 * true.
 *
 * The other half of the design is what is allowed to stop a boot. The two pin configurations are --
 * a chain whose control lines will not configure has nothing to enumerate. Nothing on the recovery
 * side is, because enumeration owns the verdict on chain health and a second opinion from here is a
 * mistake this project has already paid for.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_l7_boot_order.hpp"

namespace
{

namespace ord = lexxhard::tof_l7_boot_order;

constexpr size_t kMaxLog{8};

struct call_log {
    ord::step at[kMaxLog]{};
    size_t n{0};

    void note(ord::step s)
    {
        if (n < kMaxLog)
            at[n] = s;
        ++n;
    }

    /* Position in the log, or kMaxLog when the step never ran. */
    size_t index_of(ord::step s) const
    {
        for (size_t i{0}; i < n && i < kMaxLog; ++i)
            if (at[i] == s)
                return i;
        return kMaxLog;
    }

    bool ran(ord::step s) const { return index_of(s) != kMaxLog; }
};

struct script {
    call_log log{};
    int set_speed_rc{0};
    int readback_rc{0};
    bool readback_matches{true};
    int recovery_rc{0};
    int restore_rc{0};
    int data_pin_rc{0};
    int clock_pin_rc{0};
};

script sc_{};

int fake_set_speed(void *)      { sc_.log.note(ord::step::set_recovery_speed); return sc_.set_speed_rc; }
int fake_recover(void *)        { sc_.log.note(ord::step::recover); return sc_.recovery_rc; }
int fake_restore(void *)        { sc_.log.note(ord::step::restore_product_speed); return sc_.restore_rc; }
int fake_data_pin(void *)       { sc_.log.note(ord::step::configure_data_pin); return sc_.data_pin_rc; }
int fake_clock_pin(void *)      { sc_.log.note(ord::step::configure_clock_pin); return sc_.clock_pin_rc; }

int fake_readback(void *, bool *matches)
{
    sc_.log.note(ord::step::read_back_speed);
    if (matches != nullptr)
        *matches = sc_.readback_matches;
    return sc_.readback_rc;
}

ord::steps wired()
{
    ord::steps s{};
    s.set_recovery_speed = fake_set_speed;
    s.read_back_speed = fake_readback;
    s.recover_survivors = fake_recover;
    s.restore_product_speed = fake_restore;
    s.configure_data_pin = fake_data_pin;
    s.configure_clock_pin = fake_clock_pin;
    return s;
}

void reset_script() { sc_ = script{}; }

}  // namespace

ZTEST_SUITE(tof_l7_boot_order, nullptr, nullptr, nullptr, nullptr, nullptr);

/* The assertion this file exists for. Everything else here protects it. */
ZTEST(tof_l7_boot_order, test_recovery_runs_strictly_before_the_first_control_line_change)
{
    reset_script();
    const ord::report r{ord::run(wired())};

    zassert_true(sc_.log.ran(ord::step::recover));
    zassert_true(sc_.log.index_of(ord::step::recover) <
                     sc_.log.index_of(ord::step::configure_data_pin),
                 "after the data pin is driven the survivor is silent and can no longer be stopped");
    zassert_true(sc_.log.index_of(ord::step::recover) <
                 sc_.log.index_of(ord::step::configure_clock_pin));
    zassert_true(r.pins_configured);
    zassert_equal(r.rc, 0);
}

/* The whole sequence, written out rather than derived, so a reordering has to disagree with a list
 * somebody has to read. */
ZTEST(tof_l7_boot_order, test_the_happy_path_runs_every_step_in_one_fixed_order)
{
    reset_script();
    const ord::report r{ord::run(wired())};

    zassert_equal(sc_.log.n, 6U);
    zassert_equal(sc_.log.at[0], ord::step::set_recovery_speed);
    zassert_equal(sc_.log.at[1], ord::step::read_back_speed);
    zassert_equal(sc_.log.at[2], ord::step::recover);
    zassert_equal(sc_.log.at[3], ord::step::restore_product_speed);
    zassert_equal(sc_.log.at[4], ord::step::configure_data_pin);
    zassert_equal(sc_.log.at[5], ord::step::configure_clock_pin);
    zassert_true(r.speed_set);
    zassert_true(r.speed_readback_ok);
    zassert_true(r.recovery_ran);
    zassert_true(r.product_speed_restored);
}

/* A speed that will not set means the recovery traffic would go out at whatever the devicetree left
 * behind, which is the product speed chosen for a schedule rather than for a device in an unknown
 * state. Skip the recovery, keep the boot. */
ZTEST(tof_l7_boot_order, test_a_failed_speed_change_skips_recovery_and_still_brings_the_chain_up)
{
    reset_script();
    sc_.set_speed_rc = -EIO;

    const ord::report r{ord::run(wired())};

    zassert_false(r.speed_set);
    zassert_false(r.recovery_ran);
    zassert_false(sc_.log.ran(ord::step::recover));
    zassert_true(r.pins_configured, "an optional step failing must not cost the chain its pins");
    zassert_equal(r.rc, 0);
}

/* A configure that silently did nothing is the case the readback exists for, and it is not
 * hypothetical enough to leave untested. */
ZTEST(tof_l7_boot_order, test_a_readback_that_disagrees_is_treated_as_a_failed_speed_change)
{
    reset_script();
    sc_.readback_matches = false;

    const ord::report r{ord::run(wired())};

    zassert_true(r.speed_set);
    zassert_false(r.speed_readback_ok);
    zassert_false(r.recovery_ran);
    zassert_true(r.pins_configured);

    reset_script();
    sc_.readback_rc = -EIO;
    const ord::report r2{ord::run(wired())};
    zassert_false(r2.speed_readback_ok, "a readback that could not be made is not a readback that agreed");
    zassert_false(r2.recovery_ran);
}

/* The speed was changed, so it is restored -- whether or not the thing it was changed for worked.
 * Everything downstream was promised the devicetree's speed. */
ZTEST(tof_l7_boot_order, test_the_product_speed_is_restored_even_when_recovery_did_not_help)
{
    reset_script();
    sc_.readback_matches = false;

    const ord::report r{ord::run(wired())};

    zassert_false(r.recovery_ran);
    zassert_true(r.product_speed_restored);
    zassert_true(sc_.log.index_of(ord::step::restore_product_speed) <
                 sc_.log.index_of(ord::step::configure_data_pin));
}

/* A survivor that would not stop is reported and nothing more. Enumeration is what decides whether
 * the chain can be used, and it is about to run. */
ZTEST(tof_l7_boot_order, test_a_recovery_failure_is_recorded_and_is_not_fatal)
{
    reset_script();
    sc_.recovery_rc = -ETIMEDOUT;

    const ord::report r{ord::run(wired())};

    zassert_true(r.recovery_ran);
    zassert_equal(r.recovery_rc, -ETIMEDOUT);
    zassert_equal(r.rc, 0, "the boot is not this pass's to stop");
    zassert_true(r.pins_configured);
}

/* The one genuine refusal. A chain whose data line will not configure has nothing to enumerate, and
 * the clock line is not touched afterwards. */
ZTEST(tof_l7_boot_order, test_a_pin_that_will_not_configure_ends_the_sequence)
{
    reset_script();
    sc_.data_pin_rc = -ENODEV;

    const ord::report r{ord::run(wired())};

    zassert_equal(r.rc, -ENODEV);
    zassert_equal(r.failed_at, ord::step::configure_data_pin);
    zassert_false(r.pins_configured);
    zassert_false(sc_.log.ran(ord::step::configure_clock_pin));

    reset_script();
    sc_.clock_pin_rc = -ENODEV;
    const ord::report r2{ord::run(wired())};
    zassert_equal(r2.failed_at, ord::step::configure_clock_pin);
    zassert_false(r2.pins_configured);
}

/* An image built without the L7 ULD supplies no recovery steps at all, and must still reach its
 * pins -- the four cliff sensors do not care that the grid path is absent. */
ZTEST(tof_l7_boot_order, test_an_image_with_no_recovery_still_configures_its_pins)
{
    reset_script();
    ord::steps s{};
    s.configure_data_pin = fake_data_pin;
    s.configure_clock_pin = fake_clock_pin;

    const ord::report r{ord::run(s)};

    zassert_true(r.pins_configured);
    zassert_false(r.recovery_ran);
    zassert_false(r.speed_set);
    zassert_equal(sc_.log.n, 2U);
    zassert_equal(r.rc, 0);
}

ZTEST(tof_l7_boot_order, test_every_step_has_its_own_name)
{
    zassert_true(strcmp(ord::step_name(ord::step::none), "none") == 0);
    zassert_true(strcmp(ord::step_name(ord::step::set_recovery_speed), "set_recovery_speed") == 0);
    zassert_true(strcmp(ord::step_name(ord::step::read_back_speed), "read_back_speed") == 0);
    zassert_true(strcmp(ord::step_name(ord::step::recover), "recover") == 0);
    zassert_true(strcmp(ord::step_name(ord::step::restore_product_speed), "restore_product_speed") == 0);
    zassert_true(strcmp(ord::step_name(ord::step::configure_data_pin), "configure_data_pin") == 0);
    zassert_true(strcmp(ord::step_name(ord::step::configure_clock_pin), "configure_clock_pin") == 0);
}
