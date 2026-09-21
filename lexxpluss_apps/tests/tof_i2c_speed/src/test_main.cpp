/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host tests for the bus-speed resolver.
 *
 * WHY THIS SUITE EXISTS AT ALL. The decision it covers used to be two lines inside
 * tof_chain_controller.cpp -- a file that needs the shell, the devicetree and a real I2C
 * controller, so no host suite links it and nothing inside it can be checked. That is the same
 * shape as the defect found on dasher2 on 2026-09-21, where the commissioning transaction was
 * configured only from a shell command and the automatic path therefore refused every request.
 *
 * WHAT IT IS GUARDING. Commissioning proves the chain at one speed and then switches to the speed
 * production reads at, re-verifying every position's identity before any mapping is committed. That
 * re-check is worth nothing if the switch did not happen: a controller that cannot do 1 MHz and
 * quietly stays at 400 kHz would pass the re-check at 400 kHz, publish PROVEN, and be read at a
 * speed nothing ever verified. So "refuse rather than fall back" is a safety property, not
 * tidiness, and the cases below are written so that hardcoding a speed fails them.
 */

#include <errno.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/ztest.h>

#include "tof_i2c_speed.hpp"

namespace spd = lexxhard::tof_i2c_speed;
namespace cm = lexxhard::tof_commissioning;

ZTEST_SUITE(tof_i2c_speed, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_i2c_speed, test_the_three_characterised_frequencies_have_controller_settings)
{
    const spd::resolution standard{spd::resolve_hz(100000)};
    zassert_true(standard.ok(), "100 kHz was refused (%d)", standard.rc);
    zassert_equal(standard.code, I2C_SPEED_STANDARD);
    zassert_false(standard.fast_mode_plus);

    const spd::resolution fast{spd::resolve_hz(400000)};
    zassert_true(fast.ok(), "400 kHz was refused (%d)", fast.rc);
    zassert_equal(fast.code, I2C_SPEED_FAST);
    zassert_false(fast.fast_mode_plus);

    /* 1 MHz is Fast-mode Plus, and naming it here is the point: "1 MHz" and "the code Zephyr wants
     * for 1 MHz" are different facts, and only the first one is in the overlay. */
    const spd::resolution fast_plus{spd::resolve_hz(1000000)};
    zassert_true(fast_plus.ok(), "1 MHz was refused (%d)", fast_plus.rc);
    zassert_equal(fast_plus.code, I2C_SPEED_FAST_PLUS,
                  "1 MHz did not resolve to Fast-mode Plus");
    zassert_true(fast_plus.fast_mode_plus,
                 "1 MHz did not request the STM32 Fast-mode Plus drive capability");
}

ZTEST(tof_i2c_speed, test_an_uncharacterised_frequency_is_refused_and_never_rounded)
{
    /* Rounding is the dangerous answer, not an error: a value nobody characterised on this harness
     * would become a bus speed by being close to one that was. Each of these is near a supported
     * value on purpose. */
    const uint32_t uncharacterised[]{0, 1, 50000, 99999, 100001, 399999, 400001, 999999, 1000001,
                                     3400000, 0xFFFFFFFFU};
    for (const uint32_t hz : uncharacterised) {
        const spd::resolution r{spd::resolve_hz(hz)};
        zassert_false(r.ok(), "%u Hz was accepted", hz);
        zassert_equal(r.rc, -ENOTSUP, "%u Hz was refused with %d, not -ENOTSUP", hz, r.rc);
    }
}

/* THE CASE THAT KILLS A HARDCODED PRODUCT SPEED. The product role must return whatever the product
 * frequency says, so an implementation that answers I2C_SPEED_FAST regardless -- which is what this
 * code did before the frequencies became configuration -- fails here rather than on a robot. */
ZTEST(tof_i2c_speed, test_each_role_resolves_its_own_configured_frequency)
{
    const spd::resolution proof{spd::resolve_role(cm::bus_speed::proof, 100000, 1000000)};
    zassert_true(proof.ok());
    zassert_equal(proof.code, I2C_SPEED_STANDARD, "the proof role did not take the proof frequency");

    const spd::resolution product{spd::resolve_role(cm::bus_speed::product, 100000, 1000000)};
    zassert_true(product.ok());
    zassert_equal(product.code, I2C_SPEED_FAST_PLUS,
                  "the product role did not take the configured product frequency");
    zassert_not_equal(product.code, I2C_SPEED_FAST,
                      "the product role answered 400 kHz while configured for 1 MHz");

    /* And the other way round, so the test cannot be satisfied by hardcoding Fast-mode Plus either.
     * The same call with the frequencies swapped must swap the answers. */
    const spd::resolution swapped_proof{spd::resolve_role(cm::bus_speed::proof, 1000000, 400000)};
    const spd::resolution swapped_product{spd::resolve_role(cm::bus_speed::product, 1000000, 400000)};
    zassert_equal(swapped_proof.code, I2C_SPEED_FAST_PLUS);
    zassert_equal(swapped_product.code, I2C_SPEED_FAST);
}

ZTEST(tof_i2c_speed, test_a_role_configured_with_an_impossible_frequency_is_refused)
{
    /* A misconfigured overlay must stop commissioning, not start it at whatever the bus happened to
     * be left at. Both roles, because both are configuration. */
    zassert_equal(spd::resolve_role(cm::bus_speed::product, 100000, 250000).rc, -ENOTSUP);
    zassert_equal(spd::resolve_role(cm::bus_speed::proof, 250000, 1000000).rc, -ENOTSUP);
}

/* THE APPLIED STATE. i2c_configure() returning 0 does not prove the requested electrical mode.
 * The STM32 driver reports a cached configuration through i2c_get_config(), so the target binding
 * separately checks TIMINGR and the SYSCFG Fast-mode Plus drive bit before it asks this helper to
 * reject any cross-field mismatch. */
ZTEST(tof_i2c_speed, test_a_controller_that_did_not_take_the_speed_or_drive_state_is_refused)
{
    const spd::resolution fmplus{spd::resolve_hz(1000000)};
    const spd::resolution fast{spd::resolve_hz(400000)};
    const spd::resolution standard{spd::resolve_hz(100000)};

    zassert_equal(spd::verify_applied(fmplus, I2C_SPEED_FAST_PLUS, true), 0,
                  "a controller that took the speed was refused anyway");

    /* The exact field failure: asked for 1 MHz, the controller is running 400 kHz. Accepting this
     * is how a mapping gets proven at one speed and read at another. */
    zassert_equal(spd::verify_applied(fmplus, I2C_SPEED_FAST, true), -ENOTSUP,
                  "a silent fall-back to 400 kHz was accepted");
    zassert_equal(spd::verify_applied(fmplus, I2C_SPEED_STANDARD, true), -ENOTSUP);
    zassert_equal(spd::verify_applied(standard, I2C_SPEED_FAST, false), -ENOTSUP);

    /* TIMINGR alone is not Fast-mode Plus on STM32F769. Conversely the stronger drive must not
     * remain silently enabled after commissioning restores the 100 kHz proof speed. */
    zassert_equal(spd::verify_applied(fmplus, I2C_SPEED_FAST_PLUS, false), -ENOTSUP,
                  "1 MHz without Fast-mode Plus drive was accepted");
    zassert_equal(spd::verify_applied(fast, I2C_SPEED_FAST, true), -ENOTSUP,
                  "Fast-mode Plus drive remained enabled at 400 kHz");
    zassert_equal(spd::verify_applied(standard, I2C_SPEED_STANDARD, true), -ENOTSUP,
                  "Fast-mode Plus drive remained enabled at the proof speed");
}
