/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The startup handshake, which is the part of the feeder that has no second chance.
 *
 * There is no handover from the interrupt in this image -- the timer callback compiles to a bare
 * return -- so the only thing standing between wdt_setup() and a board nobody is feeding is the
 * ORDER: the thread exists and is blocked before the watchdog is installed, it is released after,
 * it makes one real feed, and initialisation waits for that feed before it does anything else. Each
 * of those is asserted here against a fake watchdog that records what it was asked.
 *
 * The state machine the feeder drives is tested next door; this is about the wiring around it.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "tof_watchdog_feeder.hpp"

namespace
{

namespace feeder = lexxhard::tof_watchdog_feeder;

int feeds_seen_{0};
int last_channel_{-1};
int feed_rc_{0};

int fake_feed(const struct device *, int channel)
{
    last_channel_ = channel;
    ++feeds_seen_;
    return feed_rc_;
}

/* Enough of a watchdog for wdt_feed() to reach our function: it calls straight through the API
 * table, so nothing else in the driver needs to exist. */
const wdt_driver_api fake_api{
    .setup = nullptr,
    .disable = nullptr,
    .install_timeout = nullptr,
    .feed = fake_feed,
};

const struct device fake_wdt{
    .name = "fake-wdt",
    .config = nullptr,
    .api = &fake_api,
    .state = nullptr,
    .data = nullptr,
};

}  // namespace

ZTEST_SUITE(tof_watchdog_feeder, nullptr, nullptr, nullptr, nullptr, nullptr);

/* ONE ORDERED SCENARIO, and it has to be one function.
 *
 * What is under test is an order, and the module keeps a single set of state for the life of the
 * image exactly as it does on the board -- so it cannot be set up again per test. Splitting this
 * across three ZTESTs looked tidier and was wrong: ztest runs them alphabetically, so the "keeps
 * feeding" check ran before the release it depended on and failed for a reason that had nothing to
 * do with the feeder. */
ZTEST(tof_watchdog_feeder, test_the_startup_handshake_in_the_order_the_board_uses)
{
    /* The thread is created while there is no watchdog at all. Nothing may be fed yet: on the board
     * this is the interval in which wdt_setup() has not run. */
    zassert_equal(feeder::start(), 0, "if this fails the caller must not start the watchdog");
    k_msleep(50);
    zassert_equal(feeds_seen_, 0, "a feeder released too early would feed a watchdog that is not there");

    /* Initialisation asking for the first feed before releasing must not block forever, and must not
     * claim success. */
    const int64_t began{k_uptime_get()};
    zassert_equal(feeder::wait_first_feed(), -ETIMEDOUT, "nothing has been released yet");
    const int64_t waited{k_uptime_get() - began};
    zassert_true(waited < 10000,
                 "the wait must be a fraction of the 10 s watchdog window, not comparable to it");

    /* A board with no ToF chain has reached no baseline, so nothing is judged and nothing stops --
     * the same behaviour the old unconditional feed had, from a thread instead of an interrupt. */
    zassert_equal(feeder::current().phase, 0U, "still waiting for a baseline");

    /* The watchdog exists now. Publishing it releases the feeder. */
    feeder::release(&fake_wdt, 3);
    k_msleep(50);
    zassert_true(feeds_seen_ > 0, "the feeder should have fed as soon as it was released");
    zassert_equal(last_channel_, 3, "the channel wdt_install_timeout returned, not a guess");

    /* And the feed it made is reported, so initialisation can proceed knowing the watchdog is being
     * served. */
    zassert_equal(feeder::wait_first_feed(), 0);

    /* Feeding continues on its own. A handshake that worked once and then stopped would be the same
     * reset, later. */
    const int after_handshake{feeds_seen_};
    k_msleep(1200);
    zassert_true(feeds_seen_ >= after_handshake + 2, "at least two more feeds in a 1.2 s window");

    const feeder::status st{feeder::current()};
    zassert_true(st.feeds > 0);
    zassert_false(st.withheld, "nothing here is a reason to stop");
    zassert_equal(st.why, 0U);
}
