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

#include "tof_task_watchdog.hpp"
#include "tof_watchdog_feeder.hpp"
#include "tof_watchdog_tombstone.hpp"

namespace
{

namespace feeder = lexxhard::tof_watchdog_feeder;
namespace tomb = lexxhard::tof_watchdog_tombstone;

int feeds_seen_{0};
int last_channel_{-1};
int feed_rc_{0};
/* Deterministic failures: a count rather than a duration, because a test that failed "for about a
 * second" would depend on how many 500 ms periods fitted into it. */
int fail_remaining_{0};
/* How many refusals the driver has actually returned. Counted separately from attempts because the
 * question that matters -- did the consecutive count reset after a success -- is answered by how
 * many refusals it took to give up, and that number cannot race: a feed slipping in before the
 * failures are enabled is a success, which this does not count. */
int failures_returned_{0};

int fake_feed(const struct device *, int channel)
{
    last_channel_ = channel;
    ++feeds_seen_;
    if (fail_remaining_ > 0) {
        --fail_remaining_;
        ++failures_returned_;
        return -EIO;
    }
    if (feed_rc_ != 0)
        ++failures_returned_;
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

    /* A REFUSAL THAT PASSES. Two in a row and then a success: the count resets and the board keeps
     * being fed, because a transient refusal is not a reason to end a boot. */
    const int before_transient{feeds_seen_};
    fail_remaining_ = 2;
    k_msleep(1600);
    zassert_true(feeds_seen_ >= before_transient + 3, "two refusals and at least one success");
    zassert_false(feeder::current().withheld, "two refusals in a row must not latch");
    zassert_equal(feeder::current().why, 0U);

    const uint32_t feeds_before_giving_up{feeder::current().feeds};
    zassert_true(feeds_before_giving_up > 0);

    /* A REFUSAL THAT DOES NOT, and the count of refusals is the assertion.
     *
     * If the success above had not reset the consecutive count, the two earlier refusals would
     * still be on it and the feeder would give up after ONE more. Asserting only that it eventually
     * stops passes either way; asserting that it took exactly three is what pins the reset. */
    failures_returned_ = 0;
    feed_rc_ = -EIO;
    k_msleep(2500);
    zassert_equal(failures_returned_, 3,
                  "three refusals, not one: the success above must have reset the count");

    const feeder::status gone{feeder::current()};
    zassert_true(gone.withheld, "the phase must be stopped, not merely the reason set");
    zassert_equal(gone.why, static_cast<uint32_t>(
                                lexxhard::tof_task_watchdog::feed_api_failed));
    zassert_equal(gone.feeds, feeds_before_giving_up, "a refused feed is not a feed");

    /* And what it left behind. */
    tomb::record rec{};
    zassert_equal(feeder::read_record(rec), tomb::status::valid,
                  "a refused feed ends in the same reset and must leave the same record");
    zassert_equal(rec.reason, static_cast<uint32_t>(
                                  lexxhard::tof_task_watchdog::feed_api_failed));
    zassert_equal(rec.feed_rc, -EIO, "the driver's own rc, not a generic code");
    zassert_true(rec.last_fed_ms > 0, "there were successful feeds before this");
    zassert_true(rec.stopped_ms > rec.last_fed_ms,
                 "the stop is strictly after the last feed -- three refused periods apart");
    zassert_equal(rec.phase, static_cast<uint32_t>(lexxhard::tof_task_watchdog::phase::stopped));

    /* It latches. Whatever the driver does afterwards, this boot is over. */
    const int after_stop{feeds_seen_};
    feed_rc_ = 0;
    k_msleep(1500);
    zassert_equal(feeds_seen_, after_stop, "the reset is the point");
}
