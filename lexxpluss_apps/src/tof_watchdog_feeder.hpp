/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The one thread that feeds the IWDG, and the startup handshake that makes it the only one.
 *
 * WHAT CHANGES. The hardware watchdog was fed from a k_timer expiry, which on Zephyr runs in
 * interrupt context, unconditionally. An interrupt that can feed is an interrupt that keeps a hung
 * system alive: every thread-level hang this project has hit left the timer running and the board
 * fed, doing nothing, for as long as anybody was willing to wait. Feeding moves to a thread that
 * feeds only while the watched work keeps finishing.
 *
 * THERE IS NO HANDOVER. An earlier plan kept the interrupt feeding until the thread had proved
 * itself, which cannot coexist with the property that matters -- exactly one wdt_feed() call site,
 * in a thread, and none reachable from an interrupt. So this image never feeds from the timer at
 * all, and instead the START-UP ORDER closes the window that a handover was invented for:
 *
 *   the feeder thread is created FIRST and blocks on a semaphore, before the watchdog exists;
 *   the watchdog is installed and started;
 *   the device and channel are published and the feeder released;
 *   the feeder issues the first real wdt_feed() and, only if it returned 0, reports it;
 *   initialisation waits for that report before it does anything else.
 *
 * Nothing between wdt_setup() and the first feed can block, because nothing runs there: the caller
 * is waiting on a semaphore the feeder gives. And if the thread cannot be created, the watchdog is
 * never started -- an IWDG with nobody to feed it is a guaranteed reset, so the failure is to leave
 * it off and say so rather than to arm something that will certainly fire.
 *
 * THE WAIT IS BOUNDED WELL INSIDE THE WATCHDOG. The IWDG window is 10,000 ms; initialisation waits
 * 2,000 ms for the first feed. If that expires something is wrong in a way this cannot fix, and the
 * honest outcome is a log line and a boot that continues toward its own reset rather than a silent
 * hang inside the init path.
 *
 * PRIORITY, AND WHY IT IS NOT THE HIGHEST. Too low and a healthy board under load starves the
 * feeder itself, and the watchdog resets a machine that was working -- the worst failure this
 * subsystem has. So it sits above the acquisition thread. It does not sit at the top: it is
 * preemptible and spends almost all of its time asleep, so it cannot itself be the reason a watched
 * thread does not run, and starvation of those threads still shows up as their heartbeats stopping,
 * which is precisely what it is looking for.
 *
 * THE TOMBSTONE IS WRITTEN BEFORE THE FEEDING STOPS. On the one transition from armed to stopped
 * the record is committed first and the feed withheld second. The other order would leave a reset
 * whose cause was still being written when the board went down.
 */

#pragma once

#include <stdint.h>

struct device;

namespace lexxhard::tof_watchdog_feeder {

/* Creates the thread and leaves it blocked. Call BEFORE the watchdog is installed: a non-zero
 * return means the caller must not start the watchdog at all. */
int start();

/* Publishes the watchdog and releases the feeder. Call after wdt_setup() has succeeded. */
void release(const struct device *wdt, int channel);

/* Blocks until the feeder has made one successful wdt_feed(). Returns 0, or -ETIMEDOUT after a
 * bound that is a fraction of the watchdog window. */
int wait_first_feed(void);

/* From the ToF chain. `baseline_point` goes true once automatic commissioning has finished and
 * before the first L7 open; `l7_expected` says this image has a grid sensor to watch at all. */
void set_baseline_point(bool ready);
void set_l7_expected(bool expected);

/* A declared long operation -- an ULD download, a commissioning pass. Suspends the bounds of the
 * chain work it holds and nothing else, and is itself bounded. */
void long_operation_begin();
void long_operation_end();

/* Reads the tombstone left by a previous boot and logs it. Safe to call before anything else. When
 * the region holds no valid record it logs one INFO line naming why -- an ordinary boot and a boot
 * after a cleared battery both read that way, and which of the two it was is worth having. */
void report_previous_stop();

/* For the shell and for tests: what the feeder decided, and why. */
struct status {
    uint32_t phase{0};
    uint32_t why{0};
    uint32_t feeds{0};
    bool withheld{false};
};
status current();

}  // namespace lexxhard::tof_watchdog_feeder
