/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * THE BENCH IMAGE'S WAY BACK TO PRODUCTION, and the reason it is not optional.
 *
 * A Package bench image is delivered as a padded TEST image over the CAN DFU and is never
 * confirmed. MCUboot reverts an unconfirmed image at the next reset, so the whole guarantee rests on
 * something producing that reset. Two cases produce one by themselves: a crash early in boot, and
 * any fault that stops board_controller feeding the independent watchdog. A third does not -- a
 * build that wedges its own subsystems while board_controller keeps feeding -- and that is exactly
 * what the L7 bring-up hit on dasher2: every CAN stream and the shell stopped, no fatal, no reset,
 * and getting the production image back took someone power-cycling the machine.
 *
 * WHY THIS FILE EXISTS SEPARATELY FROM THE DIAG IMAGE'S VERSION. The diag image carries the same
 * timer, but it is on another branch and its arming sits inside a chain build's confirm block. A
 * bench image cannot inherit that protection by being similar to it; nothing in a
 * ENABLE_TOF_PACKAGE_BENCH build armed anything, which left an unconfirmed image running until a
 * human noticed.
 *
 * TWO DELIBERATE DIFFERENCES FROM THE DIAG IMAGE:
 *
 *   - THERE IS NO CONFIRM COMMAND ANYWHERE IN A BENCH IMAGE. The diag image could be made permanent
 *     from its shell. This one cannot, by construction, so the only two outcomes are a revert on the
 *     deadman and a revert on any other reset. An acceptance run has no reason to make a bench image
 *     permanent, and the command would be the one way the guarantee could be spent by accident.
 *   - AN EXTENSION IS BOUNDED AND MANUAL. `pkg_tof hold` re-arms the window, at most
 *     kMaxExtensions times. Nothing re-arms it automatically: a timer that renews itself on
 *     activity is a timer that never fires during a session that went wrong slowly.
 *
 * THE UNATTENDED RUNTIME ARMS THIS TOO, under ENABLE_TOF_PACKAGE_AUTO_DEADMAN, and gets a
 * STRICTLY SHORTER leash than the bench does: that image registers no `pkg_tof` command group, so
 * `hold` does not exist in it and the window cannot be extended at all. One flash buys
 * kWindowMinutes of running and no more. It still carries the ordinary Zephyr shell -- CONFIG_SHELL
 * is set for every image this project builds -- so this is a statement about which commands exist,
 * not about whether a console does.
 *
 * WHY AN IMAGE MEANT TO STAY UP ARMS A TIMER THAT REBOOTS IT. Being unconfirmed is not by itself a
 * protection: a revert only happens once something RESETS the board, and the hazard this runtime
 * adds is a Package thread that wedges while the board controller keeps feeding the watchdog --
 * exactly the dasher2 case above, with nothing left to produce the reset. So the first acceptance
 * of a candidate keeps the timer, and the configuration that is meant to stay resident turns it
 * off deliberately, once the image has been watched. The default is therefore ON: forgetting the
 * flag leaves a machine protected rather than stranded.
 *
 * THE REBOOT IS A WHOLE-MACHINE POWER CYCLE, not an SCB-only reset: the SCB owns the main power
 * rails, so a cold reboot drops everything and the machine comes back in about 80 s. That is the
 * safe direction -- it costs a re-flash, while not rebooting leaves a machine running an image
 * nobody is watching -- but it is not quiet, and an operator has to be told it is coming.
 */

#pragma once

#if defined(ENABLE_TOF_PACKAGE_BENCH) || defined(ENABLE_TOF_PACKAGE_AUTO_DEADMAN)

#include <stdint.h>

namespace lexxhard::tof_package_bench_deadman {

/* The window an acceptance run gets, and the most it can be extended to. 10 minutes covers the
 * scripted sequence (status, bind, up, read, stop, and a second up/read/stop) several times over;
 * two extensions bound the worst case at 30 minutes from arming, after which the only way to keep
 * this image running is to re-flash it, which is a decision someone has to make again. */
inline constexpr uint32_t kWindowMinutes{10};
inline constexpr uint32_t kMaxExtensions{2};

/* Armed once every subsystem thread is up, so the window belongs to the operator rather than to
 * boot. Does nothing when the image is already confirmed: an unconditional timer in a confirmed
 * image is a reboot loop, not a protection. */
void arm(void);

/* Milliseconds until the cold reboot, or 0 when nothing is armed -- which in an unconfirmed image
 * means the protection is absent and only a manual power cycle gets production back. */
uint32_t remaining_ms(void);

bool armed(void);
bool image_confirmed(void);

/* Re-arms for another `minutes`, counted from now. Returns -EPERM once kMaxExtensions are spent,
 * -EINVAL for a window outside 1..kWindowMinutes, and -EALREADY when the image is confirmed and
 * there is nothing to protect. */
int extend(uint32_t minutes);

uint32_t extensions_used(void);

}  // namespace lexxhard::tof_package_bench_deadman

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO_DEADMAN
