/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * An unconfirmed test image reverts only after reset. The timer supplies that
 * reset even if a Package thread hangs while the board watchdog keeps feeding.
 * Bench builds allow two manual `pkg_tof hold` extensions and no confirm command;
 * auto builds have no hold command. The SCB reset drops the main power rails.
 */

#pragma once

#if defined(ENABLE_TOF_PACKAGE_BENCH) || defined(ENABLE_TOF_PACKAGE_AUTO_DEADMAN)

#include <stdint.h>

namespace lexxhard::tof_package_bench_deadman {

/* Ten-minute test window; two extensions cap the run at 30 minutes from arming. */
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
