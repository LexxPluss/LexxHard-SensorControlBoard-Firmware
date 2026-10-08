/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_package_bench_deadman.hpp"

#if defined(ENABLE_TOF_PACKAGE_BENCH)

#include <errno.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(tof_package_bench_deadman);

namespace lexxhard::tof_package_bench_deadman {
namespace {

/* OFF THE SYSTEM CLOCK INTERRUPT, WHICH IS THE WHOLE POINT. The case this exists for is a build
 * that has stopped scheduling its own threads while the watchdog is still being fed, so a watchdog
 * thread, a work item or anything else that needs the scheduler would be wedged along with it. A
 * k_timer expiry runs in the clock ISR and fires regardless, and sys_reboot() is reachable from
 * there: it locks interrupts and resets the core. */
K_TIMER_DEFINE(revert_timer, [](k_timer *) { sys_reboot(SYS_REBOOT_COLD); }, nullptr);

uint32_t extensions_used_{0};

}  // namespace

bool image_confirmed(void)
{
    return boot_is_img_confirmed();
}

bool armed(void)
{
    return k_timer_remaining_ticks(&revert_timer) > 0;
}

uint32_t remaining_ms(void)
{
    const k_ticks_t left{k_timer_remaining_ticks(&revert_timer)};
    return left > 0 ? k_ticks_to_ms_floor32(left) : 0U;
}

uint32_t extensions_used(void)
{
    return extensions_used_;
}

void arm(void)
{
    if (image_confirmed()) {
        /* NOT A SILENT NO-OP. A confirmed bench image is a mistake -- nothing in this build confirms
         * one, so it took a deliberate act elsewhere -- and it has no revert left to protect. Arming
         * anyway would cold-reboot the machine every kWindowMinutes forever. */
        LOG_WRN("image is CONFIRMED: no unattended revert armed, and none is possible. This image "
                "will keep running until it is replaced.");
        return;
    }
    k_timer_start(&revert_timer, K_MINUTES(kWindowMinutes), K_NO_WAIT);
    LOG_WRN("unattended revert armed: WHOLE-MACHINE cold reboot in %u min, which reverts this "
            "unconfirmed bench image to production. `pkg_tof hold` extends it up to %u times. There "
            "is no confirm command in this image.",
            kWindowMinutes, kMaxExtensions);
}

int extend(uint32_t minutes)
{
    if (image_confirmed())
        return -EALREADY;
    if (minutes == 0U || minutes > kWindowMinutes)
        return -EINVAL;
    if (extensions_used_ >= kMaxExtensions)
        return -EPERM;

    ++extensions_used_;
    k_timer_start(&revert_timer, K_MINUTES(minutes), K_NO_WAIT);
    LOG_WRN("unattended revert re-armed: cold reboot in %u min (extension %u of %u)", minutes,
            extensions_used_, kMaxExtensions);
    return 0;
}

}  // namespace lexxhard::tof_package_bench_deadman

#endif  // ENABLE_TOF_PACKAGE_BENCH
