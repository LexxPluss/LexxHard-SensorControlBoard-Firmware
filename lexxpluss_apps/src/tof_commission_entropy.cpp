/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commission_entropy.hpp"

#if defined(ENABLE_TOF_AUTO_COMMISSION)

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>

namespace lexxhard::tof_commission_entropy {

namespace {

/* THE PERIPHERAL, BY COMPATIBLE, and this is the whole no-fallback rule expressed as code. Asking
 * for "the entropy device" would take whatever the configuration provided, including Zephyr's
 * timer-seeded generator; naming `st,stm32-rng` means a build without the hardware RNG enabled does
 * not compile. */
BUILD_ASSERT(DT_HAS_COMPAT_STATUS_OKAY(st_stm32_rng),
             "automatic commissioning needs the STM32 hardware RNG enabled -- stack "
             "overlays/auto_commission.overlay, which is the only thing that does it");

/* And the fallbacks are refused by name as well, because selecting one of them alongside the real
 * peripheral would compile and would silently decide which device the entropy API returns. */
#if defined(CONFIG_TIMER_RANDOM_GENERATOR)
#error "automatic commissioning must not be built with the timer-seeded generator"
#endif
#if defined(CONFIG_TEST_RANDOM_GENERATOR)
#error "automatic commissioning must not be built with the test random generator"
#endif
#if defined(CONFIG_XOSHIRO_RANDOM_GENERATOR)
#error "automatic commissioning must not be built with the xoshiro PRNG"
#endif

const struct device *rng()
{
    static const struct device *const dev{DEVICE_DT_GET_ONE(st_stm32_rng)};
    return dev;
}

} // namespace

bool available()
{
    return device_is_ready(rng());
}

int draw_token(void *, uint32_t *out)
{
    if (out == nullptr)
        return -EINVAL;

    /* Cleared first. A caller that ignored the return value would otherwise read whatever was on its
     * stack as a token, and the one value that must never be mistaken for a token is a plausible
     * one. */
    *out = 0;

    if (!device_is_ready(rng()))
        return -ENODEV;

    uint8_t buf[sizeof(uint32_t)]{};
    if (const int rc{entropy_get_entropy(rng(), buf, sizeof buf)}; rc != 0)
        return rc;

    /* Assembled byte by byte rather than memcpy'd into a uint32_t: the token is compared as a number
     * on both ends of a little-endian wire field, and building it explicitly means this file decides
     * the order rather than the compiler's idea of the host's. */
    *out = static_cast<uint32_t>(buf[0]) | (static_cast<uint32_t>(buf[1]) << 8) |
           (static_cast<uint32_t>(buf[2]) << 16) | (static_cast<uint32_t>(buf[3]) << 24);

    /* A zero draw is returned as a zero draw. It is an ordinary sample -- once in 2^32 -- and the
     * session layer is where "zero means no token" lives: it draws again and refuses only on a
     * second zero. Deciding it here would put the same rule in two places, and the one that drifted
     * would be the one nobody tested. */
    return 0;
}

} // namespace lexxhard::tof_commission_entropy

#endif // ENABLE_TOF_AUTO_COMMISSION
