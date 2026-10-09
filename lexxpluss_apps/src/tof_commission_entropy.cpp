/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_commission_entropy.hpp"

#include "tof_commission_entropy_poll.hpp"

#if defined(ENABLE_TOF_AUTO_COMMISSION)

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/kernel.h>

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

    /* POLLED WITH A DEADLINE RATHER THAN ASKED TO BLOCK, and the blocking version is not a style
     * choice this replaces -- it is a hang. entropy_get_entropy() lands on
     * entropy_stm32_rng_get_entropy(), which loops `if (bytes == 0) k_sem_take(&sem_sync,
     * K_FOREVER)` and then returns 0 unconditionally; it has no failure return at all. And the
     * interrupt that refills the pool begins `byte = random_byte_get(); if (byte < 0) return;`, so
     * a seed or clock error leaves without giving that semaphore. An RNG that stops refilling did
     * not fail this call: it parked the caller for ever, silently, taking the bootstrap thread with
     * it -- and this file's own contract, that a read which fails ends in no session, could never
     * be honoured because the read never returned.
     *
     * entropy_get_entropy_isr() with no flags is the non-blocking half: it returns how many bytes
     * it could take from the ISR pool, zero included, and never waits. The loop that accumulates
     * them and gives up lives in tof_commission_entropy_poll.hpp, because this translation unit is
     * nailed to the st,stm32-rng node and cannot be compiled by a host suite.
     *
     * THE DEADLINE, with its basis rather than a round number. The driver refills from the RNG
     * interrupt and the peripheral produces a 32-bit word roughly every 42 RNG clock cycles, so on
     * working hardware the pool is filled in well under a millisecond. 50 ms is three orders of
     * magnitude above that, which is the margin that makes a timeout mean "this RNG is not
     * producing" rather than "the board was busy". It is spent once, at init, before any session
     * exists, so the cost of the pessimistic value is a boot that takes 50 ms longer to decide it
     * has no session -- on a board that then announces none anyway. */
    constexpr int64_t kDeadlineMs{50};

    const detail::poll_io io{
        [](void *, uint8_t *dst, uint16_t len) {
            return entropy_get_entropy_isr(rng(), dst, len, 0);
        },
        [](void *) { return k_uptime_get(); },
        [](void *) { k_sleep(K_MSEC(1)); },
        nullptr,
    };

    uint8_t buf[sizeof(uint32_t)]{};
    if (const int rc{detail::fill(io, buf, sizeof buf, kDeadlineMs)}; rc != 0)
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
