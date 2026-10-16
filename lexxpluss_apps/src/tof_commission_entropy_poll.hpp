/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The deadline-bounded fill that tof_commission_entropy uses, separated from the peripheral so it
 * can be tested at all.
 *
 * WHY IT IS NOT IN THAT FILE. tof_commission_entropy.cpp is bound to the `st,stm32-rng` devicetree
 * node and asserts at compile time that the node exists -- which is the no-fallback rule expressed
 * as code and must not be relaxed for testability. A host suite therefore cannot compile that
 * translation unit, so the logic that decides when a read has failed could not be driven by
 * anything. It is here, over injected seams, and the production file supplies the real ones at one
 * call site.
 *
 * WHAT IT IS FOR. Zephyr's blocking entropy call cannot fail on this driver: it waits on a
 * semaphore with K_FOREVER until the pool refills and returns zero either way, and the refill
 * interrupt leaves early on a seed or clock error without giving that semaphore. An RNG that stops
 * producing parks the caller for ever rather than failing. This turns that into a bounded wait with
 * a reportable outcome.
 */

#pragma once

#if defined(ENABLE_TOF_AUTO_COMMISSION)

#include <errno.h>
#include <stdint.h>

namespace lexxhard::tof_commission_entropy::detail {

/* The three seams, all of them narrow on purpose. `take` is the non-blocking entropy read: it
 * returns how many bytes it could take, zero included, or a negative errno. `now_ms` is a monotonic
 * clock. `wait` is whatever the caller does between attempts -- a short sleep in production, nothing
 * at all in a test. */
struct poll_io {
    int (*take)(void *ctx, uint8_t *dst, uint16_t len){nullptr};
    int64_t (*now_ms)(void *ctx){nullptr};
    void (*wait)(void *ctx){nullptr};
    void *ctx{nullptr};
};

/* Fills `len` bytes or gives up.
 *
 * Returns 0 with `buf` filled, -ETIMEDOUT when the deadline passed with bytes still owed, the
 * take's own negative errno if it reports one, or -EINVAL for a seam that is not wired.
 *
 * THE DEADLINE IS CHECKED AFTER AN ATTEMPT, NOT BEFORE, so a deadline of zero still makes one
 * attempt: on working hardware the first read usually satisfies the request, and refusing to look
 * even once would turn a tight deadline into a guaranteed failure. */
inline int fill(const poll_io &io, uint8_t *buf, uint16_t len, int64_t deadline_ms)
{
    if (io.take == nullptr || io.now_ms == nullptr || io.wait == nullptr || buf == nullptr)
        return -EINVAL;

    const int64_t give_up_at{io.now_ms(io.ctx) + deadline_ms};
    uint16_t have{0};

    while (have < len) {
        const int got{io.take(io.ctx, buf + have, static_cast<uint16_t>(len - have))};
        if (got < 0)
            return got;
        /* A take that claims more than was asked for is a broken seam, not a windfall: trusting it
         * would mean reporting success for a buffer that was never filled. */
        if (got > static_cast<int>(len - have))
            return -EIO;
        have = static_cast<uint16_t>(have + got);
        if (have >= len)
            return 0;
        if (io.now_ms(io.ctx) >= give_up_at)
            return -ETIMEDOUT;
        io.wait(io.ctx);
    }
    return 0;
}

}  // namespace lexxhard::tof_commission_entropy::detail

#endif  // ENABLE_TOF_AUTO_COMMISSION
