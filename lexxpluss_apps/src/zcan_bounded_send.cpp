/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "zcan_bounded_send.hpp"

#include <zephyr/sys/atomic.h>

#include <errno.h>

namespace lexxhard::zcan_bounded_send {

namespace {

atomic_t queued_{};
atomic_t refused_{};
atomic_t completed_{};
atomic_t failed_{};

/* Saturate instead of wrapping. A counter that returns to zero after four billion frames reads as
 * "nothing has failed", which is the one answer it must never give. */
void bump(atomic_t &c)
{
    atomic_val_t seen{atomic_get(&c)};
    while (seen != static_cast<atomic_val_t>(UINT32_MAX)) {
        if (atomic_cas(&c, seen, seen + 1))
            return;
        seen = atomic_get(&c);
    }
}

/* RUNS IN THE TX INTERRUPT. Atomics only: no logging, no locks, nothing that can sleep. The frame
 * is already out of the mailbox by the time this runs, so there is nothing here to retry and
 * nothing to free -- callers own their own frames and have long since returned. */
void on_done(const device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    bump(error == 0 ? completed_ : failed_);
}

}  // namespace

int send(const device *dev, const can_frame *frame, k_timeout_t timeout)
{
    if (dev == nullptr || frame == nullptr)
        return -EINVAL;

    /* The callback is what makes this bounded: with a null callback Zephyr would wait on the
     * completion semaphore with K_FOREVER after this returns. See the header. */
    const int rc{can_send(dev, frame, timeout, on_done, nullptr)};
    bump(rc == 0 ? queued_ : refused_);
    return rc;
}

counts snapshot()
{
    return counts{
        .queued{static_cast<uint32_t>(atomic_get(&queued_))},
        .refused{static_cast<uint32_t>(atomic_get(&refused_))},
        .completed{static_cast<uint32_t>(atomic_get(&completed_))},
        .failed{static_cast<uint32_t>(atomic_get(&failed_))},
    };
}

void reset_counts()
{
    atomic_clear(&queued_);
    atomic_clear(&refused_);
    atomic_clear(&completed_);
    atomic_clear(&failed_);
}

}  // namespace lexxhard::zcan_bounded_send
