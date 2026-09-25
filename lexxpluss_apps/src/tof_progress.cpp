/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_progress.hpp. The read order in read() is the part that matters.
 */

#include "tof_progress.hpp"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

namespace lexxhard::tof_progress {

namespace {

constexpr size_t kCount{static_cast<size_t>(activity::count)};

atomic_t begun_[kCount]{};
atomic_t ended_[kCount]{};
atomic_t zcan_{};

bool usable(activity a)
{
    return static_cast<size_t>(a) < kCount;
}

}  // namespace

void begin(activity a)
{
    if (usable(a))
        atomic_inc(&begun_[static_cast<size_t>(a)]);
}

void end(activity a)
{
    if (usable(a))
        atomic_inc(&ended_[static_cast<size_t>(a)]);
}

void zcan_tick()
{
    atomic_inc(&zcan_);
}

activity current_send_slot()
{
    /* The same discrimination the diagnostic images used: everything that is not the system work
     * queue is the acquisition slot. It is decided from the calling thread rather than from the CAN
     * id because the id says what the frame is, not which sender is stuck in it. */
    return k_current_get() == &k_sys_work_q.thread ? activity::send_workq : activity::send_acq;
}

snapshot read()
{
    snapshot s{};
    for (size_t i{0}; i < kCount; ++i) {
        /* ENDED FIRST, THEN BEGUN, and the order is the whole reason this is not a plain loop.
         *
         * `begun` never trails `ended`, so reading the older value from `ended` keeps that true of
         * the pair: the worst reading is a pair that looks in-flight for one sample when the work
         * had just completed. Reading `begun` first could produce the opposite -- more completions
         * than starts -- which is a state the feeder has no meaning for. */
        s.at[i].ended = static_cast<uint32_t>(atomic_get(&ended_[i]));
        s.at[i].begun = static_cast<uint32_t>(atomic_get(&begun_[i]));
    }
    s.zcan_loops = static_cast<uint32_t>(atomic_get(&zcan_));
    return s;
}

}  // namespace lexxhard::tof_progress
