/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Periodic SYNC due-time logic of the TDMA host. Pure: the tick and the send function are passed in.
 * The first poll is due at once. The grid is kept (next_due += period); when a poll finds the schedule one
 * period or more behind, one SYNC is sent and the grid is realigned to now + period (reported late). A rejected
 * send is not retried: it is reported as skipped and the due time advances as for a sent frame.
 * Ticks are uint32 of kTickUs and are compared wrap-safe (unsigned subtraction).
 */

#pragma once

#include <cstdint>

#include "lcs_frame.hpp"
#include "lcs_timing.hpp"

namespace lexxhard::lcs {

/* 1000 SPS timing budget of the load cell boards. */
inline constexpr uint32_t kSyncPeriodUs{5000};

constexpr uint32_t sync_period_ticks(const uint32_t period_us)
{
    return period_us / kTickUs;
}

static_assert(kSyncPeriodUs % kTickUs == 0, "the SYNC period must be a whole number of ticks");
static_assert(sync_period_ticks(kSyncPeriodUs) > 0 && sync_period_ticks(kSyncPeriodUs) < 0x80000000U,
              "the SYNC period must fit the wrap-safe comparison");

enum class SyncOutcome : uint8_t { NotDue, Sent, Skipped };

struct SyncPoll {
    SyncOutcome outcome;
    bool late; /* the grid was realigned */
};

class SyncScheduler {
public:
    static constexpr uint32_t kPeriodTicks{sync_period_ticks(kSyncPeriodUs)};

    /* send is callable as bool(const DownlinkFrame &); true means the transmission was accepted. */
    template <typename Send>
    SyncPoll poll(const uint32_t tick, Send &&send)
    {
        if (!started_) {
            started_ = true;
            next_due_ = tick;
        }
        const uint32_t overdue{tick - next_due_};
        if (overdue >= kHalfRange) {
            return {SyncOutcome::NotDue, false};
        }
        const bool late{overdue >= kPeriodTicks};
        next_due_ = late ? tick + kPeriodTicks : next_due_ + kPeriodTicks;
        const bool accepted{send(encode_downlink(kBroadcastId, kCmdLinkSync, 0))};
        return {accepted ? SyncOutcome::Sent : SyncOutcome::Skipped, late};
    }

    /* The next poll() at this tick would send a SYNC. */
    bool due(const uint32_t tick) const
    {
        return !started_ || tick - next_due_ < kHalfRange;
    }

    uint32_t next_due() const
    {
        return next_due_;
    }

private:
    static constexpr uint32_t kHalfRange{0x80000000U};

    uint32_t next_due_{0};
    bool started_{false};
};

}
