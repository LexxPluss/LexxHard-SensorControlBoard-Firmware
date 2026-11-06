/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * ACTION window of the TDMA host. Pure: the tick and the request queue and transmitter are passed in.
 * The window is measured from the last SYNC that was sent; after a skipped SYNC the phase is undefined and nothing
 * is sent until the next SYNC. One send attempt is made per period; a refused request stays queued and is tried
 * again in the next period. Ticks are uint32 of kTickUs and are compared wrap-safe (unsigned subtraction).
 */

#pragma once

#include <cstdint>
#include <optional>

#include "lcs_frame.hpp"
#include "lcs_request.hpp"
#include "lcs_sync_schedule.hpp"
#include "lcs_timing.hpp"

namespace lexxhard::lcs {

/* INTERIM: ID1 slot end 1300 us + SYNC 171 us + guard 200 us, measured from the SYNC start. */
inline constexpr uint32_t kActionWindowStartUs{1671};
inline constexpr uint32_t kActionLengthUs{171};
inline constexpr uint32_t kActionGuardUs{200};

/* Rounded up plus one tick: the SYNC send and the decision are sampled on the tick. */
inline constexpr uint32_t kActionWindowStartTicks{(kActionWindowStartUs + kTickUs - 1) / kTickUs + 1};
inline constexpr uint32_t kActionWindowLatestTicks{(kSyncPeriodUs - kActionLengthUs - kActionGuardUs) / kTickUs - 1};

static_assert(kActionWindowStartTicks < kActionWindowLatestTicks);
static_assert((kActionWindowStartTicks - 1) * kTickUs >= kActionWindowStartUs);
static_assert((kActionWindowLatestTicks + 1) * kTickUs + kActionLengthUs + kActionGuardUs <= kSyncPeriodUs);

enum class ActionOutcome : uint8_t { NotSent, Sent, Refused };

class ActionWindow {
public:
    void on_sync(const SyncPoll &poll, const uint32_t tick)
    {
        switch (poll.outcome) {
        case SyncOutcome::Sent:
            origin_ = tick;
            defined_ = true;
            attempted_ = false;
            break;
        case SyncOutcome::Skipped:
            defined_ = false;
            break;
        case SyncOutcome::NotDue:
            break;
        }
    }

    /* peek is std::optional<Request>(), pop is void(), send is bool(const DownlinkFrame &). */
    template <typename Peek, typename Pop, typename Send>
    ActionOutcome poll(const uint32_t tick, Peek &&peek, Pop &&pop, Send &&send)
    {
        if (!defined_ || attempted_) {
            return ActionOutcome::NotSent;
        }
        const uint32_t elapsed{tick - origin_};
        if (!(elapsed >= kActionWindowStartTicks && elapsed <= kActionWindowLatestTicks)) {
            return ActionOutcome::NotSent;
        }
        const std::optional<Request> request{peek()};
        if (!request) {
            return ActionOutcome::NotSent;
        }
        attempted_ = true;
        if (!send(encode_downlink(request->target, request->cmd, request->param))) {
            return ActionOutcome::Refused;
        }
        pop();
        return ActionOutcome::Sent;
    }

private:
    bool defined_{false};
    uint32_t origin_{0};
    bool attempted_{false};
};

}
