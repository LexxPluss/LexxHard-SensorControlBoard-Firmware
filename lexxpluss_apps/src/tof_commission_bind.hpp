/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Where the commissioning runtime meets the real bus, the real proof and the real entropy. This is
 * the only file in the firmware that knows all four at once, and it is where the allocated
 * identifiers enter -- read from the generated contract, never written down here.
 *
 * WHAT IT DECIDES, and the three outcomes are deliberately not two:
 *
 *   running          a session was drawn. The receive filter is installed and the worker is up:
 *                    the board can be commissioned.
 *   answering_only   no session could be drawn -- the entropy peripheral did not answer. The filter
 *                    IS still installed and every request is answered `no_session`, because a host
 *                    holding a durable pending request from a previous boot needs a terminal answer
 *                    to stop retransmitting. No worker, no announcement, nothing proved.
 *   refused          the configuration or the device is wrong. NOTHING is installed and nothing is
 *                    started, because a filter without a runtime behind it takes an identifier off
 *                    the bus and answers nothing on it.
 *
 * THE SEND IS NON-BLOCKING, and that is not a preference. `on_frame()` runs in the CAN receive
 * callback, so the answer is sent from inside that callback; the synchronous form of can_send()
 * waits for transmission to COMPLETE (see tof_cliff_can.cpp, which documents the same trap) and
 * doing that from a receive callback stalls reception behind the bus. The callback form returns as
 * soon as the frame is queued, and a full mailbox comes back as -EAGAIN, which the runtime counts
 * and the host's retransmission recovers.
 *
 * THE STATIONARY CONDITION IS NOT DEFAULTED. `config::enumeration_permitted` has no default and a
 * null one is refused: a board must not re-enumerate its chain because nobody said it should not.
 * Wiring it to a constant `true` on a bench is a decision to write down where the bench is
 * configured, not something this file may assume.
 *
 * NOTHING CALLS THIS YET, and that is why the profile is still off by default: turning it on is a
 * deployment act, not a consequence of linking.
 */

#pragma once

#include <stdint.h>

#if defined(ENABLE_TOF_AUTO_COMMISSION)

struct device;

namespace lexxhard::tof_commission_bind {

struct config {
    /* Straight through to the runtime and the session layer. Off, and zero budgets, unless a
     * deployment says otherwise. */
    bool profile_enabled{false};
    uint8_t max_proof_attempts{0};
    uint8_t max_start_attempts{0};
    uint32_t announce_period_ms{1000};
    uint32_t poll_ms{20};

    /* Required. See the note above: there is no default and no constant `true` here. */
    bool (*enumeration_permitted)(void *ctx){nullptr};
    void *ctx{nullptr};
};

enum class outcome : uint8_t {
    running,
    answering_only,
    refused,
};

struct result {
    outcome state{outcome::refused};
    int rc{0};                     /* what the runtime's init() said */
    bool filter_installed{false};
    bool worker_started{false};
    int filter_id{-1};             /* what can_add_rx_filter() returned, for diagnostics */
};

/* `can_dev` is passed in rather than looked up, so the test can hand it a real CAN device that is
 * not the board's. What is exercised either way is the real driver, the real filter and the real
 * send path. */
result start(const struct device *can_dev, const config &cfg);

} // namespace lexxhard::tof_commission_bind

#endif // ENABLE_TOF_AUTO_COMMISSION
