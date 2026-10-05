/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
 * THE SEND IS NEVER CALLED FROM THE RECEIVE PATH, and the non-blocking form is not enough on its
 * own. `on_frame()` runs in the CAN receive INTERRUPT -- the bxCAN driver calls a filter's callback
 * straight out of can_stm32_rx_isr_handler() -- and can_send() is not callable from there whatever
 * timeout it is handed: can_stm32_bxcan.c takes k_mutex_lock(&data->inst_mutex, K_FOREVER) before
 * it reads the timeout, so K_NO_WAIT and the completion callback bound the wait for a MAILBOX and
 * do nothing about the mutex. k_mutex_lock() from an ISR is a kernel assertion.
 *
 * So `bind_send` below is only ever reached from a thread: the runtime queues what the interrupt
 * composed and a work item drains the queue. It still uses the CALLBACK form, because the
 * synchronous one waits for transmission to COMPLETE (see tof_cliff_can.cpp, which documents that
 * trap) and the work item has no business holding a thread for the length of a bus arbitration. A
 * full mailbox comes back -EAGAIN, which the runtime counts and the host's retransmission recovers.
 *
 * THE STATIONARY CONDITION IS NOT DEFAULTED. `config::enumeration_permitted` has no default and a
 * null one is refused: a board must not re-enumerate its chain because nobody said it should not.
 * Wiring it to a constant `true` on a bench is a decision to write down where the bench is
 * configured, not something this file may assume.
 *
 * WHO CALLS IT: tof_chain_controller::init(), once per boot, through tof_commission_wiring -- which
 * is where the order and the once-ness live, because a second can_add_rx_filter() on the request
 * identifier does not fail, it delivers every request twice. It runs after the cliff runtime's
 * bootstrap has returned, because the worker's prove and start hooks reach that runtime.
 *
 * WHAT THE CALLER STILL DOES NOT DECIDE FOR THE BOARD, so that being wired is not mistaken for
 * being enabled: whether the board may entertain a commissioning request at all, and whether it may
 * re-enumerate the chain when it gets one. Both are deployment acts rather than consequences of
 * linking, both are off unless an image says otherwise, and neither may be changeable at runtime.
 * They enter as the devicetree properties commission-profile-enabled and
 * commission-enumeration-permitted, absent by default and absent from the auto-commission overlay;
 * `config` above is where they reach this file.
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
