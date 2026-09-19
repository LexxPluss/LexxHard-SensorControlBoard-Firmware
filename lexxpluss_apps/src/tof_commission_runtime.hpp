/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The board's side of the commissioning downlink as a running thing: one worker, one periodic
 * announcement, and a receive entry point. It owns no policy and, deliberately, NO IDENTIFIERS.
 *
 * WHY THE IDENTIFIERS ARE INJECTED. Registration of the commissioning pair has not come back, and
 * 0x214/0x215 turned out to belong to the grid transport already -- which is exactly the kind of
 * thing a placeholder constant hides until somebody greps for it. So this file names no number, the
 * configuration carries both identifiers, and a configuration that omits either is refused rather
 * than defaulted. The production values arrive through tof_commission_ids.hpp, which is generated
 * from the wire contract and does not compile until the registrar has answered; the tests pass their
 * own. When the answer comes, one generated header changes and nothing here does.
 *
 * WHAT IT IS NOT. It is not the state machine, which is tof_commission_session, and it is not the
 * proof, which is tof_commissioning. It moves frames between them, on the right threads.
 *
 * THREADS, AND WHY THE SPLIT IS THIS WAY. `on_frame()` is the receive path: it validates and answers
 * from whatever context CAN delivers on, taking a spinlock for a few hundred instructions and never
 * blocking. `service_once()` is the worker: it runs the transaction, which takes the chain mutex and
 * hundreds of milliseconds of I2C. One thread calls it -- `start()` creates that thread and refuses
 * a second -- and the session layer's own claim makes a second caller harmless rather than a defect
 * that first appears on a bus.
 *
 * DEFAULT OFF, AND NOTHING IS ENABLED BY EXISTING. `config::profile_enabled` is false when
 * default-constructed, `init()` refuses a configuration without hooks or identifiers, and no CAN
 * filter is installed from here: installing one is the binding's job, because the binding is where
 * the identifiers are.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_commission_session.hpp"
#include "tof_commissioning.hpp"

namespace lexxhard::tof_commission_runtime {

namespace session = tof_commission;

struct config {
    /* Both required and both checked. Zero is not a CAN identifier and "the same value for both"
     * is a wiring mistake that would answer a request with a frame the sender reads back as a
     * request. */
    uint32_t request_id{0};
    uint32_t status_id{0};

    /* How often the session frame goes out. A host that has just come up learns the session token
     * from it, so the period is the worst case for how long an unattended machine waits before it
     * can address the board at all. */
    uint32_t announce_period_ms{1000};

    /* How long the worker sleeps when there is nothing queued. It bounds how late a queued
     * transaction starts, not how long one takes. */
    uint32_t poll_ms{20};

    /* Passed straight through to the session layer. False means the board answers `disabled`. */
    bool profile_enabled{false};
    uint8_t max_proof_attempts{0};
    uint8_t max_start_attempts{0};
};

struct hooks {
    /* Puts exactly `len` bytes on the bus under `id`. Non-zero means it did not go out, which is
     * counted and otherwise tolerated: the host retransmits, and a board that blocked its receive
     * path on a full mailbox would be a worse failure than a lost frame. */
    int (*send)(void *ctx, uint32_t id, const uint8_t *data, size_t len);

    /* May the chain be re-enumerated right now? ASKED, NEVER INFERRED -- a trustworthy stationary
     * condition does not exist yet and this module must not invent one. A binding that answers a
     * constant `true` has not answered the question; it has removed it. */
    bool (*enumeration_permitted)(void *ctx);

    /* The real transaction and the real acquisition start. Injected so a test can drive the runtime
     * without hardware; production passes tof_commissioning::prove and
     * tof_cliff_runtime::start_acquisition. */
    int (*prove)(void *ctx, uint32_t epoch, tof_commissioning::outcome *out);
    int (*start)(void *ctx);

    /* Entropy for the session token. Production passes tof_commission_entropy::draw_token; there is
     * no default and no fallback. */
    int (*draw_token)(void *ctx, uint32_t *out);

    void *ctx;
};

struct counters {
    uint32_t frames_in{0};        /* frames offered to on_frame() under the request identifier */
    uint32_t frames_ignored{0};   /* offered under some other identifier */
    uint32_t status_sent{0};
    uint32_t sessions_sent{0};
    uint32_t send_failures{0};
    uint32_t transactions{0};     /* worker steps that produced a terminal status */
};

/* Draws the session token and configures the state machine. Returns 0, or negative when the
 * configuration is incomplete (-EINVAL) or no session could be established (-ENODEV) -- and in the
 * second case the board deliberately says NOTHING on the bus, because a board that cannot tell this
 * boot from the last one has no business announcing a session at all. */
int init(const config &cfg, const hooks &h);

/* The receive path. Frames under any other identifier are ignored and counted. Never blocks, never
 * proves, never takes the chain. */
void on_frame(uint32_t id, const uint8_t *data, size_t len);

/* One turn of the worker: run the queued transaction if there is one, and send the periodic session
 * frame when it is due. Exposed so a test drives exactly the code the thread runs. */
struct service_result {
    bool sent_status{false};
    bool sent_session{false};
    bool terminal{false};
};
service_result service_once(int64_t now_ms);

/* Creates the single worker thread. -EALREADY if one is already running, which is the point: one
 * worker, decided here rather than hoped for. */
int start();
bool running();

counters stats();

} // namespace lexxhard::tof_commission_runtime

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
