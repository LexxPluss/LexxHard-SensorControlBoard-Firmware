/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The protocol's third layer: sessions, the validation order, the idempotency table and the per-boot
 * budgets. The codec below it turns bytes into structs; `tof_auto_commission` above it sequences a
 * proof and a start. This is what decides whether a request is answered at all.
 *
 * THE RX PATH DOES NO WORK. `handle_request()` runs in the CAN receive callback: it validates,
 * answers, and at most queues. It never proves, never takes the chain lock and never blocks -- a
 * transaction takes hundreds of milliseconds and the receive path cannot be gone for that long.
 * `worker_step()` is where the transaction actually runs, on a thread that is allowed to block.
 *
 * BUDGETS ARE PER BOOT, WHICH IS WHY `tof_auto_commission` IS INITIALISED ONCE. A new
 * `request_seq`, a host restart and a `start_only` after a failed `prove_and_start` all draw on the
 * same two counters. Re-initialising the sequencer per request would hand out a fresh budget every
 * time the host changed a sequence number, which a retry loop does by accident -- turning "bounded
 * retry" into unbounded retry without anyone intending it.
 *
 * THE PROOF IS NOT REIMPLEMENTED. The prove hook runs `tof_commissioning::prove()` and hands back its
 * `outcome`, which `tof_commission_map` translates. Nothing here decides what a failed walk means.
 *
 * DEFAULT OFF, AND NO NUMERIC IDENTIFIER. `config::profile_enabled` is false when
 * default-constructed, and this file names no CAN identifier: registration of 0x214-0x219 has not
 * come back and the draft is not frozen.
 */

#pragma once

#include "tof_commission_wire.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_commissioning.hpp"

namespace lexxhard::tof_commission {

namespace wire = tof_commission_wire;

struct hooks {
    /* 0 and a NON-ZERO token, or negative when no entropy is available. Injected so the tests can
     * drive the no-entropy path, which is unreachable on a bench; the production binding is the
     * STM32 RNG entropy device and nothing else -- never a timer PRNG, whose adjacent boots would be
     * correlated and would make the 2^-32 collision bound a fiction. */
    int (*draw_token)(void *ctx, uint32_t *out);

    /* Asked, never inferred: the trustworthy stationary condition does not exist yet. */
    bool (*enumeration_permitted)(void *ctx);

    /* The real transaction. Returns 0 when a mapping was installed, and fills `out` either way. */
    int (*prove)(void *ctx, uint32_t epoch, tof_commissioning::outcome *out);

    /* Acquisition. 0 means the thread is running. */
    int (*start)(void *ctx);

    void *ctx;
};

struct config {
    /* Fixed for the life of the session, read once at boot. A request refused with `disabled` leaves
     * no table entry, so if this could change mid-session the very same frame -- retransmitted or
     * merely delayed -- would later be treated as new and run a transaction nobody asked for at that
     * moment. Enabling takes effect at the next boot, which is also when the token changes. */
    bool profile_enabled{false};
    uint8_t max_proof_attempts{0};
    uint8_t max_start_attempts{0};
};

/* What the caller must put on the bus, decided entirely here so the transport does no thinking. */
struct rx_action {
    bool send_status{false};
    wire::transaction_status status{};
    /* Sent after the status frame where both are set, so a host that has just been told
     * `stale_session` can resynchronise without waiting for the next periodic announcement. */
    bool send_session{false};
    bool queued{false};
};

enum class worker_state : uint8_t {
    idle,     /* nothing queued */
    running,  /* a transaction is in flight */
};

/* Drawn once. Returns false when no token could be had, which is not a recoverable state: the
 * subsystem announces no session and answers every request `no_session`, because a board that cannot
 * tell this boot from the last one has no business acting on a request that claims to know. */
bool init(const config &cfg, const hooks &h);

bool has_session();
uint32_t session_token();

/* The session announcement, for the periodic sender. */
wire::session_status announcement();

/* RX path. Never blocks, never proves. `len` is the DLC as received. */
rx_action handle_request(const uint8_t *data, size_t len);

/* Worker path. Runs at most one step of the queued transaction and reports whether there is a status
 * frame to send. Returns idle when there is nothing to do. */
struct worker_result {
    worker_state state{worker_state::idle};
    bool send_status{false};
    wire::transaction_status status{};
};
worker_result worker_step();

/* Diagnostics, for the tests and for whatever reports counters later. */
struct counters {
    uint32_t discarded_bad_length{0};
    uint32_t discarded_bad_version{0};
    uint32_t refused_stale_session{0};
    uint32_t refused_seq_conflict{0};
    uint32_t replayed_from_table{0};
    uint32_t accepted{0};
};
counters stats();

} // namespace lexxhard::tof_commission

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
