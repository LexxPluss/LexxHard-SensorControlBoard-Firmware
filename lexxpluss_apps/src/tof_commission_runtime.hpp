/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * The board's side of the commissioning downlink as a running thing: one worker, one periodic
 * announcement, and a receive entry point. It owns no policy and, deliberately, NO IDENTIFIERS.
 *
 * WHY THE IDENTIFIERS ARE INJECTED, NOW THAT THEY EXIST. The pair was allocated on 2026-09-19 --
 * 0x218 request, 0x219 status -- and it still does not appear here. The values live in the generated
 * wire contract, the binding reads them from there and passes them in, and the tests pass their own:
 * a suite using the real pair could not tell a runtime that reads its configuration from one that
 * ignores it. A configuration that omits either identifier is refused rather than defaulted, because
 * a default identifier is a frame on somebody else's conversation. 0x214/0x215 turning out to belong
 * to the grid transport is exactly the kind of thing a placeholder constant hides until somebody
 * greps for it.
 *
 * WHAT IT IS NOT. It is not the state machine, which is tof_commission_session, and it is not the
 * proof, which is tof_commissioning. It moves frames between them, on the right threads.
 *
 * THREADS, AND WHY THE SPLIT IS THIS WAY. `on_frame()` is the receive path, and ON THIS BOARD IT IS
 * AN ISR: the bxCAN driver calls a filter's callback straight out of its receive interrupt. So it
 * takes a spinlock for a few hundred instructions, never blocks, and -- the part that is not
 * obvious -- never sends. It composes the answer and puts the bytes in a bounded queue that a work
 * item drains from thread context, because can_send() takes a K_FOREVER mutex before it looks at
 * the timeout it was given. A full queue drops the answer and counts it; waiting is the one thing
 * an interrupt may not do, and the host's retransmission is what recovers a dropped frame.
 *
 * `service_once()` is the worker: it runs the transaction, which takes the chain mutex and hundreds
 * of milliseconds of I2C. It already runs on a thread, so it sends directly -- which is why its
 * result booleans can still mean "it reached the bus". One thread calls it -- `start()` creates that
 * thread and refuses a second -- and the session layer's own claim makes a second caller harmless
 * rather than a defect that first appears on a bus.
 *
 * ONE INIT PER BOOT, AND IT IS A PRECONDITION RATHER THAN A LOCK. `init()` reconfigures the session
 * layer, which draws a new token and resets the attempt budgets; #116 states that as a once-per-boot
 * operation and this layer inherits it. Calling it again while the worker is alive would move the
 * ground under a transaction in flight, and the worker is NOT stoppable -- `start()` refuses a
 * second thread precisely because the first one exists for the life of the process. Nothing here
 * enforces it: a lease or a lock would be a mechanism to make a mistake survivable that no caller in
 * this firmware can make, since the production caller runs once from the bootstrap. The suites
 * respect it by isolating the case that cannot.
 *
 * A BOARD WITH NO ENTROPY STILL ANSWERS. It announces no session -- it cannot tell this boot from
 * the last one, so it has nothing to announce -- but a well-formed request is still answered, with
 * `no_session`. The difference matters to a host that comes back holding a durable pending request:
 * silence leaves it retransmitting for ever, while `no_session` is terminal and tells it to stop and
 * report. Refusing to speak and refusing to act are different refusals.
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
    /* Both required and both checked: non-zero, distinct, and 11 bits. This bus is CAN classic with
     * standard identifiers, so a value above 0x7FF is not a wider identifier here -- it is a
     * configuration that would be truncated or refused by the driver, after the runtime had already
     * reported itself configured. "The same value for both" is the other wiring mistake: it would
     * answer a request with a frame the sender reads back as a request. */
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
    /* Answers the receive path composed and could not hand to the transmit queue. SEPARATE from
     * send_failures, which is a transport refusal: this one is the board overrunning its own queue,
     * and the host's retransmission is what recovers it. */
    uint32_t tx_dropped{0};
    uint32_t transactions{0};     /* worker steps that produced a terminal status */
};

/* Draws the session token and configures the state machine. Returns 0, or -EINVAL for a
 * configuration that is incomplete, or -ENODEV when no session could be drawn.
 *
 * -ENODEV IS NOT "DEAD". The receive path still works and still answers -- with `no_session`, which
 * is what the protocol says and what breaks a returning host out of its retransmit loop. What the
 * board does not do is announce a session or run a transaction. */
int init(const config &cfg, const hooks &h);

/* Whether a session token was drawn. False means every request is answered `no_session` and no
 * announcement is sent. */
bool has_session();

/* The receive path, callable from an ISR. Frames under any other identifier are ignored and
 * counted. Never blocks, never sends, never proves, never takes the chain: the answer it composes
 * is queued, and a work item puts it on the bus. */
void on_frame(uint32_t id, const uint8_t *data, size_t len);

/* One turn of the worker: run the queued transaction if there is one, and send the periodic session
 * frame when it is due. Exposed so a test drives exactly the code the thread runs. */
struct service_result {
    /* TRUE MEANS IT REACHED THE BUS. A send that failed leaves these false and shows up in
     * `counters::send_failures`; naming them for the attempt would make a caller that logs them
     * report traffic that does not exist. */
    bool sent_status{false};
    bool sent_session{false};
    /* The worker produced a terminal status this call, whether or not it could be sent. */
    bool terminal{false};
};
service_result service_once(int64_t now_ms);

/* Creates the single worker thread. The claim is taken under the same lock as everything else, so
 * two callers racing produce one thread and one -EALREADY rather than two threads. -EPERM when
 * there is no session: there is nothing for a worker to do on a board that answers every request
 * `no_session` from the receive path. */
int start();
bool running();

counters stats();

} // namespace lexxhard::tof_commission_runtime

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
