/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_commission_runtime.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include "tof_commission_wire.hpp"

namespace lexxhard::tof_commission_runtime {

namespace wire = tof_commission_wire;

namespace {

config cfg_{};
hooks hooks_{};

/* Written once by init(), before any thread that reads them exists, and read-only afterwards. The
 * lock below is for the things that are not. */
bool configured_{false};
bool session_ok_{false};

/* THE COUNTERS ARE SHARED. `on_frame()` runs on whatever thread CAN delivers on and the worker runs
 * on its own, so a plain `++` on either side is a data race the moment a real filter is installed --
 * and `stats()` reading them unlocked would hand out a total that never existed. The lock is taken
 * for the increment and for the snapshot, and NEVER across a send or a transaction. */
struct k_spinlock lock_;
counters stats_{};

void count(uint32_t counters::*field, uint32_t by = 1)
{
    k_spinlock_key_t key{k_spin_lock(&lock_)};
    stats_.*field += by;
    k_spin_unlock(&lock_, key);
}

int64_t last_announce_ms_{0};
bool announced_once_{false};

/* The worker. One thread, created once; `start()` is what makes that true rather than a comment.
 *
 * 4 KiB is a starting point and not a measurement: the transaction that runs on this thread walks
 * the chain twice and isolates the tail, and the depth of that has been measured on the shell
 * thread and not here. CONFIG_THREAD_ANALYZER on a board, before release. */
constexpr size_t kWorkerStack{4096};
K_THREAD_STACK_DEFINE(worker_stack, kWorkerStack);
struct k_thread worker_thread;
/* NOT cleared by init(). A thread, once created, exists for the life of the process, and the
 * k_thread object it was created on cannot be reused while it does -- so a second init() must not
 * make start() willing to create another one on top of it. */
bool worker_started_{false};

/* The send itself happens with NO lock held: it is the transport's call, it may block or fail, and
 * holding a spinlock across it would mask interrupts for as long as a mailbox takes. */
int send_frame(uint32_t id, const uint8_t *data, size_t len)
{
    const int rc{hooks_.send != nullptr ? hooks_.send(hooks_.ctx, id, data, len) : -ENODEV};
    if (rc != 0)
        count(&counters::send_failures);
    return rc;
}

/* ------------------------------------------------- the transmit queue, and why it exists ------ */
/*
 * THE RECEIVE PATH MAY NOT SEND. On this board `on_frame()` runs in the CAN interrupt: the bxCAN
 * driver calls the filter's callback straight out of can_stm32_rx_isr_handler(). And can_send() is
 * not callable from there, whatever timeout it is given -- can_stm32_bxcan.c takes
 * k_mutex_lock(&data->inst_mutex, K_FOREVER) before it looks at the timeout at all, so the
 * K_NO_WAIT and the completion callback bound how long it waits for a MAILBOX and do nothing about
 * the mutex. k_mutex_lock() from an ISR is a kernel assertion, not a slow path.
 *
 * This was invisible in the suites and would not have been visible until a board: native_sim's
 * loopback controller delivers in a thread, so every test here ran the send on a thread that was
 * allowed to block.
 *
 * So the interrupt does bounded work -- compose the answer and put the bytes in a queue -- and a
 * work item on the system workqueue does the sending from thread context. Bounded is the operative
 * word: a full queue drops the frame and counts it, because the one thing the interrupt may never
 * do is wait. The host's retransmission is what recovers a dropped answer, and the session's replay
 * table makes that retransmission cheap and idempotent.
 *
 * NOT THE PROOF WORKER. A board that could draw no token starts no worker and still has to answer
 * `no_session`; hanging the transmit on the worker would make that board silent, which is the exact
 * failure the three-outcome split in the binding exists to prevent.
 */
constexpr size_t kTxQueueDepth{8};

struct tx_item {
    uint32_t id;
    uint8_t data[wire::kFrameLen];
    /* Which counter the successful send belongs to. Carried with the frame because the ISR knows
     * what it composed and the work item would otherwise have to guess from the bytes. */
    bool is_session;
};

K_MSGQ_DEFINE(tx_q_, sizeof(tx_item), kTxQueueDepth, 4);
struct k_work tx_work_;
bool tx_work_inited_{false};

void tx_work_handler(struct k_work *)
{
    tx_item item{};

    /* Drain, rather than one per submission: k_work_submit() on an item that is already queued is a
     * no-op, so two frames composed back to back in one interrupt can share a single run. */
    while (k_msgq_get(&tx_q_, &item, K_NO_WAIT) == 0) {
        if (send_frame(item.id, item.data, sizeof item.data) != 0)
            continue;
        count(item.is_session ? &counters::sessions_sent : &counters::status_sent);
    }
}

/* Called from the receive path, which may be an ISR. Everything here is bounded. */
void queue_frame(uint32_t id, const uint8_t *data, bool is_session)
{
    tx_item item{};

    item.id = id;
    memcpy(item.data, data, sizeof item.data);
    item.is_session = is_session;

    if (k_msgq_put(&tx_q_, &item, K_NO_WAIT) != 0) {
        /* ITS OWN COUNTER. A frame that was never queued did not fail to send -- folding it into
         * send_failures would report a transport refusal for a queue the board overran itself. */
        count(&counters::tx_dropped);
        return;
    }
    k_work_submit(&tx_work_);
}

/* The thread-context forms, used by the worker. `service_once()` already runs on a thread, so it
 * hands the frame to the transport itself and reports what the transport said -- accepted into a
 * mailbox, or refused. Queueing there would push that answer one step further away, into "this
 * board agreed with itself to try later", which is not a fact about the bus at all. Neither form
 * learns whether the frame was actually transmitted; see service_result in the header. */
bool send_status(const wire::transaction_status &s)
{
    uint8_t frame[wire::kFrameLen]{};
    wire::encode_transaction_status(s, frame);
    if (send_frame(cfg_.status_id, frame, sizeof frame) != 0)
        return false;
    count(&counters::status_sent);
    return true;
}

bool send_session()
{
    uint8_t frame[wire::kFrameLen]{};
    wire::encode_session_status(session::announcement(), frame);
    if (send_frame(cfg_.status_id, frame, sizeof frame) != 0)
        return false;
    count(&counters::sessions_sent);
    return true;
}

/* The receive-path forms. Same bytes, queued instead of sent. */
void queue_status(const wire::transaction_status &s)
{
    uint8_t frame[wire::kFrameLen]{};
    wire::encode_transaction_status(s, frame);
    queue_frame(cfg_.status_id, frame, false);
}

void queue_session()
{
    uint8_t frame[wire::kFrameLen]{};
    wire::encode_session_status(session::announcement(), frame);
    queue_frame(cfg_.status_id, frame, true);
}

/* The hooks the session layer gets. They are this module's own, forwarding to the binding's, so the
 * session layer never sees the runtime's context pointer and the binding never sees the session's
 * signatures. */
bool rt_permitted(void *)
{
    return hooks_.enumeration_permitted != nullptr && hooks_.enumeration_permitted(hooks_.ctx);
}

int rt_prove(void *, uint32_t epoch, tof_commissioning::outcome *out)
{
    return hooks_.prove != nullptr ? hooks_.prove(hooks_.ctx, epoch, out) : -ENODEV;
}

int rt_start(void *)
{
    return hooks_.start != nullptr ? hooks_.start(hooks_.ctx) : -ENODEV;
}

int rt_draw(void *, uint32_t *out)
{
    return hooks_.draw_token != nullptr ? hooks_.draw_token(hooks_.ctx, out) : -ENODEV;
}

void worker_entry(void *, void *, void *)
{
    for (;;) {
        (void)service_once(k_uptime_get());
        k_msleep(cfg_.poll_ms);
    }
}

} // namespace

int init(const config &cfg, const hooks &h)
{
    /* IDENTIFIERS ARE REQUIRED AND ARE CHECKED, because the alternative is a default -- and a
     * default identifier is a frame on somebody else's conversation. Zero is not one, and one value
     * for both would answer a request with a frame the sender reads back as a request. */
    if (cfg.request_id == 0 || cfg.status_id == 0 || cfg.request_id == cfg.status_id)
        return -EINVAL;
    /* Standard 11-bit identifiers: this contract's transport is CAN classic. A value that does not
     * fit is a configuration error to report now, not one for the driver to truncate later. */
    if (cfg.request_id > 0x7FFu || cfg.status_id > 0x7FFu)
        return -EINVAL;
    if (cfg.announce_period_ms == 0 || cfg.poll_ms == 0)
        return -EINVAL;
    if (h.send == nullptr || h.enumeration_permitted == nullptr || h.prove == nullptr ||
        h.start == nullptr || h.draw_token == nullptr)
        return -EINVAL;

    cfg_ = cfg;
    hooks_ = h;
    /* Initialised once. A work item that may be queued or running must not be re-initialised, and
     * the queue is purged rather than re-created for the same reason. */
    if (!tx_work_inited_) {
        k_work_init(&tx_work_, tx_work_handler);
        tx_work_inited_ = true;
    }
    k_msgq_purge(&tx_q_);
    {
        k_spinlock_key_t key{k_spin_lock(&lock_)};
        stats_ = counters{};
        k_spin_unlock(&lock_, key);
    }
    configured_ = false;
    session_ok_ = false;
    announced_once_ = false;
    last_announce_ms_ = 0;

    session::config sc{};
    sc.profile_enabled = cfg_.profile_enabled;
    sc.max_proof_attempts = cfg_.max_proof_attempts;
    sc.max_start_attempts = cfg_.max_start_attempts;
    const session::hooks sh{rt_draw, rt_permitted, rt_prove, rt_start, nullptr};
    session_ok_ = session::init(sc, sh);

    /* CONFIGURED EITHER WAY, and that is the fix for a host that cannot get out of a retransmit
     * loop. With no token the board announces nothing -- it has no session to announce -- but the
     * receive path stays up and answers `no_session`, which is terminal at the host and tells it to
     * stop and report. An earlier version dropped the frame instead, so a host holding a durable
     * pending request from a previous boot retransmitted into silence for ever. */
    configured_ = true;
    return session_ok_ ? 0 : -ENODEV;
}

bool has_session()
{
    return session_ok_;
}

void on_frame(uint32_t id, const uint8_t *data, size_t len)
{
    /* NOT gated on a session. A board with no token still answers -- see init(). */
    if (!configured_)
        return;
    if (id != cfg_.request_id) {
        /* Not ours. Counted rather than ignored silently, because a runtime that is being handed
         * the wrong identifier looks exactly like one that is being handed nothing. */
        count(&counters::frames_ignored);
        return;
    }
    count(&counters::frames_in);

    /* handle_request() takes a spinlock and touches a table; it does not block, and it is the only
     * work this path is allowed to do. The answers it produces are QUEUED -- see the transmit queue
     * above for why sending here is not an option on this board. */
    const session::rx_action a{session::handle_request(data, len)};
    if (a.send_status)
        queue_status(a.status);
    if (a.send_session)
        queue_session();
}

service_result service_once(int64_t now_ms)
{
    service_result out{};
    /* Nothing to work on without a session: every request was refused in the receive path, so
     * nothing can be queued and there is nothing to announce. */
    if (!configured_ || !session_ok_)
        return out;

    const session::worker_result w{session::worker_step()};
    if (w.send_status) {
        out.sent_status = send_status(w.status);
        out.terminal = true;
        count(&counters::transactions);
    }

    /* The announcement is on the same thread as the worker, so it does not go out while a
     * transaction is running -- and that is honest rather than unfortunate: `transaction_in_progress`
     * would be stale by the time the host read it, and the host does not need it to make progress.
     * What it needs is the token, which does not change while the board is up. */
    if (!announced_once_ || now_ms - last_announce_ms_ >= static_cast<int64_t>(cfg_.announce_period_ms)) {
        out.sent_session = send_session();
        last_announce_ms_ = now_ms;
        announced_once_ = true;
    }
    return out;
}

int start()
{
    if (!configured_ || !session_ok_)
        return -EPERM;

    /* CLAIMED UNDER THE LOCK, because "check then set" on a plain bool is two threads creating two
     * threads. The claim is taken before the thread exists, so the loser gets -EALREADY rather than
     * a second worker on the same job. */
    k_spinlock_key_t key{k_spin_lock(&lock_)};
    if (worker_started_) {
        k_spin_unlock(&lock_, key);
        return -EALREADY;
    }
    worker_started_ = true;
    k_spin_unlock(&lock_, key);

    k_thread_create(&worker_thread, worker_stack, K_THREAD_STACK_SIZEOF(worker_stack), worker_entry,
                    nullptr, nullptr, nullptr, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
    k_thread_name_set(&worker_thread, "tof_commission");
    return 0;
}

bool running()
{
    /* Under the lock, like the claim itself. start() writes this field from whatever thread called
     * it, so an unlocked read here is a data race in the same way the counters were -- and the
     * answer would be the one the reader's cache happened to hold. */
    k_spinlock_key_t key{k_spin_lock(&lock_)};
    const bool r{worker_started_};
    k_spin_unlock(&lock_, key);
    return r;
}

counters stats()
{
    /* A coherent snapshot rather than a field-by-field read, so a caller cannot see a total that
     * never existed. */
    k_spinlock_key_t key{k_spin_lock(&lock_)};
    const counters c{stats_};
    k_spin_unlock(&lock_, key);
    return c;
}

} // namespace lexxhard::tof_commission_runtime

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
