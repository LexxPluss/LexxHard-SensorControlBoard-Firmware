/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commission_session.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include "tof_auto_commission.hpp"
#include "tof_commission_map.hpp"

namespace lexxhard::tof_commission {

namespace ac = tof_auto_commission;
namespace map = tof_commission_map;

namespace {

/* Bounded, and refusing when full rather than evicting. Evicting an entry re-opens replay of exactly
 * the request it described, and wrapping the sequence would make two different requests share an
 * identity -- so a full table ends commissioning for this boot, which a boot clears anyway because a
 * boot changes the session. Sized for far more attempts than a healthy machine makes. */
constexpr uint8_t kTableSize{16};

struct entry {
    bool used{false};
    uint8_t seq{0};
    wire::opcode op{wire::opcode::prove_and_start};
    uint8_t wire_epoch{0};
    bool terminal{false};
    wire::transaction_status status{};
};

/* RX, the worker and the announcement sender all touch this state from different threads. The lock
 * is held only for short, allocation-free stretches and NEVER across a transaction: the worker takes
 * the job under it, releases it, runs the hundreds of milliseconds of proof outside, and takes it
 * again to commit the terminal status. Holding it across the proof would block the CAN callback for
 * the whole transaction, which is the thing the split exists to prevent. */
struct k_spinlock lock_;

config cfg_{};
hooks hooks_{};
uint32_t token_{0};
bool has_session_{false};
counters stats_{};

/* What the AUTOMATIC path has proven, and under which epoch. Needed because `already_started` alone
 * cannot answer a request: it says acquisition is running, not that it is running under the epoch
 * this request asked for. */
bool has_proven_{false};
uint8_t proven_epoch_{0};

entry table_[kTableSize]{};
uint8_t used_entries_{0};

/* One slot. A second request while one is in flight is `busy_chain`, and an exact retransmission of
 * the in-flight one is answered with its current phase. */
bool running_{false};
uint8_t running_index_{0};

/* Where the sequencer picks up the epoch. It is the queued request's, never generated here -- there
 * is no firmware-side issuer on this branch and there is not meant to be. */
uint32_t pending_epoch_{0};
tof_commissioning::outcome last_outcome_{};

entry *find(uint8_t seq)
{
    for (auto &e : table_)
        if (e.used && e.seq == seq)
            return &e;
    return nullptr;
}

entry *claim(uint8_t seq, wire::opcode op, uint8_t epoch)
{
    for (auto &e : table_) {
        if (e.used)
            continue;
        e = entry{};
        e.used = true;
        e.seq = seq;
        e.op = op;
        e.wire_epoch = epoch;
        ++used_entries_;
        return &e;
    }
    return nullptr;
}

wire::transaction_status refusal(uint8_t seq, uint8_t epoch, wire::result res)
{
    wire::transaction_status s{};
    s.seq = seq;
    s.wire_epoch = epoch;
    s.ph = wire::phase::refused;
    s.res = res;
    return s;
}

/* --- the sequencer's hooks, bound once --- */

bool ac_permitted(void *)
{
    return hooks_.enumeration_permitted != nullptr && hooks_.enumeration_permitted(hooks_.ctx);
}

int ac_acquire_epoch(void *, uint32_t *out)
{
    *out = pending_epoch_;
    return 0;
}

int ac_prove(void *, uint32_t epoch)
{
    last_outcome_ = tof_commissioning::outcome{};
    const int rc{hooks_.prove != nullptr ? hooks_.prove(hooks_.ctx, epoch, &last_outcome_) : -ENODEV};
    if (rc == 0) {
        /* Recorded here rather than inferred from the sequencer's state, because the sequencer knows
         * that A mapping is proven and not which epoch proved it. */
        has_proven_ = true;
        proven_epoch_ = static_cast<uint8_t>(epoch);
    }
    return rc;
}

int ac_start(void *)
{
    return hooks_.start != nullptr ? hooks_.start(hooks_.ctx) : -ENODEV;
}

} // namespace

bool init(const config &cfg, const hooks &h)
{
    cfg_ = cfg;
    hooks_ = h;
    stats_ = counters{};
    for (auto &e : table_)
        e = entry{};
    used_entries_ = 0;
    running_ = false;
    token_ = 0;
    has_session_ = false;

    has_proven_ = false;
    proven_epoch_ = 0;

    /* Zero is reserved as "no token", so a draw that yields it is retried once before the subsystem
     * gives up -- a single zero from a healthy generator is an ordinary sample, not a fault. Two in
     * a row is treated as no entropy at all. */
    uint32_t drawn{0};
    if (hooks_.draw_token == nullptr)
        return false;
    if (hooks_.draw_token(hooks_.ctx, &drawn) != 0 || drawn == 0) {
        drawn = 0;
        if (hooks_.draw_token(hooks_.ctx, &drawn) != 0 || drawn == 0)
            return false;
    }
    token_ = drawn;
    has_session_ = true;

    /* Initialised ONCE, here, so the budgets accumulate across every request this boot sees. */
    ac::config seq{};
    seq.enabled = cfg_.profile_enabled;
    seq.max_attempts = cfg_.max_proof_attempts;
    seq.max_start_attempts = cfg_.max_start_attempts;
    ac::hooks ah{ac_permitted, ac_acquire_epoch, ac_prove, ac_start, nullptr};
    ac::init(seq, ah);
    return true;
}

bool has_session()
{
    return has_session_;
}

uint32_t session_token()
{
    return token_;
}

wire::session_status announcement()
{
    wire::session_status s{};
    /* Under the lock for `transaction_in_progress`: it is the one field the worker changes while an
     * announcement may be being built, and a torn answer here tells the host the chain is free when a
     * transaction is running. */
    k_spinlock_key_t key{k_spin_lock(&lock_)};
    s.session_token = token_;
    s.profile_enabled = cfg_.profile_enabled;
    s.transaction_in_progress = running_;
    k_spin_unlock(&lock_, key);
    return s;
}

rx_action handle_request(const uint8_t *data, size_t len)
{
    rx_action out{};

    /* Decoding touches no shared state, so it happens before the lock. */
    wire::request req{};
    const wire::decode_error de{wire::decode_request(data, len, req)};

    k_spinlock_key_t key{k_spin_lock(&lock_)};

    /* ORDER IS LOAD-BEARING: length, version, session token, opcode, profile, and only then the
     * table. A stale-session frame carries a sequence number that means nothing here, and recording
     * it before the token was checked would let a frame from a finished boot occupy a sequence the
     * live host still needs. The opcode is checked AFTER the token for the same reason. */
    switch (de) {
    case wire::decode_error::none:
        break;
    case wire::decode_error::bad_length:
        /* Discarded whole. No result frame -- a result answers a request, and this is not one we can
         * identify. The session frame is re-announced instead, because a host whose frames are being
         * dropped needs the session and version it should be speaking. */
        ++stats_.discarded_bad_length;
        out.send_session = has_session_;
        k_spin_unlock(&lock_, key);
        return out;
    case wire::decode_error::bad_version:
        ++stats_.discarded_bad_version;
        out.send_session = has_session_;
        k_spin_unlock(&lock_, key);
        return out;
    case wire::decode_error::bad_kind:
    case wire::decode_error::reserved_not_zero:
    default:
        out.send_status = true;
        out.status = refusal(0, 0, wire::result::internal_error);
        k_spin_unlock(&lock_, key);
        return out;
    }

    if (!has_session_) {
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::no_session);
        k_spin_unlock(&lock_, key);
        return out;
    }

    if (req.session_token != token_) {
        /* Answered, because the frame is well-formed enough to answer -- and the table is NOT
         * touched, which is the whole reason the token is checked before it. */
        ++stats_.refused_stale_session;
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::stale_session);
        out.send_session = true;
        k_spin_unlock(&lock_, key);
        return out;
    }

    if (!wire::is_known_opcode(req.raw_op)) {
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::bad_opcode);
        k_spin_unlock(&lock_, key);
        return out;
    }
    const wire::opcode op{static_cast<wire::opcode>(req.raw_op)};

    if (!cfg_.profile_enabled) {
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::disabled);
        k_spin_unlock(&lock_, key);
        return out;
    }

    if (entry *e = find(req.seq); e != nullptr) {
        if (e->op != op || e->wire_epoch != req.wire_epoch) {
            /* Not an honest retransmission, and guessing which one the host meant is how a stale
             * frame ends up re-proving a working chain. Refused for the life of the session. */
            ++stats_.refused_seq_conflict;
            out.send_status = true;
            out.status = refusal(req.seq, req.wire_epoch, wire::result::seq_conflict);
            k_spin_unlock(&lock_, key);
            return out;
        }
        /* An exact retransmission: replayed from the table, terminal or not. The transaction does not
         * run again, no chain is re-enumerated and no epoch is consumed. */
        ++stats_.replayed_from_table;
        out.send_status = true;
        out.status = e->status;
        k_spin_unlock(&lock_, key);
        return out;
    }

    entry *e{claim(req.seq, op, req.wire_epoch)};
    if (e == nullptr) {
        /* Full. Refusing until reboot is the only option that keeps the guarantee -- and this one
         * refusal is NOT cached, because there is nowhere to cache it; a retransmission gets the
         * same answer by taking the same path, which is the one case where recomputing is
         * equivalent to replaying. */
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::seq_space_exhausted);
        k_spin_unlock(&lock_, key);
        return out;
    }

    e->status = wire::transaction_status{};
    e->status.seq = req.seq;
    e->status.wire_epoch = req.wire_epoch;

    if (running_) {
        /* CLAIMED FIRST, THEN REFUSED, and the order matters. An earlier version checked `running_`
         * before claiming, so the busy answer left no entry -- and the identical frame, retransmitted
         * after the in-flight transaction finished, was treated as a new request and RAN. Caching it
         * makes a retransmission replay busy, which is what the protocol says a retransmission does. */
        e->status.ph = wire::phase::refused;
        e->status.res = wire::result::busy_chain;
        e->terminal = true;
        out.send_status = true;
        out.status = e->status;
        k_spin_unlock(&lock_, key);
        return out;
    }

    e->status.ph = wire::phase::accepted;
    e->status.res = wire::result::ok;

    pending_epoch_ = req.wire_epoch;
    running_ = true;
    running_index_ = static_cast<uint8_t>(e - table_);
    ++stats_.accepted;

    out.queued = true;
    out.send_status = true;
    out.status = e->status;
    k_spin_unlock(&lock_, key);
    return out;
}

worker_result worker_step()
{
    worker_result out{};

    /* THE LOCK IS TAKEN THREE TIMES AND HELD ACROSS NONE OF THE WORK. Take the job, release, run the
     * transaction -- hundreds of milliseconds of I2C with the chain mutex held -- and take the lock
     * again to publish the terminal status. Holding it across the proof would leave CAN reception
     * spinning with interrupts masked for the length of an enumeration. */
    k_spinlock_key_t key{k_spin_lock(&lock_)};
    if (!running_) {
        k_spin_unlock(&lock_, key);
        out.state = worker_state::idle;
        return out;
    }
    const uint8_t index{running_index_};
    const wire::opcode op{table_[index].op};
    const uint8_t req_epoch{table_[index].wire_epoch};
    k_spin_unlock(&lock_, key);

    out.state = worker_state::running;

    struct verdict {
        wire::phase ph{wire::phase::refused};
        wire::result res{wire::result::internal_error};
        wire::wire_stage stage{wire::wire_stage::not_started};
        wire::wire_detail detail{wire::wire_detail::none};
    };
    verdict v{};

    /* `has_proven_`/`proven_epoch_` are written by ac_prove() and read here, both on this thread and
     * nowhere else, so they need no lock -- and they are read BEFORE ac::step(), because the whole
     * point is to decide whether the sequencer may be stepped at all. */
    const ac::state st{ac::current()};
    const bool holds_proof{st == ac::state::proven || st == ac::state::started};

    /* THE OPCODE BOUNDARY, AND IT IS ENFORCED HERE RATHER THAN INSIDE THE SEQUENCER.
     *
     * The sequencer has one entry point and it decides what to do from its own state, not from the
     * request: stepped while `waiting` it runs a full proof, stepped while `proven` it only starts.
     * So a `start_only` that reached ac::step() with nothing proven would re-enumerate the chain --
     * exactly what the opcode exists to promise it will not do -- and report `started` as if the host
     * had asked for it. An earlier version did precisely that, because worker_step() never read
     * `entry.op` at all.
     *
     * The epoch check is the same defect seen from the accounting side. Once a proof is held, the
     * sequencer will not prove anything else this boot: stepped again it starts, or reports
     * `already_started`, and in both cases the epoch it is running under is the one it proved, not the
     * one this request named. Answering `done`/`ok` there tells the host its ordinal was accepted when
     * a different ordinal is installed, which is the one lie the host's `accepted` field cannot
     * recover from. `epoch_mismatch` is terminal on the host side and asks for no retry. */
    bool gated{false};
    if (holds_proof && (!has_proven_ || proven_epoch_ != req_epoch)) {
        gated = true;
    } else if (op == wire::opcode::start_only && !holds_proof &&
               st != ac::state::disabled && st != ac::state::misconfigured) {
        /* Nothing is proven, so no epoch is the proven one. `disabled` and `misconfigured` are let
         * through because ac::step() answers those without reaching for a hook, and a configuration
         * fault reported as an epoch disagreement would send somebody looking at the wrong thing. */
        gated = true;
    }

    if (gated) {
        v.ph = wire::phase::refused;
        v.res = wire::result::epoch_mismatch;
        v.stage = wire::wire_stage::not_started;
        v.detail = wire::wire_detail::none;
    } else {
        switch (const ac::step_result step{ac::step()}; step) {
        case ac::step_result::started:
        case ac::step_result::already_started: {
            /* The desired end state holds, AND it holds for this request's epoch -- the gate above is
             * what makes the second half of that sentence true. `already_started` is reported the same
             * way as `started` on purpose: a request that finds acquisition running under its own
             * epoch has got what it asked for, and reporting a failure would push the host into
             * retrying something that is done. */
            const map::outcome o{map::map_stage(tof_commissioning::stage::none)};
            v.ph = wire::phase::done;
            v.res = o.res;
            v.stage = o.stage;
            v.detail = o.detail;
            break;
        }
        case ac::step_result::proof_failed: {
            /* The detail comes from the transaction's own outcome, translated by the mapper. Nothing
             * here decides what a failed walk means. */
            const map::outcome o{map::map_result(last_outcome_)};
            v.res = o.res;
            v.stage = o.stage;
            v.detail = o.detail;
            break;
        }
        case ac::step_result::attempts_exhausted:
            v.res = wire::result::attempts_exhausted;
            break;
        case ac::step_result::start_failed:
            v.res = wire::result::start_failed;
            v.stage = wire::wire_stage::acquisition_start;
            v.detail = wire::wire_detail::acquisition_refused;
            break;
        case ac::step_result::start_attempts_exhausted:
            v.res = wire::result::attempts_exhausted;
            v.stage = wire::wire_stage::acquisition_start;
            v.detail = wire::wire_detail::acquisition_refused;
            break;
        case ac::step_result::not_permitted:
            v.res = wire::result::not_permitted;
            break;
        case ac::step_result::no_epoch:
            /* Unreachable: the epoch comes from the queued request. Terminal rather than ignored,
             * because a silent loop here would spin the worker on a request that can never
             * progress. */
            v.res = wire::result::internal_error;
            break;
        case ac::step_result::disabled:
            v.res = wire::result::disabled;
            break;
        case ac::step_result::misconfigured:
            v.res = wire::result::misconfigured;
            break;
        }
    }

    key = k_spin_lock(&lock_);
    entry &e{table_[index]};
    e.status.ph = v.ph;
    e.status.res = v.res;
    e.status.stage = v.stage;
    e.status.detail = v.detail;
    /* Every outcome above is terminal. There are no intermediate phases on this path: `proving`,
     * `proven` and `starting` exist in the wire enumeration as OPTIONAL DIAGNOSTICS, and this
     * firmware does not emit them, because the transaction is a single blocking call with no
     * observable interior. A host must therefore not treat their absence as a fault -- it waits for a
     * terminal phase or its own timeout, which is what it would do anyway. */
    e.terminal = true;
    running_ = false;
    out.send_status = true;
    out.status = e.status;
    out.state = worker_state::idle;
    k_spin_unlock(&lock_, key);
    return out;
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

} // namespace lexxhard::tof_commission

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
