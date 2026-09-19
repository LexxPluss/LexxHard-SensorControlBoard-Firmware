/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commission_session.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

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

config cfg_{};
hooks hooks_{};
uint32_t token_{0};
bool has_session_{false};
counters stats_{};

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
    return hooks_.prove != nullptr ? hooks_.prove(hooks_.ctx, epoch, &last_outcome_) : -ENODEV;
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

    uint32_t drawn{0};
    if (hooks_.draw_token == nullptr || hooks_.draw_token(hooks_.ctx, &drawn) != 0 || drawn == 0)
        return false;
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
    s.session_token = token_;
    s.profile_enabled = cfg_.profile_enabled;
    s.transaction_in_progress = running_;
    return s;
}

rx_action handle_request(const uint8_t *data, size_t len)
{
    rx_action out{};

    /* ORDER IS LOAD-BEARING. Length, then version, then the session token, then the opcode, then the
     * profile, and only then the table. A stale-session frame carries a sequence number that means
     * nothing here, and recording it before the token was checked would let a frame from a finished
     * boot occupy a sequence the live host still needs. */
    wire::request req{};
    switch (wire::decode_request(data, len, req)) {
    case wire::decode_error::none:
        break;
    case wire::decode_error::bad_length:
        /* Discarded whole. No result frame -- a result answers a request, and this is not one we can
         * identify. The session frame is re-announced instead, because a host whose frames are being
         * dropped needs the session and version it should be speaking. */
        ++stats_.discarded_bad_length;
        out.send_session = has_session_;
        return out;
    case wire::decode_error::bad_version:
        ++stats_.discarded_bad_version;
        out.send_session = has_session_;
        return out;
    case wire::decode_error::bad_opcode:
        out.send_status = true;
        out.status = refusal(data != nullptr && len == wire::kFrameLen ? data[2] : 0,
                             data != nullptr && len == wire::kFrameLen ? data[3] : 0,
                             wire::result::bad_opcode);
        return out;
    case wire::decode_error::bad_kind:
    case wire::decode_error::reserved_not_zero:
    default:
        out.send_status = true;
        out.status = refusal(0, 0, wire::result::internal_error);
        return out;
    }

    if (!has_session_) {
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::no_session);
        return out;
    }

    if (req.session_token != token_) {
        /* Answered, because the frame is well-formed enough to answer -- and the table is NOT
         * touched, which is the whole reason the token is checked before it. */
        ++stats_.refused_stale_session;
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::stale_session);
        out.send_session = true;
        return out;
    }

    if (!cfg_.profile_enabled) {
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::disabled);
        return out;
    }

    if (entry *e = find(req.seq); e != nullptr) {
        if (e->op != req.op || e->wire_epoch != req.wire_epoch) {
            /* Not an honest retransmission, and guessing which one the host meant is how a stale
             * frame ends up re-proving a working chain. Refused for the life of the session. */
            ++stats_.refused_seq_conflict;
            out.send_status = true;
            out.status = refusal(req.seq, req.wire_epoch, wire::result::seq_conflict);
            return out;
        }
        /* An exact retransmission: replayed from the table, terminal or not. The transaction does not
         * run again, no chain is re-enumerated and no epoch is consumed. */
        ++stats_.replayed_from_table;
        out.send_status = true;
        out.status = e->status;
        return out;
    }

    if (running_) {
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::busy_chain);
        return out;
    }

    entry *e{claim(req.seq, req.op, req.wire_epoch)};
    if (e == nullptr) {
        /* Full. Refusing until reboot is the only option that keeps the guarantee. */
        out.send_status = true;
        out.status = refusal(req.seq, req.wire_epoch, wire::result::seq_space_exhausted);
        return out;
    }

    e->status = wire::transaction_status{};
    e->status.seq = req.seq;
    e->status.wire_epoch = req.wire_epoch;
    e->status.ph = wire::phase::accepted;
    e->status.res = wire::result::ok;

    pending_epoch_ = req.wire_epoch;
    running_ = true;
    running_index_ = static_cast<uint8_t>(e - table_);
    ++stats_.accepted;

    out.queued = true;
    out.send_status = true;
    out.status = e->status;
    return out;
}

worker_result worker_step()
{
    worker_result out{};
    if (!running_) {
        out.state = worker_state::idle;
        return out;
    }

    entry &e{table_[running_index_]};
    out.state = worker_state::running;

    const ac::step_result step{ac::step()};

    const auto terminal = [&](wire::result res, wire::wire_stage st, wire::wire_detail d) {
        e.status.ph = wire::phase::refused;
        e.status.res = res;
        e.status.stage = st;
        e.status.detail = d;
        e.terminal = true;
        running_ = false;
    };

    switch (step) {
    case ac::step_result::started:
    case ac::step_result::already_started: {
        /* The desired end state holds. `already_started` is reported the same way on purpose: a
         * request that finds acquisition already running has got what it asked for, and reporting a
         * failure would push the host into retrying something that is done. */
        const map::outcome o{map::map_stage(tof_commissioning::stage::none)};
        e.status.ph = wire::phase::done;
        e.status.res = o.res;
        e.status.stage = o.stage;
        e.status.detail = o.detail;
        e.terminal = true;
        running_ = false;
        break;
    }
    case ac::step_result::proof_failed: {
        /* The detail comes from the transaction's own outcome, translated by the mapper. Nothing
         * here decides what a failed walk means. */
        const map::outcome o{map::map_result(last_outcome_)};
        terminal(o.res, o.stage, o.detail);
        break;
    }
    case ac::step_result::attempts_exhausted:
        terminal(wire::result::attempts_exhausted, wire::wire_stage::not_started,
                 wire::wire_detail::none);
        break;
    case ac::step_result::start_failed:
        terminal(wire::result::start_failed, wire::wire_stage::acquisition_start,
                 wire::wire_detail::acquisition_refused);
        break;
    case ac::step_result::start_attempts_exhausted:
        terminal(wire::result::attempts_exhausted, wire::wire_stage::acquisition_start,
                 wire::wire_detail::acquisition_refused);
        break;
    case ac::step_result::not_permitted:
        terminal(wire::result::not_permitted, wire::wire_stage::not_started, wire::wire_detail::none);
        break;
    case ac::step_result::no_epoch:
        /* Unreachable: the epoch comes from the queued request. Terminal rather than ignored,
         * because a silent loop here would spin the worker on a request that can never progress. */
        terminal(wire::result::internal_error, wire::wire_stage::not_started,
                 wire::wire_detail::none);
        break;
    case ac::step_result::disabled:
        terminal(wire::result::disabled, wire::wire_stage::not_started, wire::wire_detail::none);
        break;
    case ac::step_result::misconfigured:
        terminal(wire::result::misconfigured, wire::wire_stage::not_started,
                 wire::wire_detail::none);
        break;
    }

    out.send_status = true;
    out.status = e.status;
    if (e.terminal)
        out.state = worker_state::idle;
    return out;
}

counters stats()
{
    return stats_;
}

} // namespace lexxhard::tof_commission

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
