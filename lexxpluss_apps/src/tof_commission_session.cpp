/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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

/* ONE SLOT PER SEQUENCE NUMBER, AND THE TABLE IS INDEXED BY IT.
 *
 * Still bounded, still never evicting -- evicting an entry re-opens replay of exactly the request it
 * described, and reusing a sequence would make two different requests share an identity. What
 * changed is the bound. It was 16, justified as "far more attempts than a healthy machine makes",
 * and that justification counted the wrong thing: the table is consumed by REQUESTS, not by
 * attempts, and a request that never ran consumes a slot too. A `busy_chain` refusal has to be
 * recorded -- see the claim-before-refuse comment in handle_request() for the retransmission that
 * ran when it was not -- so sixteen retries against a machine that is not quiescent exhausted the
 * table and ended commissioning for the boot without a single proof having run.
 *
 * Sizing it from a request rate would have been another guess. The sequence number is eight bits, so
 * 256 is the whole space a host can address in one session, and at that size the table cannot fill
 * before the host has used every identity available to it. "Table full" and "sequence space
 * exhausted" become the same condition, which is what the wire's result code already called it.
 *
 * It also lets the table be indexed by the sequence number instead of searched, which removes two
 * linear scans from a CAN receive callback holding a spinlock. At 12 bytes an entry this is 3,072 B
 * of bss against 192 B before; the chain image measures 54.8% of its RAM region in use. */
constexpr uint16_t kTableSize{256};
static_assert(kTableSize == 1U << (8U * sizeof(wire::request::seq)),
              "the table is indexed by the wire's sequence number and must cover all of it");

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
uint16_t used_entries_{0};

/* One slot. A second request while one is in flight is `busy_chain`, and an exact retransmission of
 * the in-flight one is answered with its current phase. */
bool running_{false};
uint16_t running_index_{0};

/* WHETHER A WORKER HAS TAKEN THE JOB, which is not the same fact as `running_` and is the reason
 * this exists. `running_` says a request is queued; it stays true for the whole transaction, so two
 * threads calling worker_step() both saw it, both took the same job out of the table, and both ran
 * the proof -- one chain enumerated twice under ONE host-issued epoch, and two terminal statuses for
 * one sequence number. (Nothing here issues an epoch: both runs would carry the one the request
 * named, and the second would meet the authority's own refusal at commit.) The claim is taken under
 * the same lock as the job and is released only after the terminal status has been written, so there
 * is no instant at which the job is unclaimed and unfinished. */
bool worker_active_{false};

/* Where the sequencer picks up the epoch. It is the queued request's, never generated here -- there
 * is no firmware-side issuer on this branch and there is not meant to be. */
uint32_t pending_epoch_{0};
tof_commissioning::outcome last_outcome_{};

entry *find(uint8_t seq)
{
    entry &e{table_[seq]};
    return e.used ? &e : nullptr;
}

entry *claim(uint8_t seq, wire::opcode op, uint8_t epoch)
{
    entry &e{table_[seq]};
    /* Unreachable: every caller runs find() first and returns on a hit, and this is the only slot
     * this sequence number can occupy. Kept as a refusal rather than an assertion because the
     * alternative to refusing would be overwriting a record somebody may still retransmit. */
    if (e.used)
        return nullptr;
    e = entry{};
    e.used = true;
    e.seq = seq;
    e.op = op;
    e.wire_epoch = epoch;
    ++used_entries_;
    return &e;
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
    worker_active_ = false;
    token_ = 0;
    has_session_ = false;

    has_proven_ = false;
    proven_epoch_ = 0;

    /* Zero is reserved as "no token", so a draw that yields it is retried once before the subsystem
     * gives up -- a single zero from a healthy generator is an ordinary sample, not a fault. Two in
     * a row is treated as no entropy at all. */
    /* EVERY HOOK IS CHECKED HERE, not where it is called, because #112's own wiring check cannot
     * see them. The sequencer is handed ac_permitted/ac_acquire_epoch/ac_prove/ac_start, which are
     * this file's adapters and are never null, so ac::wired() is satisfied by construction however
     * little is actually bound underneath. What each missing hook produced instead was a plausible
     * operational answer rather than a fault: a missing enumeration_permitted made ac_permitted
     * return false and the host read `not_permitted` forever, which is indistinguishable from a
     * machine that is simply never quiescent; a missing prove spent an attempt from the proof budget
     * before returning -ENODEV, because the sequencer increments the counter before the call; and a
     * missing start was found only after a real proof had run, installed a mapping and spent the
     * epoch, which is exactly the expensive discovery #112's budget comment exists to avoid.
     *
     * Refusing the session instead leaves ac::state::misconfigured, which the wire already has a
     * result for, and costs a configuration mistake nothing but a clear answer. */
    uint32_t drawn{0};
    if (hooks_.draw_token == nullptr || hooks_.enumeration_permitted == nullptr ||
        hooks_.prove == nullptr || hooks_.start == nullptr ||
        hooks_.installed_mapping == nullptr)
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
    running_index_ = static_cast<uint16_t>(e - table_);
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
    if (worker_active_) {
        /* Another thread has this job. Reported as running and NOTHING else: no transaction, and no
         * status frame -- a second terminal status for one sequence number would tell the host its
         * request finished twice, and the second one would be describing a transaction this call
         * never ran. */
        k_spin_unlock(&lock_, key);
        out.state = worker_state::running;
        return out;
    }
    worker_active_ = true;
    const uint16_t index{running_index_};
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

    /* THE AUTHORITY IS ASKED, NOT THIS FILE'S MEMORY OF IT, and that is a correction.
     *
     * The gate used to compare the request's epoch against has_proven_/proven_epoch_, which only
     * ac_prove() writes. Anything that installed or lost a mapping outside this session was
     * therefore invisible to it: `tof cliff prove <epoch>` from the shell goes straight to
     * tof_commissioning::prove() and installs a different epoch, and a mapping can go LOST on its
     * own. A later request naming the OLD epoch then passed the gate, ac::step() answered
     * `already_started`, and the host was told done/ok while something else -- or nothing -- was
     * installed. That is the exact lie the paragraph below says this gate exists to prevent, so the
     * gate has to read the one place that knows.
     *
     * has_proven_/proven_epoch_ are kept, but only as the sequencer's own account of what IT proved;
     * they no longer decide anything on their own.
     *
     * Asked through a hook rather than by calling tof_authority::current() directly, for the reason
     * every other dependency here is injected: this module is tested on the host, where the
     * authority is not linked, and the cases worth testing are exactly the ones a real authority
     * makes hard to produce -- somebody else's epoch installed, or a mapping that went LOST. */
    const ac::state st{ac::current()};
    const bool holds_proof{st == ac::state::proven || st == ac::state::started};
    uint8_t in_force_epoch{0};
    const bool authority_proven{hooks_.installed_mapping(hooks_.ctx, &in_force_epoch)};
    const bool authority_holds_this_epoch{authority_proven && in_force_epoch == req_epoch};

    /* AND THE TWO VIEWS MUST AGREE WITH EACH OTHER, which asking the authority alone does not get.
     *
     * The first version of this fix gated on "the authority holds this request's epoch" and nothing
     * else, and that left a path open. Session proves and starts epoch 7, so the sequencer sits in
     * `started`. The shell then proves epoch 9 outside the session, which silences acquisition on
     * its way through. The host asks for epoch 9: the authority does hold 9, so the gate let it
     * past -- and ac::step() answered `already_started` from a state that belongs to epoch 7, with
     * nothing re-started. The host was told done/ok for a mapping that was never started.
     *
     * So the authority's mapping must not be allowed to endorse a `started` that predates it. The
     * sequencer's own record of what IT proved is the link: when the epoch in force is not the one
     * this session proved, the sequencer's state says nothing about what is installed, and the
     * request is refused rather than answered from it. */
    const bool views_agree{authority_proven && has_proven_ && proven_epoch_ == in_force_epoch};

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
    if (holds_proof && !(authority_holds_this_epoch && views_agree)) {
        /* The sequencer will not prove anything else this boot, so whatever is installed is what
         * this request would be answered about. If that is not this request's epoch -- a different
         * epoch, or no mapping at all because it went LOST -- or if it is not the epoch this
         * session proved, then the request cannot be answered with done/ok. It is reported as a
         * disagreement about the epoch because that is what it is from the host's side: the ordinal
         * it named is not the one the running sequence belongs to.
         *
         * WHAT THIS STILL DOES NOT CHECK, so that nobody reads more into it: that the acquisition
         * thread is actually running. The sequencer's `started` is taken at its word once the two
         * views agree, and a mapping that is in force under the right epoch with acquisition
         * stopped underneath it would still be answered done. Detecting that needs the acquisition
         * state, which this module is not given and which the wiring branch should supply.
         *
         * AND THAT GAP HAS A REAL PATH, which an earlier version of this comment denied by claiming
         * the shell could not stop acquisition without moving the epoch. It can.
         * tof_commissioning::prove() quiesces at STEP 1 and only then takes the chain at STEP 2, so
         * a run that refuses there -- `chain_busy`, and the early refusals after it share the shape
         * -- returns with acquisition STOPPED, no mapping revoked and the epoch unchanged. Both
         * views then still agree, this gate passes, ac::step() answers already_started, and the
         * host is told done for a chain that is not acquiring. Nothing here can see it. The
         * regression belongs with the branch that can ask about acquisition. */
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
        /* WHETHER THIS TRANSACTION RAN ANYTHING, measured rather than inferred from the result.
         *
         * The wire says `done` carries the outcome and `refused` carries why it never ran, and the
         * phase used to be left at its `refused` default for everything except started and
         * already_started -- so proof_failed, start_failed, attempts_exhausted and the
         * busy_at_commit that map_result() produces all went out as "never ran". A host written
         * against the header read those as costing it nothing, when busy_at_commit's own comment
         * says the host owes a new ordinal for it.
         *
         * The result code alone cannot answer the question, which is why this is a measurement:
         * attempts_exhausted comes back BOTH from a proof that just failed as the last of its
         * budget and from a step that found the budget already spent and did nothing. The
         * sequencer's counters tell those apart, and nothing else here can. */
        const uint8_t attempts_before{ac::attempts_used()};
        const uint8_t starts_before{ac::start_attempts_used()};
        const ac::step_result step{ac::step()};
        const bool proof_ran{ac::attempts_used() != attempts_before};
        const bool ran{proof_ran || ac::start_attempts_used() != starts_before};

        switch (step) {
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
            /* THE REASON THE LAST PROOF FAILED, which used to be dropped. #112 returns
             * attempts_exhausted rather than proof_failed when the proof that just failed was the
             * last in the budget, and this branch did not read last_outcome_ -- so the host got
             * not_started/none for a transaction that had run a whole walk. With
             * max_proof_attempts = 1 that was every proof failure there is. The result stays
             * attempts_exhausted, because the budget is the fact the host must act on; the stage
             * and detail now say what went wrong while spending it. */
            if (proof_ran) {
                const map::outcome o{map::map_result(last_outcome_)};
                v.stage = o.stage;
                v.detail = o.detail;
            }
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

        /* Set once, from the measurement, rather than per case. `started` and `already_started`
         * have already said `done` for themselves: the first ran, and the second is the one result
         * that is `done` without running anything, because the end state the host asked for holds.
         *
         * Everything else is `done` exactly when this step consumed an attempt. That includes the
         * failures -- a proof that ran and failed is an outcome, not a refusal -- and excludes the
         * cases that never reached a hook: not_permitted, disabled, misconfigured, no_epoch, a
         * budget found already spent, and a start budget with nothing left to attempt. */
        if (v.ph != wire::phase::done)
            v.ph = ran ? wire::phase::done : wire::phase::refused;
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
    /* RELEASED HERE, AFTER THE TABLE HOLDS THE OUTCOME, and not a line earlier. Clearing it before
     * the status was written would leave a window in which the job is neither claimed nor finished,
     * which is the window this flag exists to close. */
    worker_active_ = false;
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
