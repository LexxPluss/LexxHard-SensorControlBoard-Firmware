/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_acquisition.hpp"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "tof_chain_controller.hpp"

LOG_MODULE_REGISTER(tof_acq, CONFIG_LOG_DEFAULT_LEVEL);

namespace lexxhard::tof_acq {

namespace {

config cfg_;
bool configured_{false};
/* Set by a successful init(), cleared only by teardown(). While it is set, a second init() is
 * refused: the health work item reads cfg_ from another context, and overwriting the
 * configuration underneath it is a data race with a function pointer in it. */
bool active_{false};

/* SYNCHRONISATION RULE, and it is one rule rather than two.
 *
 * running_ and in_cycle_ are touched by more than one context -- the acquisition side and the
 * commissioning side -- so every read and every write of them happens with the chain lock held.
 * No exceptions for "advisory" or "optimisation" reads: they are plain bools, so an unsynchronised
 * read while another thread writes one under the lock is a data race, which is undefined behaviour
 * rather than a stale value, and a later check under the lock does not repair it.
 *
 * configured_ and active_ are different: they are written only by init() and teardown(), which are
 * lifecycle calls made from ONE context. That is what makes reading them outside the lock sound,
 * and it is a constraint on whoever wires up the runtime rather than a property of this file -- if a
 * future acquisition thread ever calls init(), teardown() or try_stop() itself, they need the same
 * treatment as the two above. */
bool running_{false};
bool in_cycle_{false};

/* The acquisition thread, and the fact of its existence.
 *
 * thread_active_ and owner_ are written only by start() and by the thread's own
 * exit path, both before/after the thread can be contended, and read by the
 * ownership guard. owner_ is what makes the guard possible at all: "is this
 * call coming from the thread that owns the ULD" cannot be answered by a flag.
 */
k_thread thread_;
k_tid_t owner_{nullptr};
/* Two flags, because they answer different questions. thread_active_ is "is a thread driving the
 * ULD right now", which is what the ownership guard and try_stop() need. thread_created_ is "does
 * the kernel thread object still belong to a thread nobody has joined", which is what makes
 * restarting safe: k_thread_create() on an object whose previous thread has not been joined reuses a
 * live kernel structure. The thread itself clears the first; only join() clears the second. */
bool thread_active_{false};
bool thread_created_{false};
thread_config tcfg_{};
atomic_t stop_requested_{ATOMIC_INIT(0)};
/* Doubles as the cadence sleep. A stop request gives it, so the thread leaves its inter-cycle wait
 * immediately instead of finishing a full period first -- the difference between commissioning
 * waiting one cadence and waiting for a timeout it then reports as a refusal. */
K_SEM_DEFINE(stop_sem_, 0, 1);
atomic_t foreign_calls_{ATOMIC_INIT(0)};

/* CUMULATIVE per-source observation, and the reason it cannot live in source_facts: that struct is
 * PER CYCLE and cleared before every read, so it can say "this cycle failed" but never "this sensor
 * has produced 4000 samples and none of them failed". Success is silent everywhere else in this
 * file -- a working cycle logs nothing, by design -- which leaves "the thread is alive" and "the
 * thread is reading four sensors" indistinguishable from outside. These are the difference.
 *
 * atomic_t rather than plain counters because the acquisition thread writes them while the shell
 * thread reads them. An unsynchronised read of a plain int written by another thread is a data race
 * and therefore undefined behaviour in C++, not merely a stale value; "diagnostics are allowed to be
 * approximate" is a statement about the NUMBER, not a licence for UB. Relaxed ordering is all that
 * is wanted: each counter is independent and nothing is published through them.
 *
 * They are NOT a gate, a control, or an input to any decision. Nothing in this file reads them. */
atomic_t tally_reads_[kMaxSources]{};
atomic_t tally_read_errors_[kMaxSources]{};
atomic_t tally_samples_[kMaxSources]{};
atomic_t tally_rearm_failures_[kMaxSources]{};

/* The most recent read's stage and port errno, packed into ONE word.
 *
 * Two separate atomics would NOT have been a snapshot: the reader can be preempted between them
 * and come back with this cycle's stage beside the last cycle's errno -- precisely the pairing the
 * whole idea was meant to rule out, reintroduced one level up. Reading source_facts::status
 * directly has the same defect and is worse, since that struct is several words wide.
 *
 * Layout: stage in bits 16-23, port errno as a 16-bit signed value in bits 0-15. An errno outside
 * that range is clamped rather than truncated, because a truncated errno is a DIFFERENT errno and
 * would be read as one. */
atomic_t tally_last_status_[kMaxSources]{};

/* Identity, so a reader can say WHICH sensor a row is about. Packed the same way: cliff flag in
 * bit 8, role id in bits 0-7.
 *
 * Written by init() and REWRITTEN BY EVERY bring_up(), because the role a source carries is not
 * known at init time: nothing has been proven then, so every cliff role is kRoleUnassigned, and the
 * proof's commit writes the real ones into the descriptor table afterwards. See the note in
 * bring_up() -- the same staleness in facts_ is what silently suppresses every measurement frame.
 *
 * It is an atomic here rather than read from cfg_/facts_ on demand because init() may run again,
 * and a reader racing a re-init would otherwise pick up half of one source table and half of
 * another. */
atomic_t tally_identity_[kMaxSources]{};

/* Lifetime, and deliberately NOT next_cycle_seq_: that one is the contract's per-epoch cycle number
 * and begin_epoch() resets it to 0, so a renumbering would read as the thread having stopped. */
atomic_t tally_cycles_{ATOMIC_INIT(0)};

/* True when the caller is allowed to drive the ULD: either no thread owns it, or this IS that
 * thread. Counting the refusals rather than only rejecting them, because a foreign call is a wiring
 * defect and a wiring defect that leaves no trace gets rediscovered instead of fixed. */
bool may_touch_devices()
{
    if (!thread_active_ || owner_ == k_current_get())
        return true;
    atomic_inc(&foreign_calls_);
    return false;
}
cycle_facts facts_;

/* The cycle number the NEXT cycle will carry.
 *
 * The contract is specific: cycle_seq "starts at 0 for the first cycle of a new
 * mapping_epoch, increments by one per COMPLETED cycle and wraps 255 -> 0". A
 * pre-increment at the top of the cycle gives 1 for the first one, which is a plain
 * violation -- and one that an earlier integration test managed to pin as expected
 * behaviour, comment and all.
 *
 * Kept as a full uint32 rather than the wire's uint8: the publisher truncates for the
 * wire, which produces the required 255 -> 0 wrap, while the untruncated value is what
 * lets a pending frame be matched to its own cycle without aliasing every 256th one.
 *
 * Reset to 0 in init() and NOWHERE ELSE.
 *
 * It is tempting to reset it in bring_up() as well, on the grounds that a bring-up is a
 * fresh enumeration attempt. That is wrong, and was briefly implemented and even pinned by
 * a test: the contract requires at most one measurement frame per
 * (source_id, mapping_epoch, cycle_seq), and this layer does not own the epoch -- it is
 * injected on the publishing side and nothing here advances it. Resetting the cycle while
 * the epoch stands still reissues triples that have already been used, which the contract
 * calls a conflict rather than a retransmission to be tolerated. So a second bring-up
 * continues the numbering.
 *
 * Advancing the epoch and restarting the cycle count are two halves of one operation, and
 * they belong to whatever owns the mapping. Implementing either half here would be
 * simulating a transition that has no API yet. The commit that lifts the PROVEN clamp is
 * where both halves land together. */
uint32_t next_cycle_seq_{0};
atomic_t snapshot_{ATOMIC_INIT(0)};
k_timer health_timer_;
bool health_timer_inited_{false};
bool health_work_inited_{false};
k_work health_work_;

// Bit layout of the snapshot. One word, so the heartbeat reads it without the lock.
constexpr int kStateBits{2};
constexpr int kProducedShift{kStateBits};
constexpr int kErrorShift{kProducedShift + kMaxSources};
constexpr int kConfiguredShift{kErrorShift + kMaxSources};
constexpr int kStartedShift{kConfiguredShift + kMaxSources};

/* ------------------------------------------------------------------ cliff ops ------ */

int cliff_open(void *dev, uint8_t addr_7bit, op_status *st)
{
    return tof_cliff_sensor_open(static_cast<VL53L4CX_Object_t *>(dev), addr_7bit, st);
}

int cliff_configure(void *dev, op_status *st)
{
    // The mode and budget are the caller's business, not this layer's, and both are
    // still unresolved in the wire contract. Until the injection point for them exists
    // the configure step is a no-op that reports success without touching the device -
    // deliberately visible as "not configured yet" rather than as a frozen value.
    ARG_UNUSED(dev);
    memset(st, 0, sizeof(*st));
    return 0;
}

int cliff_start(void *dev, op_status *st)
{
    return tof_cliff_sensor_start(static_cast<VL53L4CX_Object_t *>(dev), st);
}

int cliff_read_sample(void *dev, void *scratch, struct tof_cliff_sample *out, op_status *st)
{
    return tof_cliff_read_once(static_cast<VL53L4CX_Object_t *>(dev),
                               static_cast<struct tof_cliff_scratch *>(scratch), out, st);
}

int cliff_stop(void *dev, op_status *st)
{
    return tof_cliff_sensor_stop(static_cast<VL53L4CX_Object_t *>(dev), st);
}

const source_ops kCliffOps{
    cliff_open, cliff_configure, cliff_start, cliff_read_sample, cliff_stop,
};

/* ------------------------------------------------------------- typed L7 grid
 * stub -- */

int l7_open(void *, uint8_t, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    return -ENOSYS;
}
int l7_configure(void *, uint8_t, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    return -ENOSYS;
}
int l7_start(void *, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    return -ENOSYS;
}
int l7_read_grid_sample(void *, void *, tof_l7::sample *out, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    if (out != nullptr)
        *out = tof_l7::sample{};
    return -ENOSYS;
}
int l7_stop(void *, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    return -ENOSYS;
}

const grid_source_ops kL7GridStubOps{
    l7_open, l7_configure, l7_start, l7_read_grid_sample, l7_stop,
};

/* ------------------------------------------------------------------ internals ------ */

uint32_t now()
{
    return cfg_.now_ms();
}

void publish_snapshot()
{
    uint32_t word{static_cast<uint32_t>(effective_mapping_state())};

    for (int i{0}; i < facts_.source_count && i < kMaxSources; ++i) {
        const source_facts &f{facts_.sources[i]};

        if (f.sample_produced)
            word |= 1U << (kProducedShift + i);
        // A fault bit, not an "anything went wrong" bit. usage_error belongs here
        // because it is a real defect; unsupported does not, because a model with no
        // implementation is a static property of this build rather than something that
        // happened to a sensor this cycle.
        if (f.transport_error || f.protocol_error || f.usage_error)
            word |= 1U << (kErrorShift + i);
        if (f.configured)
            word |= 1U << (kConfiguredShift + i);
        if (f.started)
            word |= 1U << (kStartedShift + i);
    }
    atomic_set(&snapshot_, static_cast<atomic_val_t>(word));
}

void health_work_handler(k_work *)
{
    // Reads one atomic word. Takes no lock and touches no device, so a stalled bring-up
    // cannot silence it.
    if (cfg_.hooks.on_cliff_health != nullptr)
        cfg_.hooks.on_cliff_health(static_cast<uint32_t>(atomic_get(&snapshot_)),
                                   effective_mapping_state());
}

void health_timer_handler(k_timer *)
{
    k_work_submit(&health_work_);
}

// One place clears the outcomes, because the previous version enumerated the fields by
// hand at two call sites and the two outcomes added later were added to neither - so a
// -EINVAL in one cycle stayed set, and its fault bit with it, for the life of the board.
void clear_outcomes(source_facts &f)
{
    f.sample_produced = false;
    f.transport_error = false;
    f.protocol_error = false;
    f.unsupported = false;
    f.usage_error = false;
    f.rearm_failed = false;
    f.status = source_status{};
}

// Records an operation's outcome as a neutral fact. The only interpretation performed
// here is the mechanical one: which shape of failure occurred. The four are kept apart
// because folding them together would decide health semantics by accident - a stubbed
// model would arrive at the cliff health frame as a broken sensor, and a bug in our own
// call would arrive as a bus fault.
/* The quiescing itself, with the chain lock ALREADY HELD.
 *
 * Factored out because there are now two ways in -- stop(), which waits for the chain, and
 * try_stop(), which refuses to -- and they must differ in exactly one respect: how they acquire
 * the lock. A second copy of "clear running_, stop every started device, clear in_cycle_" would
 * be free to drift, and the direction it would drift in is a device left ranging while the
 * subsystem believes it is quiesced -- during an enumeration that re-addresses parts underneath
 * it.
 *
 * Callers publish the snapshot after releasing, not here: publishing under the chain lock would
 * put a lock between the health path and the acquisition path, and the heartbeat is required to
 * keep running while the chain is busy for seconds at a time. */
void stop_locked()
{
    running_ = false;

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};

        if (!f.started)
            continue;
        if (d.kind == model::l4_cliff) {
            op_status st{};
            (void)d.ops->stop(d.dev, &st);
        } else {
            tof_l7::operation_status st{};
            (void)d.grid_ops->stop(d.dev, &st);
        }
        f.started = false;
    }

    in_cycle_ = false;
}

void record_outcome(source_facts &f, int rc)
{
    if (rc == 0)
        return;
    if (rc == -EPROTO || rc == -EBADMSG)
        f.protocol_error = true;
    else if (rc == -ENOSYS)
        f.unsupported = true;
    else if (rc == -EINVAL)
        f.usage_error = true;
    else
        f.transport_error = true;
}

void record(source_facts &f, int rc, const op_status &st)
{
    f.status.domain = status_domain::l4;
    f.status.stage = static_cast<uint8_t>(st.stage);
    f.status.port_errno = st.port_errno;
    f.status.uld_status = st.uld_rc;
    f.status.sample_present = st.sample_present;
    f.status.rearm_failed = st.rearm_failed;
    f.rearm_failed = st.rearm_failed;
    record_outcome(f, rc);
}

void record(source_facts &f, int rc, const tof_l7::operation_status &st)
{
    f.status.domain = status_domain::l7;
    f.status.stage = static_cast<uint8_t>(st.failed_stage);
    f.status.port_errno = st.port_errno;
    f.status.uld_status = st.uld_status;
    f.status.sample_present = st.sample_present;
    f.status.rearm_failed = false;
    f.rearm_failed = false;
    record_outcome(f, rc);
}

}  // namespace

const grid_source_ops &l7_grid_stub_ops()
{
    return kL7GridStubOps;
}

const source_ops &l4_cliff_ops()
{
    return kCliffOps;
}

const char *operation_stage_name(const source_status &status)
{
    switch (status.domain) {
    case status_domain::none:
        return "none";
    case status_domain::l4:
        return tof_cliff_stage_name(static_cast<enum tof_cliff_stage>(status.stage));
    case status_domain::l7:
        return tof_l7::stage_name(static_cast<tof_l7::stage>(status.stage));
    }
    return "unknown";
}

mapping_state effective_mapping_state()
{
    return clamp_mapping_state(cfg_.mapping_state_provider != nullptr
                                   ? cfg_.mapping_state_provider()
                                   : mapping_state::not_ready);
}

mapping_state clamp_mapping_state(mapping_state reported)
{
    if (reported == mapping_state::proven) {
        // A proven mapping is not something this firmware is entitled to claim yet, so
        // reporting NOT_READY keeps the consumer's own fail-safe path in charge.
        //
        // WHAT IS ACTUALLY MISSING, as of 2026-08-21. This list has now been wrong TWICE.
        // First it said "until the two-board enable chain is fixed", and the hardware was
        // fixed on 08-17 (a 50 ohm series resistor on the data line). Then it said the chain
        // enumerates only one of four cliff positions so a real-machine proof "must still
        // fail" -- and on 08-21 dasher1 enumerated all six positions and proved them. Both
        // times a stale reason would have sent the reader to the wrong conclusion, so keep
        // this current or delete it; a clamp whose stated reason is false is worse than one
        // with no comment.
        //
        // What HAS been accepted on hardware (dasher1, 2026-08-21): a full proof at
        // epoch 2 with walk1 and walk2 both COMPLETE, refusal none -- so all 28 proof
        // checks passed on real data, including the four frozen roles, address distinctness,
        // fingerprint equality across the two walks, and every isolation rule
        // (tail 0x2f answered, neighbour 0x2e proven silent). The descriptors were keyed
        // from that mapping: positions 3-6 report role_id 0,1,2,3.
        //
        // BOTH of the items that were on this list are now closed, on 2026-09-17:
        //
        //   - The 400 kHz question. A proof no longer ends at the speed it was carried out
        //     at: commissioning sets 100 kHz for the walks, retimes to 400 kHz, and
        //     re-verifies every position's identity at its assigned address BEFORE the
        //     authority is told anything. A chain that does not answer at the product speed
        //     never reaches commit_proof(), so PROVEN can no longer mean "proven only at a
        //     speed the acquisition schedule cannot use".
        //   - The stack watermark. Measured on dasher2 under 3.6.0-109: 1144 / 2048 (55 %),
        //     stable across several thousand cycles, with all four sources' reads and samples
        //     advancing together and no read or re-arm failures. The devicetree number is no
        //     longer a number chosen without a measurement.
        //
        // SO WHY IS THE CLAMP STILL HERE. Because lifting it is its own commit, and the point
        // of that separation is that the first image with it lifted gets validated for
        // 0x216/0x217 immediately and for nothing else -- see the paragraph below, which is
        // the reason this cannot be folded into the change that closed the prerequisites.
        // Leaving it shut for one build is cheap; conflating "the prerequisites are closed"
        // with "the wire has been observed" is the mistake this whole list exists to prevent.
        //
        // Deliberately NOT on this list, because it cannot be: "no 0x216 capture" and "no
        // correlated cycle health". This clamp is what prevents both, so requiring them
        // before lifting it is circular. They are what must be verified IMMEDIATELY AFTER
        // it is lifted, on the same image, before that image is used for anything else.
        //
        // What WAS on this list and is now closed, because a list that only grows stops being
        // read: the mapping authority, the epoch plus cycle-reset transaction, the cycle
        // health frame, the single runtime bootstrap, role_id being keyed from the authority's
        // installed mapping inside the commit transaction rather than from the descriptor's
        // index, and the acquisition thread with its request-stop plus bounded join. None of
        // them lifts this clamp, and the log line below has to keep naming what is actually
        // left -- a stale diagnostic sends whoever reads it to the wrong place.
        //
        // Unconditional, with no build flag to lift it. A conditional safety bypass is
        // one careless -D away from shipping and would not show up in a diff of the code
        // it disables; lifting this is an edit here, in its own commit -- which is now the
        // only thing standing between this board and 0x216 on the wire.
        static bool warned{false};
        if (!warned) {
            warned = true;
            /* INFO, not WRN, and that is a deliberate downgrade. While the prerequisites were
             * open this was a warning about something wrong; now it reports a deliberate state
             * on the ordinary success path, and an error-level line there would contaminate
             * every acceptance transcript that follows a good proof. */
            LOG_INF("mapping reported PROVEN; clamped to NOT_READY -- the transaction and "
                    "acquisition prerequisites are closed, and the clamp stays until its "
                    "removal in a separate commit, followed immediately by 0x216/0x217 "
                    "end-to-end validation");
        }
        return mapping_state::not_ready;
    }
    return reported;
}

bool publication_allowed()
{
    return effective_mapping_state() == mapping_state::proven;
}

uint32_t snapshot()
{
    return static_cast<uint32_t>(atomic_get(&snapshot_));
}

bool is_idle()
{
    // Between two cycles the scheduler still owns the chain: it will take the lock again
    // within one period, and commissioning cannot enumerate in that gap without
    // re-addressing parts underneath the next read. Idle means stopped.
    //
    // Under the lock, because the two flags are written by the acquisition context and read
    // here by the commissioning one. A lock-free read can see running_ already false while
    // in_cycle_ has not yet been cleared -- or the reverse -- and answer "idle" about a state
    // that never existed. Blocking until the current cycle finishes is the correct behaviour
    // for a commissioning caller and is exactly what it is asking about.
    //
    // Never call this from the health path: it can wait for a whole cycle.
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    const bool idle{!running_ && !in_cycle_};
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    return idle;
}

void copy_facts(cycle_facts &out)
{
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    out = facts_;
    k_mutex_unlock(&tof_chain_controller::chain_lock());
}

int init(const config &cfg)
{
    /* A live subsystem is not re-configurable. cfg_ is read by the health work item from another
     * context, so replacing it here would be a data race on a struct full of function pointers.
     * Retiring the subsystem is teardown()'s job and has to be asked for explicitly. */
    if (active_)
        return -EALREADY;
    if (cfg.source_count <= 0 || cfg.source_count > kMaxSources)
        return -EINVAL;
    if (cfg.sources == nullptr || cfg.now_ms == nullptr ||
        cfg.mapping_state_provider == nullptr)
        return -EINVAL;
    /* on_cycle_begin is required, not optional. A sink that never hears the start of a cycle
     * cannot report a cycle that produced nothing, and a silently missing hook would turn every
     * zero-sample cycle into a gap the consumer reads as "the producer stopped". */
    if (cfg.hooks.on_cycle_begin == nullptr || cfg.hooks.on_cycle == nullptr ||
        cfg.hooks.on_cliff_health == nullptr)
        return -EINVAL;
    // No defaults on purpose: an invented period would become the specification.
    if (cfg.periods.cycle_period_ms == 0 || cfg.periods.health_period_ms == 0)
        return -EINVAL;
    bool has_cliff{false};
    bool has_grid{false};
    for (int i{0}; i < cfg.source_count; ++i) {
        const source_desc &d{cfg.sources[i]};

        if (d.kind == model::l4_cliff) {
            has_cliff = true;
            if (d.ops == nullptr || d.ops->open == nullptr || d.ops->configure == nullptr ||
                d.ops->start == nullptr || d.ops->read_cliff_sample == nullptr ||
                d.ops->stop == nullptr || d.grid_ops != nullptr || d.dev == nullptr ||
                d.scratch == nullptr)
                return -EINVAL;
        } else {
            has_grid = true;
            if (d.grid_ops == nullptr || d.grid_ops->open == nullptr ||
                d.grid_ops->configure == nullptr || d.grid_ops->start == nullptr ||
                d.grid_ops->read_grid_sample == nullptr || d.grid_ops->stop == nullptr ||
                d.ops != nullptr)
                return -EINVAL;
            /* The explicit stub is allowed to have no object yet. Any real grid table
             * must supply both; its implementation owns what those types are. */
            if (d.grid_ops != &l7_grid_stub_ops() &&
                (d.dev == nullptr || d.scratch == nullptr || d.grid_frequency_hz == 0))
                return -EINVAL;
        }
    }
    if ((has_cliff && cfg.hooks.on_cliff_sample == nullptr) ||
        (has_grid && cfg.hooks.on_grid_sample == nullptr))
        return -EINVAL;

    cfg_ = cfg;
    configured_ = true;
    running_ = false;
    in_cycle_ = false;

    facts_ = cycle_facts{};
    /* A fresh start is a fresh epoch, so the first cycle must carry 0. */
    next_cycle_seq_ = 0;
    /* Cleared with the rest of the per-configuration state: counts carried across an init() would
     * describe a source table that no longer exists. */
    for (int i{0}; i < kMaxSources; ++i) {
        atomic_clear(&tally_reads_[i]);
        atomic_clear(&tally_read_errors_[i]);
        atomic_clear(&tally_samples_[i]);
        atomic_clear(&tally_rearm_failures_[i]);
        atomic_clear(&tally_last_status_[i]);
        atomic_clear(&tally_identity_[i]);
    }
    for (int i{0}; i < cfg.source_count; ++i) {
        const source_desc &d{cfg.sources[i]};
        atomic_set(&tally_identity_[i],
                   static_cast<atomic_val_t>((d.kind == model::l4_cliff ? 0x100U : 0U) |
                                             (d.role_id & 0xFFU)));
    }
    atomic_clear(&tally_cycles_);
    facts_.source_count = cfg.source_count;
    for (int i{0}; i < cfg.source_count; ++i) {
        facts_.sources[i].kind = cfg.sources[i].kind;
        facts_.sources[i].addr_7bit = cfg.sources[i].addr_7bit;
        facts_.sources[i].role_id = cfg.sources[i].role_id;
        facts_.sources[i].configured = true;
    }
    publish_snapshot();

    // Armed before bring-up, so the heartbeat is already running while the sensors are
    // still being opened. That is the whole point: NOT_READY has to be audible during
    // exactly the window where something might hang.
    //
    // Initialised once, for the same reason as the timer below: while the heartbeat is alive
    // this work item can be queued or running, and re-initialising it then is no safer than
    // re-initialising a live timer. The timer bug was the one that spun; this one was simply
    // waiting for a re-init to land in the wrong microsecond.
    if (!health_work_inited_) {
        k_work_init(&health_work_, health_work_handler);
        health_work_inited_ = true;
    }
    /* Initialised ONCE, and never again while it may be running.
     *
     * This used to be an unconditional k_timer_init() on every init(), which worked only
     * because stop() also stopped the timer -- so the next init() always found it dead.
     * The moment stop() correctly stopped killing the heartbeat, a second init() was
     * re-initialising a timer that was still in the kernel's timeout list, which k_timer_init
     * does not permit (it is documented for use "prior to its first use"). The observable was
     * the whole host suite spinning at 100% CPU inside the first test that slept, with no
     * output and no crash -- and it would do the same on the board, where nothing would print
     * at all. k_timer_start below restarts a running timer on its own, which is all a re-init
     * of this subsystem actually needs. */
    if (!health_timer_inited_) {
        k_timer_init(&health_timer_, health_timer_handler, nullptr);
        health_timer_inited_ = true;
    }
    k_timer_start(&health_timer_, K_MSEC(cfg.periods.health_period_ms),
                  K_MSEC(cfg.periods.health_period_ms));
    active_ = true;
    return 0;
}

int begin_epoch()
{
    if (!configured_)
        return -EINVAL;

    /* Idle only, and the check and the reset happen together under the chain lock.
     *
     * Both halves matter. `running_` alone is not idle: between two cycles the scheduler
     * still owns the chain, so in_cycle_ has to be clear as well. And checking outside the
     * lock would let a cycle start between the check and the reset, which renumbers a
     * sequence the consumer is half-way through assembling -- the contract's uniqueness
     * guarantee is over (source_id, mapping_epoch, cycle_seq), the triple rather than the
     * cycle alone, so a renumber under a live epoch reissues triples already used. */
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    const bool idle{!running_ && !in_cycle_};
    if (idle)
        next_cycle_seq_ = 0;
    k_mutex_unlock(&tof_chain_controller::chain_lock());

    return idle ? 0 : -EBUSY;
}

int bring_up()
{
    if (!configured_)
        return -EINVAL;
    if (!may_touch_devices())
        return -EPERM;

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    in_cycle_ = true;

    /* Deliberately NOT resetting next_cycle_seq_ here. Restarting the cycle count without
     * advancing the epoch reissues (source_id, epoch, cycle_seq) triples that have already
     * been used, and the contract calls that a conflict. See the note on next_cycle_seq_.
     */

    /* RE-READ THE IDENTITY FROM THE DESCRIPTOR TABLE, here, every bring-up.
     *
     * init() runs at boot, long before any mapping has been proven, so the role ids it copied are
     * all kRoleUnassigned. The proof's commit transaction later writes the real ones into the
     * descriptor table -- the same array cfg_.sources points at -- and nothing propagated them into
     * facts_. The copies stayed at 255 for the life of the process.
     *
     * That is not a cosmetic staleness. The publisher refuses to pack a sample whose descriptor
     * role and facts role disagree, counts it as suppressed_role_mismatch and marks the whole cycle
     * invalid; and facts_.sources[i].role_id is the value it would have encoded. So once the PROVEN
     * clamp is lifted, a stale copy here suppresses EVERY measurement frame while the board
     * otherwise looks healthy -- health flowing, sensors reading, nothing in any log.
     *
     * bring-up is the right place: it runs under the chain lock, from the owning thread, after the
     * commit that keyed the descriptors, and again after every re-proof, since proving stops
     * acquisition and starting it brings the sources up afresh. */
    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};

        facts_.sources[i].kind = d.kind;
        facts_.sources[i].addr_7bit = d.addr_7bit;
        facts_.sources[i].role_id = d.role_id;
        atomic_set(&tally_identity_[i],
                   static_cast<atomic_val_t>((d.kind == model::l4_cliff ? 0x100U : 0U) |
                                             (d.role_id & 0xFFU)));
    }

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};

        f.started = false;
        clear_outcomes(f);

        int rc{0};
        if (d.kind == model::l4_cliff) {
            op_status st{};

            rc = d.ops->open(d.dev, d.addr_7bit, &st);
            if (rc == 0)
                rc = d.ops->configure(d.dev, &st);
            if (rc == 0)
                rc = d.ops->start(d.dev, &st);
            record(f, rc, st);
        } else {
            tof_l7::operation_status st{};

            rc = d.grid_ops->open(d.dev, d.addr_7bit, &st);
            if (rc == 0)
                rc = d.grid_ops->configure(d.dev, d.grid_frequency_hz, &st);
            if (rc == 0)
                rc = d.grid_ops->start(d.dev, &st);
            record(f, rc, st);
        }

        if (rc != 0) {
            LOG_WRN("source %d open/configure/start failed at %s rc %d errno %d", i,
                    operation_stage_name(f.status), rc, f.status.port_errno);
            /* One source that will not open must not stop the others from ranging. */
            continue;
        }
        f.started = true;
    }

    running_ = true;
    in_cycle_ = false;
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    publish_snapshot();
    return 0;
}

void run_cycle()
{
    /* running_ is read ONLY under the chain lock, here and everywhere else.
     *
     * There used to be an unlocked pre-check above this, excused as an advisory read that was
     * allowed to be stale. That excuse does not exist in C++: running_ is a plain bool written by
     * try_stop() under the lock from another thread, so an unsynchronised read of it is a data race
     * and therefore undefined behaviour -- not a value that is merely out of date. A second check
     * under the lock does not repair the first one, and "it is only an optimisation" is not a
     * defence for UB. The alternative would be to make it atomic, which buys one saved lock
     * acquisition per idle period in exchange for two synchronisation rules for one variable.
     *
     * The check itself matters as soon as there is an acquisition thread. The thread can find
     * running_ true, wait here for whoever holds the chain, and be woken by the very try_stop()
     * that cleared it -- then run a full cycle AFTER the quiesce reported success to a commissioner
     * who has been told the chain is safe to enumerate in. Enumeration drops enable lines, so the
     * cycle would be reading parts that are being re-addressed underneath it.
     *
     * Returning here leaves the cycle NOT BEGUN: no on_cycle_begin, no on_cycle, and
     * next_cycle_seq_ untouched. That is the correct account of what happened -- the cycle did not
     * happen, so it owes no health frame and must not consume a cycle number. */
    if (!may_touch_devices())
        return;

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);

    if (!running_) {
        k_mutex_unlock(&tof_chain_controller::chain_lock());
        return;
    }

    in_cycle_ = true;

    facts_.cycle_seq = next_cycle_seq_;
    facts_.began_ms = now();

    /* Before the first sensor is touched. A cycle that produces no sample at all still has to be
     * announced, and this is the only point at which that is possible: every later hook is
     * per-sample. Under the chain lock, like the sample hook, so a sink sees one consistent
     * ordering. */
    cfg_.hooks.on_cycle_begin(facts_.cycle_seq);

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};

        // A source that never started has nothing happening this cycle, and its
        // bring-up diagnosis is the only record of why. Clearing before this check -
        // which is what the first version did - erased the reason on the first cycle and
        // left nothing but started == false to go on.
        if (!f.started)
            continue;

        clear_outcomes(f);

        if (d.kind == model::l4_cliff) {
            struct tof_cliff_sample sample {};
            op_status st{};
            const int rc{d.ops->read_cliff_sample(d.dev, d.scratch, &sample, &st)};

            record(f, rc, st);
            atomic_inc(&tally_reads_[i]);
            if (rc != 0)
                atomic_inc(&tally_read_errors_[i]);
            /* Counted from op_status rather than from the sticky flag in source_facts, so this is
             * the number of TIMES a re-arm failed and not "is it still broken". The sticky one
             * answers a different question and already exists. */
            if (st.rearm_failed)
                atomic_inc(&tally_rearm_failures_[i]);
            {
                int clamped{st.port_errno};
                if (clamped > INT16_MAX)
                    clamped = INT16_MAX;
                else if (clamped < INT16_MIN)
                    clamped = INT16_MIN;
                atomic_set(&tally_last_status_[i],
                           static_cast<atomic_val_t>((static_cast<uint32_t>(st.stage) & 0xFFU) << 16 |
                                                     (static_cast<uint32_t>(clamped) & 0xFFFFU)));
            }

            f.sample_produced = st.sample_present && sample.fresh;
            if (f.sample_produced)
                atomic_inc(&tally_samples_[i]);
            if (f.sample_produced)
                cfg_.hooks.on_cliff_sample(i, facts_.cycle_seq, f, sample);
        } else {
            tof_l7::sample sample{};
            tof_l7::operation_status st{};
            const int rc{d.grid_ops->read_grid_sample(d.dev, d.scratch, &sample, &st)};

            record(f, rc, st);
            f.sample_produced = st.sample_present && sample.fresh;
            if (f.sample_produced)
                cfg_.hooks.on_grid_sample(i, facts_.cycle_seq, f, sample);
        }
    }

    in_cycle_ = false;
    k_mutex_unlock(&tof_chain_controller::chain_lock());

    publish_snapshot();
    cfg_.hooks.on_cycle(facts_);

    /* Per COMPLETED cycle, which is why this is here and not at the top. */
    ++next_cycle_seq_;
    atomic_inc(&tally_cycles_);
}

void teardown()
{
    /* The real shutdown: quiesce acquisition, stop the heartbeat, then WAIT for any health work
     * already submitted to finish. Separate from stop() because they answer different questions
     * -- "pause reading" versus "this subsystem is going away".
     *
     * The cancel is the part that is easy to leave out and wrong to: k_timer_stop() only stops
     * future ticks, and the last tick may already have submitted a work item that is queued or
     * mid-flight. Without the synchronous cancel a health frame can be emitted AFTER teardown
     * returned, which makes "the subsystem is down" false at exactly the moment something is
     * relying on it -- and leaves the work item reading a cfg_ the next init() is entitled to
     * replace. */
    if (!configured_)
        return;
    /* The thread first, and through the same request-and-join path: retiring the subsystem while a
     * thread is still driving sensors would tear cfg_ out from under it. An unbounded wait is
     * correct HERE and only here -- teardown means the subsystem is going away, so there is nothing
     * left to stay responsive for, and a thread that never exits is a hang worth having rather than
     * a half-retired subsystem still touching a bus. */
    if (thread_created_) {
        request_stop();
        (void)k_thread_join(&thread_, K_FOREVER);
        thread_created_ = false;
    }
    stop();
    k_timer_stop(&health_timer_);
    static k_work_sync sync;
    (void)k_work_cancel_sync(&health_work_, &sync);

    /* RETIRED, not merely paused. Clearing configured_ is the point of the whole call: it used
     * to be left set, so after a teardown bring_up() and begin_epoch() still accepted the OLD
     * configuration and would restart acquisition with the heartbeat already stopped -- a
     * producer emitting measurements with no liveness channel, which is the one combination the
     * consumer cannot reason about. Commissioning walks this path on every attempt, so it would
     * not have stayed theoretical for long. A fresh init() is now the only way back. */
    active_ = false;
    configured_ = false;
}

void stop()
{
    if (!configured_)
        return;
    if (!may_touch_devices())
        return;

    // Quiesce, then release. Commissioning may only enumerate once this returns: the
    // enumeration drops enable lines, which re-addresses parts underneath a reader.
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    stop_locked();
    /* The health timer is deliberately NOT stopped here.
     *
     * stop() means "stop reading sensors", and the contract requires health to keep flowing
     * while no acquisition runs -- that is precisely when a consumer most needs to know the
     * subsystem is alive and why it is not producing. Stopping the heartbeat because
     * commissioning quiesced the chain would turn a controlled pause into a silence
     * indistinguishable from a crashed producer, and the consumer's own timeout would raise a
     * fault for a machine that is behaving exactly as asked.
     *
     * teardown() is what stops the heartbeat, and it exists so that shutting the subsystem down
     * is a separate, deliberate act. */
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    publish_snapshot();
}

static void thread_entry(void *, void *, void *)
{
    /* The whole ULD lifecycle, on one thread, in one place.
     *
     * bring_up() failures are per source and deliberately not fatal here: one dead cliff sensor must
     * not stop the other three from ranging, and the facts say which one it was. A total failure
     * still produces cycles -- empty ones -- and the health frame is what reports that, which is
     * better than a thread that exits and leaves a consumer to infer why. */
    /* A non-zero RETURN used to be discarded here. It means bring_up() refused before reaching any
     * source at all -- wrong configuration, or an ownership record this thread did not recognise as
     * its own -- so no source was even attempted and running_ was never set. Every cycle after it is
     * then a silent no-op. It is logged and deliberately NOT folded into sensor_fault_mask: no
     * sensor failed, and the heartbeat already withholds cycle_valid while no cycle completes, so a
     * consumer stays fail-closed on it. A named producer-internal wire reason is its own design,
     * not something to smuggle in by reusing a sensor's bit. */
    if (int const rc{bring_up()}; rc != 0)
        LOG_ERR("acquisition bring-up refused before source bring-up: rc %d", rc);

    while (atomic_get(&stop_requested_) == 0) {
        run_cycle();
        /* The cadence, and the stop signal, in one wait. Sleeping for the period and checking the
         * flag afterwards would make every stop request cost up to a full period before it was even
         * noticed -- and that period is what a caller's join timeout would then have to cover. */
        (void)k_sem_take(&stop_sem_, K_MSEC(cfg_.periods.cycle_period_ms));
    }

    /* At a cycle boundary, from the thread that owns the devices. This is the reason try_stop() no
     * longer stops devices itself: a foreign thread doing it while this one is mid-cycle interleaves
     * two callers inside the ULD, whose port keeps ONE transport record. */
    stop();
    thread_active_ = false;
    owner_ = nullptr;
}

int start(const thread_config &tcfg)
{
    if (!configured_)
        return -EINVAL;
    /* thread_created_ as well as thread_active_: a thread that has exited but has not been joined
     * still owns the kernel object, and creating over it is undefined. A caller that requested a
     * stop and never joined gets -EALREADY, which is the honest answer -- it has not finished
     * stopping. */
    if (thread_active_ || thread_created_)
        return -EALREADY;
    /* No defaults, the same rule the periods follow. A join timeout invented here would decide how
     * long commissioning waits before refusing, which is a deployment decision. */
    if (tcfg.stack == nullptr || tcfg.stack_size == 0 || tcfg.join_timeout_ms == 0)
        return -EINVAL;

    tcfg_ = tcfg;
    atomic_set(&stop_requested_, 0);
    k_sem_reset(&stop_sem_);

    /* NOT touching next_cycle_seq_. Starting a thread is not the start of an epoch: the contract
     * numbers cycles from 0 per mapping_epoch, begin_epoch() does that as one step of the
     * authority's commit, and a reset here would either renumber a sequence a consumer is part-way
     * through or reissue a (source_id, epoch, cycle_seq) triple that has already been used. */
    thread_active_ = true;
    thread_created_ = true;

    /* K_FOREVER, then an explicit start, because the ownership record has to be COMPLETE before
     * the new thread can run a single instruction.
     *
     * With K_NO_WAIT the thread becomes runnable inside k_thread_create(), so a priority higher
     * than the caller's preempts right there -- before the return value has been assigned to
     * owner_. The new thread then calls may_touch_devices(), finds thread_active_ already true and
     * owner_ still null, and concludes that IT is the foreign thread. bring_up() returns -EPERM at
     * the guard, running_ is never set, and every later run_cycle() takes the !running_ path
     * forever: a live thread that owns the chain, reports itself running, and never touches a
     * sensor. Nothing logged it, because the only evidence was a discarded return value.
     *
     * That is not a rare interleaving. The product configuration makes it certain: acquisition runs
     * at priority 7 and the shell thread that issues the start runs at the Zephyr default 14, so the
     * preemption is guaranteed rather than possible -- which is why it reproduced on the first
     * machine it was tried on and why the existing tests, whose thread sits BELOW the ztest thread,
     * never opened the window at all.
     *
     * Suspended creation makes the order a property of the code rather than of the scheduler. It is
     * also why owner_ is not simply assigned &thread_ beforehand: that would work, but only because
     * k_thread_create happens to return that pointer, which is a convention this file would then
     * depend on silently. */
    owner_ = k_thread_create(&thread_, tcfg_.stack, tcfg_.stack_size, thread_entry, nullptr, nullptr,
                             nullptr, tcfg_.priority, 0, K_FOREVER);
    k_thread_start(owner_);
    return 0;
}

void request_stop()
{
    /* Safe from any thread and idempotent: the flag is atomic and the semaphore has a limit of one.
     * It does not wait, because the caller may be a shell thread that must stay responsive; join()
     * is where waiting is bounded and reported. */
    atomic_set(&stop_requested_, 1);
    k_sem_give(&stop_sem_);
}

int join(uint32_t timeout_ms)
{
    if (!thread_created_)
        return 0;
    /* Bounded, and a timeout is the end of it. There is no way to abort a thread
     * that may be inside a vendor driver holding the chain lock with a
     * half-finished transfer -- k_thread_abort() would leave the lock held and
     * the bus mid-transaction, which is strictly worse than refusing to
     * commission. So the answer to a timeout is -EBUSY and the caller's refusal.
     */
    if (k_thread_join(&thread_, K_MSEC(timeout_ms)) != 0)
        return -EBUSY;
    thread_created_ = false;
    return 0;
}

bool thread_running()
{
    return thread_active_;
}

uint32_t foreign_lifecycle_calls()
{
    return static_cast<uint32_t>(atomic_get(&foreign_calls_));
}

/* One value per call, by design. A caller on the shell stack has about a hundred bytes to spare, so
 * handing it a struct or an array to hold would be the diagnostic corrupting the thing it is meant
 * to observe. Out-of-range indices return 0 rather than reading past the array. */
uint32_t source_reads(int index)
{
    return index >= 0 && index < kMaxSources ? static_cast<uint32_t>(atomic_get(&tally_reads_[index]))
                                             : 0U;
}

uint32_t source_read_errors(int index)
{
    return index >= 0 && index < kMaxSources
               ? static_cast<uint32_t>(atomic_get(&tally_read_errors_[index]))
               : 0U;
}

uint32_t source_samples(int index)
{
    return index >= 0 && index < kMaxSources
               ? static_cast<uint32_t>(atomic_get(&tally_samples_[index]))
               : 0U;
}

uint32_t source_rearm_failures(int index)
{
    return index >= 0 && index < kMaxSources
               ? static_cast<uint32_t>(atomic_get(&tally_rearm_failures_[index]))
               : 0U;
}

uint32_t source_last_status(int index)
{
    return index >= 0 && index < kMaxSources
               ? static_cast<uint32_t>(atomic_get(&tally_last_status_[index]))
               : 0U;
}

uint32_t source_identity(int index)
{
    return index >= 0 && index < kMaxSources
               ? static_cast<uint32_t>(atomic_get(&tally_identity_[index]))
               : 0U;
}

uint32_t cycles_completed()
{
    return static_cast<uint32_t>(atomic_get(&tally_cycles_));
}

#ifdef CONFIG_ZTEST
k_tid_t thread_id_for_test()
{
    return owner_;
}
#endif

int try_stop()
{
    /* Nothing configured is not a failure to quiesce: there is no acquisition to stop and this
     * layer is holding nothing. Answering -EBUSY here would make commissioning refuse on a board
     * where acquisition was never brought up at all -- which is every build that enables the ULD
     * without the bring-up probe, i.e. the common case today. */
    if (!configured_)
        return 0;

    /* With a thread running, this function touches no device: it asks, then waits with the injected
     * bound, and the THREAD is what stops the sensors. Stopping them from here would put a second
     * caller inside the ULD while the thread is mid-cycle.
     *
     * A timeout leaves everything exactly as it was -- thread running, devices ranging -- and says
     * -EBUSY. Nothing is killed: see join(). */
    if (thread_active_) {
        request_stop();
        return join(tcfg_.join_timeout_ms);
    }

    if (k_mutex_lock(&tof_chain_controller::chain_lock(), K_NO_WAIT) != 0)
        return -EBUSY; // somebody else owns the chain; nothing touched, so refusing
                       // is safe

    stop_locked();
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    publish_snapshot();

    /* Idle by construction rather than by a second query: the state was set under the lock we
     * just held, and re-reading it through is_idle() would take the chain again with K_FOREVER --
     * reintroducing the block this function exists to remove. */
    return 0;
}

}  // namespace lexxhard::tof_acq

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
