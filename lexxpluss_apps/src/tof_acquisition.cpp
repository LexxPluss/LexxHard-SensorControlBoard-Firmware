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
 * thread_active_ and owner_ are written only by start() and by the thread's own exit path, both
 * before/after the thread can be contended, and read by the ownership guard. owner_ is what makes
 * the guard possible at all: "is this call coming from the thread that owns the ULD" cannot be
 * answered by a flag. */
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

int cliff_start(void *dev, void *stream, op_status *st)
{
    return tof_cliff_sensor_start(static_cast<VL53L4CX_Object_t *>(dev),
                                  static_cast<struct tof_cliff_stream_state *>(stream), st);
}

int cliff_read_sample(void *dev, void *scratch, void *stream, struct tof_cliff_sample *out,
                      op_status *st)
{
    return tof_cliff_read_once(static_cast<VL53L4CX_Object_t *>(dev),
                               static_cast<struct tof_cliff_scratch *>(scratch),
                               static_cast<struct tof_cliff_stream_state *>(stream), out, st);
}

int cliff_stop(void *dev, op_status *st)
{
    return tof_cliff_sensor_stop(static_cast<VL53L4CX_Object_t *>(dev), st);
}

const source_ops kCliffOps{
    cliff_open, cliff_configure, cliff_start, cliff_read_sample, cliff_stop,
};

/* -------------------------------------------------------------------- L7 stub ------ */

int l7_open(void *, uint8_t, op_status *st)
{
    memset(st, 0, sizeof(*st));
    return -ENOSYS;
}
int l7_configure(void *, op_status *st)
{
    memset(st, 0, sizeof(*st));
    return -ENOSYS;
}
int l7_start(void *, void *, op_status *st)
{
    memset(st, 0, sizeof(*st));
    return -ENOSYS;
}
int l7_read_cliff_sample(void *, void *, void *, struct tof_cliff_sample *out, op_status *st)
{
    memset(st, 0, sizeof(*st));
    if (out != nullptr) {
        memset(out, 0, sizeof(*out));
    }
    return -ENOSYS;
}
int l7_stop(void *, op_status *st)
{
    memset(st, 0, sizeof(*st));
    return -ENOSYS;
}

const source_ops kL7StubOps{
    l7_open, l7_configure, l7_start, l7_read_cliff_sample, l7_stop,
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
        // rearm_failed is here too: a sensor that will not range again is a faulted
        // sensor, and this word is the only signal guaranteed to be sent. Without it the
        // sticky would exist in facts_ and never reach anyone.
        if (f.transport_error || f.protocol_error || f.usage_error || f.rearm_failed)
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

// One place clears the PER-CYCLE outcomes, because the previous version enumerated the
// fields by hand at two call sites and the two outcomes added later were added to neither
// - so a -EINVAL in one cycle stayed set, and its fault bit with it, for the life of the
// board.
//
// rearm_failed is deliberately NOT among them, and the name says so. It describes the
// device rather than the cycle: "this sensor will not produce another sample". Clearing it
// here made a wedged sensor indistinguishable from a quiet one on the next cycle -- the
// device is stopped, so read_once returns rc 0 with nothing ready, record() sets nothing,
// and the published state becomes configured=1 started=1 fault=0 produced=0, which is
// bit-for-bit what a healthy sensor with nothing ready this cycle looks like. Only a
// complete, successful re-bring-up clears it; see clear_sticky_on_bring_up().
void clear_cycle_outcomes(source_facts &f)
{
    f.sample_produced = false;
    f.transport_error = false;
    f.protocol_error = false;
    f.unsupported = false;
    f.usage_error = false;
    f.status = op_status{};
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
        op_status st{};

        if (!f.started)
            continue;
        (void)d.ops->stop(d.dev, &st);
        f.started = false;
    }

    in_cycle_ = false;
}

void record(source_facts &f, int rc, const op_status &st)
{
    f.status = st;
    if (rc == 0)
        return;
    if (rc == -EPROTO)
        f.protocol_error = true;
    else if (rc == -ENOSYS)
        f.unsupported = true;
    else if (rc == -EINVAL)
        f.usage_error = true;
    else
        f.transport_error = true;
    // One-way. A plain assignment here would let the NEXT ordinary error clear a sticky
    // set by an earlier re-arm failure, which is the same squashing this flag exists to
    // prevent, arriving one call later.
    if (st.rearm_failed)
        f.rearm_failed = true;
}

}  // namespace

const source_ops &l7_stub_ops()
{
    return kL7StubOps;
}

const source_ops &l4_cliff_ops()
{
    return kCliffOps;
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
        // What HAS been accepted on hardware (dasher1, 2026-08-21): a full proof at epoch 2
        // with walk1 and walk2 both COMPLETE, refusal none -- so all 28 proof checks passed
        // on real data, including the four frozen roles, address distinctness, fingerprint
        // equality across the two walks, and every isolation rule (tail 0x2f answered,
        // neighbour 0x2e proven silent). The descriptors were keyed from that mapping:
        // positions 3-6 report role_id 0,1,2,3.
        //
        // What is still missing:
        //
        //   - That proof required i2c2 at 100 kHz (diagnostic overlay). At the product's
        //     400 kHz, walk1 has never once reached COMPLETE on this machine, and a proof
        //     needs two COMPLETE walks. So there is no acceptance of the PROVEN path at the
        //     speed the product runs, and lifting the clamp would open a path proven only at
        //     a speed the final acquisition schedule cannot use. The cause is OPEN across
        //     hardware and firmware -- signal integrity, the STM32 timing configuration and
        //     the long-transfer path are all candidates, and one firmware contributor is
        //     confirmed: the STM32 timeout path ends the transfer and waits K_FOREVER for
        //     cleanup with no SWRST and no bit-bang release, so a first timeout leaves the
        //     bus unrecovered. Do not record this as "hardware's problem".
        //   - No boot timing and no stack watermark for the acquisition thread, whose stack
        //     size is therefore a devicetree number chosen without a measurement
        //     (acq-stack-size = 2048, acq-thread-priority = 7; the overlay says as much).
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
        // it disables; lifting this is an edit here, in its own commit, once both of the
        // above are closed.
        static bool warned{false};
        if (!warned) {
            warned = true;
            LOG_WRN("mapping reported PROVEN; clamped to NOT_READY -- proven on hardware "
                    "only at 100 kHz, never at the product's 400 kHz, and the acquisition "
                    "thread has no stack watermark");
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
        cfg.hooks.on_cliff_sample == nullptr || cfg.hooks.on_cliff_health == nullptr)
        return -EINVAL;
    // No defaults on purpose: an invented period would become the specification.
    if (cfg.periods.cycle_period_ms == 0 || cfg.periods.health_period_ms == 0)
        return -EINVAL;
    for (int i{0}; i < cfg.source_count; ++i) {
        const source_desc &d{cfg.sources[i]};

        if (d.ops == nullptr || d.ops->open == nullptr || d.ops->read_cliff_sample == nullptr)
            return -EINVAL;
        // The cliff path needs both a device object and a scratch; the stubbed grid
        // path is allowed to have neither yet.
        // The cliff path needs a device, the shared scratch AND its own stream state;
        // a null stream would mean the replay guard silently never arms.
        if (d.kind == model::l4_cliff &&
            (d.dev == nullptr || d.scratch == nullptr || d.stream == nullptr))
            return -EINVAL;
    }

    cfg_ = cfg;
    configured_ = true;
    running_ = false;
    in_cycle_ = false;

    facts_ = cycle_facts{};
    /* A fresh start is a fresh epoch, so the first cycle must carry 0. */
    next_cycle_seq_ = 0;
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

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};
        op_status st{};
        int rc;

        f.started = false;
        clear_cycle_outcomes(f);

        rc = d.ops->open(d.dev, d.addr_7bit, &st);
        if (rc != 0) {
            record(f, rc, st);
            LOG_WRN("source %d open failed at %s rc %d errno %d", i,
                    tof_cliff_stage_name(st.stage), rc, st.port_errno);
            // One sensor that will not open must not stop the others from ranging.
            continue;
        }

        rc = d.ops->configure(d.dev, &st);
        if (rc != 0) {
            record(f, rc, st);
            continue;
        }

        rc = d.ops->start(d.dev, d.stream, &st);
        if (rc != 0) {
            record(f, rc, st);
            continue;
        }
        f.started = true;
        /* The ONLY place the re-arm sticky is cleared, and it is here rather than at the top
         * of this loop on purpose. A recovery that fails at open, at configure or at start
         * must leave the sticky standing: clearing it before the outcome is known would
         * report a still-wedged sensor as healthy, and every `continue` above is a recovery
         * that did not happen. Reaching this line means the full open -> configure -> start
         * sequence succeeded, which is the only evidence that the device will range again. */
        f.rearm_failed = false;
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
        struct tof_cliff_sample sample{};
        op_status st{};
        int rc;

        // A source that never started has nothing happening this cycle, and its
        // bring-up diagnosis is the only record of why. Clearing before this check -
        // which is what the first version did - erased the reason on the first cycle and
        // left nothing but started == false to go on.
        if (!f.started)
            continue;

        clear_cycle_outcomes(f);

        rc = d.ops->read_cliff_sample(d.dev, d.scratch, d.stream, &sample, &st);
        record(f, rc, st);
        f.sample_produced = sample.fresh;

        // The payload goes to the model's own sink, so neither packer ever has to step
        // over the other model's data, and the fail direction stays out of here.
        if (f.sample_produced && d.kind == model::l4_cliff)
            cfg_.hooks.on_cliff_sample(i, facts_.cycle_seq, f, sample);
    }

    in_cycle_ = false;
    k_mutex_unlock(&tof_chain_controller::chain_lock());

    publish_snapshot();
    cfg_.hooks.on_cycle(facts_);

    /* Per COMPLETED cycle, which is why this is here and not at the top. */
    ++next_cycle_seq_;
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
    (void)bring_up();

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
    owner_ = k_thread_create(&thread_, tcfg_.stack, tcfg_.stack_size, thread_entry, nullptr, nullptr,
                             nullptr, tcfg_.priority, 0, K_NO_WAIT);
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
    /* Bounded, and a timeout is the end of it. There is no way to abort a thread that may be inside
     * a vendor driver holding the chain lock with a half-finished transfer -- k_thread_abort() would
     * leave the lock held and the bus mid-transaction, which is strictly worse than refusing to
     * commission. So the answer to a timeout is -EBUSY and the caller's refusal. */
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
        return -EBUSY;   // somebody else owns the chain; nothing touched, so refusing is safe

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
