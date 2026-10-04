/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_acquisition.hpp"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#if defined(ENABLE_TOF_L7_ULD)
/* Only for the stage vocabulary. An image without the grid driver has no l7 domain to name. */
#include "tof_l7_status.hpp"
#endif

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
 * ATOMIC, and the decision they feed is made under the chain lock. Two separate defects needed
 * both halves.
 *
 * They are written by start() and by the thread's own exit path and read by any caller, so as
 * plain objects they were a data race in the C++ sense -- undefined, not merely stale, whatever
 * the observed values happened to be. Atomics settle that.
 *
 * Atomics alone would not settle the other half. The guard used to run BEFORE the caller took the
 * chain lock, so a caller could pass while no thread owned the chain, block on the mutex, and drive
 * the ULD after start() had installed an owner. Every guarded path now takes the lock first and
 * asks second, which is why the function is named for its precondition.
 *
 * owner_ is what makes the guard possible at all: "is this call coming from the thread that owns
 * the ULD" cannot be answered by a flag. */
k_thread thread_;
atomic_ptr_t owner_{ATOMIC_PTR_INIT(nullptr)};
/* Two flags, because they answer different questions. thread_active_ is "is a thread driving the
 * ULD right now", which is what the ownership guard and try_stop() need. thread_created_ is "does
 * the kernel thread object still belong to a thread nobody has joined", which is what makes
 * restarting safe: k_thread_create() on an object whose previous thread has not been joined reuses a
 * live kernel structure. The thread itself clears the first; only join() clears the second. */
atomic_t thread_active_{ATOMIC_INIT(0)};
/* Not atomic and deliberately so: it is read and written only by start() and join(), both of which
 * are lifecycle calls from the commissioning side, never from the acquisition thread. */
bool thread_created_{false};
thread_config tcfg_{};
atomic_t stop_requested_{ATOMIC_INIT(0)};
/* Doubles as the cadence sleep. A stop request gives it, so the thread leaves its inter-cycle wait
 * immediately instead of finishing a full period first -- the difference between commissioning
 * waiting one cadence and waiting for a timeout it then reports as a refusal. */
K_SEM_DEFINE(stop_sem_, 0, 1);
atomic_t foreign_calls_{ATOMIC_INIT(0)};
/* The acquisition thread stops the devices on its way out, and the only caller that cares is
 * commissioning -- which does not see the thread, only join(). Without this the thread's stop
 * failure was a log line and nothing else, and try_stop() reported quiescence over a device
 * that was still ranging. */
atomic_t thread_stop_rc_{ATOMIC_INIT(0)};

/* True when the caller is allowed to drive the ULD: either no thread owns it, or this IS that
 * thread. Counting the refusals rather than only rejecting them, because a foreign call is a wiring
 * defect and a wiring defect that leaves no trace gets rediscovered instead of fixed.
 *
 * PRECONDITION, in the name: the chain lock is already held. Deciding ownership and then acquiring
 * the lock is the time-of-check-to-time-of-use this replaced. */
bool may_touch_devices_locked()
{
    if (atomic_get(&thread_active_) == 0 ||
        atomic_ptr_get(&owner_) == static_cast<void *>(k_current_get()))
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
// Defined below; stop_locked() needs it to record a stop that failed.
void record(source_facts &f, int rc, const op_status &st);

/* Returns 0 when every started source is stopped, otherwise the first failure.
 *
 * A device that would not stop stays started in the facts, and that is the whole point.
 * Recording it as stopped was a lie with teeth: try_stop() reported success, commissioning
 * took that as permission to drop enable lines while the device was still ranging, and the
 * later cleanup skipped it because the flag said there was nothing to stop. Quiescence is a
 * claim about hardware, so it cannot be established by clearing a bool.
 *
 * Every source is attempted even after one fails. Stopping three of four is strictly better
 * than stopping one and giving up, and the return value reports that it was not complete. */
/* Is any source NOT CONFIRMED STOPPED, as far as this module knows?
 *
 * stop_locked() deliberately leaves `started` set on a device whose stop failed. The device may
 * or may not still be ranging -- a failed stop means the result is unknown, not that ranging is
 * known to continue -- and `started` is the only record that the question is open. It is the one
 * flag that survives a failed stop, so an idle predicate ignoring it answers about the scheduler
 * rather than about the devices.
 *
 * Caller holds the chain lock. */
bool any_source_unquiesced_locked()
{
    for (int i{0}; i < facts_.source_count; ++i) {
        const source_facts &f{facts_.sources[i]};
        if (f.started || f.cleanup_pending)
            return true;
    }
    return false;
}

#if defined(ENABLE_TOF_L7_ULD)
/* The grid adapter's status, converted for the same reason the L4 one is: the stage is a raw
 * vendor number and means nothing without the domain that interprets it. */
source_status from_l7(const tof_l7::operation_status &st)
{
    source_status out{};

    out.domain = status_domain::l7;
    out.stage = static_cast<uint8_t>(st.failed_stage);
    out.port_errno = st.port_errno;
    out.uld_status = st.uld_status;
    out.sample_present = st.sample_present;
    return out;
}

void record_l7(source_facts &f, int rc, const tof_l7::operation_status &st)
{
    f.status = from_l7(st);
    if (rc == 0)
        return;
    if (rc == -EPROTO || rc == -EBADMSG)
        f.protocol_error = true;
    else if (rc == -ENOSYS)
        f.unsupported = true;
    else if (rc == -EINVAL || rc == -EPERM)
        f.usage_error = true;
    else
        f.transport_error = true;
}
#endif

#if defined(ENABLE_TOF_L7_ULD)
/* Bring one grid source up, with the SAME obligations the cliff path carries and one rule that is
 * simpler here.
 *
 * A FAILED START ALWAYS OWES A STOP. The cliff adapter can sometimes say the device was left quiet,
 * so it reports ranging_unknown and this layer only takes the obligation when it is set. The grid
 * adapter cannot: vl53l7cx_start_ranging() writes the start command and then polls and reads back,
 * so a failure anywhere after the command went out is consistent with a device that is ranging. So
 * the obligation is unconditional rather than reported, and the adapter agrees -- a failed start
 * leaves it in lifecycle::stop_unconfirmed, where it refuses to configure, start or be read until a
 * stop returns success.
 *
 * `started` is deliberately not set on any failure path: it means "may be read", and a source whose
 * session cannot be accounted for must not enter run_cycle(). */
void bring_up_grid_locked(int i, const source_desc &d, source_facts &f)
{
    tof_l7::operation_status st{};

    if (d.grid_ops == nullptr || d.grid_ops->open == nullptr ||
        d.grid_ops->configure == nullptr || d.grid_ops->start == nullptr ||
        d.grid_ops->read_grid_sample == nullptr || d.grid_ops->stop == nullptr) {
        f.usage_error = true;
        LOG_ERR("source %d is an l7_grid with no grid_ops table", i);
        return;
    }

    int rc{d.grid_ops->open(d.dev, d.addr_7bit, &st)};
    if (rc != 0) {
        record_l7(f, rc, st);
        LOG_WRN("grid source %d open failed at %s rc %d errno %d", i,
                tof_l7::stage_name(st.failed_stage), rc, st.port_errno);
        // One sensor that will not open must not stop the others from ranging.
        return;
    }

    rc = d.grid_ops->configure(d.dev, d.grid_frequency_hz, &st);
    if (rc != 0) {
        record_l7(f, rc, st);
        return;
    }

    rc = d.grid_ops->start(d.dev, &st);
    if (rc != 0) {
        record_l7(f, rc, st);
        f.cleanup_pending = true;
        f.rearm_failed = true;
        LOG_ERR("grid source %d start failed at %s rc %d -- the device may be ranging, a stop is "
                "owed", i, tof_l7::stage_name(st.failed_stage), rc);
        return;
    }

    f.started = true;
    /* Same place and same reason as the cliff path: only a complete open -> configure -> start
     * clears the sticky, so every early return above leaves it standing. */
    f.rearm_failed = false;
}

/* Stop one grid source. The flag discipline is the cliff path's, verbatim: both obligations stay
 * set when the stop is not confirmed, and only a stop that returned success discharges them. */
int stop_grid_locked(int i, const source_desc &d, source_facts &f)
{
    tof_l7::operation_status st{};

    if (d.grid_ops == nullptr || d.grid_ops->stop == nullptr)
        return -EINVAL;

    const int rc{d.grid_ops->stop(d.dev, &st)};
    if (rc != 0) {
        record_l7(f, rc, st);
        f.rearm_failed = true;
        /* SET, not merely left alone. The two flags are independent claims -- `started` says the
         * source may be read, `cleanup_pending` says it still owes a successful stop -- and an
         * unconfirmed stop changes both. Leaving them as they were kept the common case (started,
         * no debt) reading as a healthy readable source with nothing owed, which is the one thing
         * a device that would not quiesce is not. */
        f.started = false;
        f.cleanup_pending = true;
        LOG_ERR("grid source %d stop failed at %s rc %d errno %d -- stop not confirmed", i,
                tof_l7::stage_name(st.failed_stage), rc, st.port_errno);
        return rc;
    }
    f.started = false;
    f.cleanup_pending = false;
    return 0;
}
#endif

int stop_locked()
{
    running_ = false;

    int first_error{0};
    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};
        op_status st{};

        /* Both obligations, not just the readable one. A source that failed its second
         * arming call with the device left armed carries cleanup_pending without started;
         * iterating on started alone skipped it forever. */
        if (!f.started && !f.cleanup_pending)
            continue;

#if defined(ENABLE_TOF_L7_ULD)
        if (d.kind == model::l7_grid) {
            if (const int grid_rc{stop_grid_locked(i, d, f)}; grid_rc != 0 && first_error == 0)
                first_error = grid_rc;
            continue;
        }
#endif

        const int rc{d.ops->stop(d.dev, &st)};
        if (rc != 0) {
            record(f, rc, st);
            /* Sticky, for the same reason a failed re-arm is: a device whose stop cannot be
             * confirmed will not produce a trustworthy sample either, and only a complete
             * re-bring-up may clear it. */
            f.rearm_failed = true;
            /* SET, not merely left alone -- see stop_grid_locked(). An unconfirmed stop makes the
             * source unreadable AND leaves a stop owed, and the flags have to say both. */
            f.started = false;
            f.cleanup_pending = true;
            LOG_ERR("source %d stop failed at %s rc %d errno %d -- stop not confirmed", i,
                    tof_cliff_stage_name(st.stage), rc, st.port_errno);
            if (first_error == 0)
                first_error = rc;
            continue;
        }
        f.started = false;
        /* The obligation is discharged only by a stop that returned success. */
        f.cleanup_pending = false;
    }

    in_cycle_ = false;
    return first_error;
}

/* The L4 adapter's status, converted rather than assigned. The fields line up one for one; what
 * the conversion adds is the domain, without which `stage` is a number whose scale depends on who
 * wrote it. */
const char *stage_name_in_domain(const source_status &status)
{
    switch (status.domain) {
    case status_domain::none:
        return "none";
    case status_domain::l4:
        return tof_cliff_stage_name(static_cast<enum tof_cliff_stage>(status.stage));
    case status_domain::l7:
#if defined(ENABLE_TOF_L7_ULD)
        return tof_l7::stage_name(static_cast<tof_l7::stage>(status.stage));
#else
        /* An image without the grid driver has no vocabulary for an L7 stage and must not invent
         * one. It also cannot produce this value: nothing records an l7 domain without the driver
         * that fills it in. */
        return "l7";
#endif
    }
    return "unknown";
}

source_status from_l4(const op_status &st)
{
    source_status out{};

    out.domain = status_domain::l4;
    out.stage = static_cast<uint8_t>(st.stage);
    out.port_errno = st.port_errno;
    out.uld_status = st.uld_rc;
    out.sample_present = st.sample_present;
    return out;
}

void record(source_facts &f, int rc, const op_status &st)
{
    f.status = from_l4(st);
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

const char *operation_stage_name(const source_status &status)
{
    return stage_name_in_domain(status);
}

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
        /* ATOMIC, because this function has two callers on two threads: the acquisition path
         * reaches it through publish_snapshot() and publication_allowed(), and
         * health_work_handler() reaches it from the system workqueue. A plain bool read and
         * written from both is a data race -- undefined behaviour, not merely a warning that
         * might print twice.
         *
         * atomic_cas is the whole guard: exactly one caller sees the 0 and takes the
         * transition, every other caller sees 1 and skips. ATOMIC_INIT is a constant
         * initialiser, so this needs no thread-safe-statics guard of its own. */
        static atomic_t warned{ATOMIC_INIT(0)};
        if (atomic_cas(&warned, 0, 1)) {
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
    /* The third term is about the devices, not the scheduler. stop_locked() clears running_
     * and in_cycle_ even when a device refused to stop, keeping that source's `started` set
     * because its state is then unknown. Without this term the predicate reported the chain
     * free while a sensor might still have been driving the bus, and commissioning -- which
     * asks exactly this question before dropping enable lines -- would have proceeded to
     * re-address a device it could not confirm was stopped. */
    const bool idle{!running_ && !in_cycle_ && !any_source_unquiesced_locked()};
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

        /* ALL FIVE, not the two this function used to name. bring_up() calls configure()
         * and start() unconditionally and stop_locked() calls stop(), so a table accepted
         * here with any of them null does not fail at init -- it dereferences null on the
         * first lifecycle operation, which is a crash in the acquisition thread rather
         * than an -EINVAL to the caller who built the table. */
        /* PER MODEL, because the two paths do not use the same table and a single check could
         * only be wrong in one direction or the other. Requiring a complete d.ops of every source
         * rejected a correct descriptor that carried only grid_ops, and -- worse -- accepted a
         * grid descriptor whose grid_ops was null or half-filled, which then dereferenced null on
         * the first lifecycle operation: a crash in the acquisition thread rather than an -EINVAL
         * to the caller who built the table. */
#if defined(ENABLE_TOF_L7_ULD)
        if (d.kind == model::l7_grid) {
            if (d.grid_ops == nullptr || d.grid_ops->open == nullptr ||
                d.grid_ops->configure == nullptr || d.grid_ops->start == nullptr ||
                d.grid_ops->read_grid_sample == nullptr || d.grid_ops->stop == nullptr)
                return -EINVAL;
            /* The adapter is handed this as its device object on every call. */
            if (d.dev == nullptr)
                return -EINVAL;
            continue;
        }
#endif
        /* ALL FIVE, not the two this function used to name. In a build without the grid driver an
         * l7_grid source reaches this too, and its table is the stub -- which is complete, so the
         * flag-off behaviour is unchanged. */
        if (d.ops == nullptr || d.ops->open == nullptr || d.ops->configure == nullptr ||
            d.ops->start == nullptr || d.ops->read_cliff_sample == nullptr ||
            d.ops->stop == nullptr)
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
    /* Same three terms as is_idle(), and for the same reason: a source left `started` by a
     * failed stop is not confirmed stopped, and renumbering the sequence while that is open
     * risks reissuing (source_id, mapping_epoch, cycle_seq) triples -- the contract's
     * uniqueness guarantee is over the triple, not the cycle. Whether such a frame has ever
     * been emitted is not established here; the point is that the gate must not depend on it
     * not happening. */
    const bool idle{!running_ && !in_cycle_ && !any_source_unquiesced_locked()};
    if (idle)
        next_cycle_seq_ = 0;
    k_mutex_unlock(&tof_chain_controller::chain_lock());

    return idle ? 0 : -EBUSY;
}

int bring_up()
{
    if (!configured_)
        return -EINVAL;

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    if (!may_touch_devices_locked()) {
        k_mutex_unlock(&tof_chain_controller::chain_lock());
        return -EPERM;
    }
    in_cycle_ = true;

    /* Deliberately NOT resetting next_cycle_seq_ here. Restarting the cycle count without
     * advancing the epoch reissues (source_id, epoch, cycle_seq) triples that have already
     * been used, and the contract calls that a conflict. See the note on next_cycle_seq_.
     */

    /* RE-READ THE IDENTITY FROM THE DESCRIPTOR TABLE, here, every bring-up.
     *
     * init() runs at boot, long before any mapping has been proven, so the role ids it copied are
     * all unassigned. The proof's commit transaction later writes the real ones into the descriptor
     * table -- the same array cfg_.sources points at -- and nothing propagated them into facts_.
     * The copies stayed unassigned for the life of the process.
     *
     * That is not a cosmetic staleness. The publisher refuses to pack a sample whose descriptor
     * role and facts role disagree, counts it as suppressed_role_mismatch and marks the whole cycle
     * invalid; and facts_.sources[i].role_id is also the value it would have encoded into the
     * frame. So once the PROVEN clamp is lifted, a stale copy here suppresses EVERY measurement
     * while the board otherwise looks healthy -- health flowing, sensors reading, nothing in any
     * log to say why the wire is empty.
     *
     * Bring-up is the right place: it runs under the chain lock, from the thread that owns the ULD,
     * after the commit that keyed the descriptors, and again after every re-proof, because proving
     * stops acquisition and starting it brings the sources up afresh. install_from_mapping() is the
     * only production path that rewrites a role, and it cannot be reached without passing through
     * here afterwards.
     *
     * Observed on dasher2 before the fix: after a successful proof under epoch 5 the descriptors
     * read roles 0..3 and facts_ still read 255 for all four. */
    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};

        facts_.sources[i].kind = d.kind;
        facts_.sources[i].addr_7bit = d.addr_7bit;
        facts_.sources[i].role_id = d.role_id;
    }

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};
        op_status st{};
        int rc;

        /* A source that is already started is stopped FIRST, and its started flag is only
         * cleared when that stop succeeds.
         *
         * This used to clear the flag unconditionally before the retry. If the retry then
         * failed, the old device could still be ranging while the facts said it was stopped
         * -- so stop_locked() skipped it for the rest of the subsystem's life and the next
         * commissioning session re-addressed a live sensor. A re-bring-up that cannot
         * quiesce the previous device is not a re-bring-up; it is two drivers on one part. */
        if (f.started || f.cleanup_pending) {
            /* THROUGH THE MODEL'S OWN TABLE. This used to call d.ops->stop() for every source
             * before the dispatch below, so a grid source being restarted -- or retried while it
             * still owed a stop -- had the cliff driver pointed at it. The dispatch a few lines
             * down was doing the right thing for the bring-up and the wrong driver had already
             * been called for the quiesce. */
            int stop_rc;
#if defined(ENABLE_TOF_L7_ULD)
            if (d.kind == model::l7_grid) {
                stop_rc = stop_grid_locked(i, d, f);
            } else
#endif
            {
                op_status stop_st{};

                stop_rc = d.ops->stop(d.dev, &stop_st);
                if (stop_rc != 0) {
                    record(f, stop_rc, stop_st);
                    f.rearm_failed = true;
                    /* Same discipline as stop_locked(): unreadable, and a stop still owed. */
                    f.started = false;
                    f.cleanup_pending = true;
                    LOG_ERR("source %d could not be stopped before re-bring-up at %s rc %d", i,
                            tof_cliff_stage_name(stop_st.stage), stop_rc);
                }
            }
            /* A re-bring-up that cannot quiesce the previous device is not a re-bring-up; it is
             * two drivers on one part. The source is left unreadable, still owing a stop, and
             * nothing is opened. */
            if (stop_rc != 0)
                continue;
            f.started = false;
            f.cleanup_pending = false;
        }
        clear_cycle_outcomes(f);

#if defined(ENABLE_TOF_L7_ULD)
        if (d.kind == model::l7_grid) {
            bring_up_grid_locked(i, d, f);
            continue;
        }
#endif

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
            /* THE ADAPTER'S ranging_unknown IS AN OBLIGATION, AND IT USED TO BE DROPPED HERE.
             *
             * The cliff adapter issues StartMeasurement() before its second arming call. When
             * that call fails it attempts a best-effort StopMeasurement(), and when that fails
             * too it sets this: the device was armed and nothing has confirmed it is quiet.
             * This branch recorded the error and moved on, leaving `started` false -- so the
             * source was never read, which is right, but stop_locked() skipped it forever,
             * which is not. The device stayed armed for the rest of the subsystem's life and
             * the next commissioning session re-addressed it.
             *
             * `started` is deliberately NOT set: it means "may be read", and a source with no
             * usable stream must not re-enter run_cycle(). The obligation is carried
             * separately. */
            if (st.ranging_unknown) {
                f.cleanup_pending = true;
                f.rearm_failed = true;
                LOG_ERR("source %d start failed at %s rc %d and its cleanup did not confirm "
                        "the device is stopped -- a stop is owed", i,
                        tof_cliff_stage_name(st.stage), rc);
            }
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
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);

    if (!may_touch_devices_locked()) {
        k_mutex_unlock(&tof_chain_controller::chain_lock());
        return;
    }

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

#if defined(ENABLE_TOF_L7_ULD)
        if (d.kind == model::l7_grid) {
            tof_l7::sample grid{};
            tof_l7::operation_status grid_st{};

            const int grid_rc{
                d.grid_ops->read_grid_sample(d.dev, d.scratch, &grid, &grid_st)};
            record_l7(f, grid_rc, grid_st);
            /* `fresh` is the adapter's all-or-nothing answer: read_once clears the sample on entry
             * and leaves it non-fresh on every refusal, so a fresh sample is a whole one. A cycle
             * in which the sensor simply had nothing ready is rc 0 and not fresh -- not a failure,
             * and not a sample. */
            if (grid_rc == 0 && grid.fresh) {
                f.sample_produced = true;
                if (cfg_.hooks.on_grid_sample != nullptr)
                    cfg_.hooks.on_grid_sample(i, facts_.cycle_seq, f, grid);
            }
            continue;
        }
#endif

        rc = d.ops->read_cliff_sample(d.dev, d.scratch, d.stream, &sample, &st);
        record(f, rc, st);
        f.sample_produced = sample.fresh;

        // The payload goes to the model's own sink, so neither packer ever has to step
        // over the other model's data, and the fail direction stays out of here.
        if (f.sample_produced && d.kind == model::l4_cliff)
            cfg_.hooks.on_cliff_sample(i, facts_.cycle_seq, f, sample);
    }

    /* Per COMPLETED cycle -- which is why it is not at the top -- and UNDER THE LOCK, which is
     * why it is not after the hooks.
     *
     * It used to be the last line of this function, after the unlock and after on_cycle(). That
     * left a window in which this cycle was finished but had not consumed its number: another
     * caller could stop acquisition, call begin_epoch() -- which takes the same lock, finds the
     * chain idle and resets next_cycle_seq_ to 0 -- and then this line would increment the new
     * epoch's counter to 1. The first cycle of that epoch would be numbered 1, and the contract
     * requires cycles to be numbered from 0 per mapping_epoch.
     *
     * begin_epoch() cannot observe the half-finished state now, because it cannot hold this lock
     * until the increment is done. The hooks stay outside the lock, where they were: they must
     * not run with the chain held. */
    ++next_cycle_seq_;
    in_cycle_ = false;
    k_mutex_unlock(&tof_chain_controller::chain_lock());

    publish_snapshot();
    cfg_.hooks.on_cycle(facts_);
}

int teardown()
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
        return 0;
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

    /* RETIREMENT IS CONDITIONAL ON THE QUIESCE SUCCEEDING, and this return used to be
     * discarded.
     *
     * stop() returns an error and leaves that source's `started` set when a device would not
     * stop -- meaning its state is unknown, not that it is known to be ranging. Retiring on
     * top of that cleared configured_, which is the one thing holding init() to -EALREADY; the
     * next init() was then free to replace the descriptors and re-address a device nobody
     * could confirm had stopped, with no path back to the old ones and nothing left that would
     * retry the cleanup.
     *
     * So a failed quiesce keeps the subsystem exactly as it is: configured, its device state
     * intact, and the heartbeat still running -- that is when a consumer most needs to be told
     * the subsystem is alive and not producing. init() stays refused, and teardown() may be
     * called again once the fault clears. Nothing here is torn down by halves. */
    if (const int rc{stop()}; rc != 0)
        return rc;

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
    return 0;
}

int stop()
{
    if (!configured_)
        return 0;

    // Quiesce, then release. Commissioning may only enumerate once this returns: the
    // enumeration drops enable lines, which re-addresses parts underneath a reader.
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    if (!may_touch_devices_locked()) {
        k_mutex_unlock(&tof_chain_controller::chain_lock());
        return -EPERM;
    }
    const int rc{stop_locked()};
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
    return rc;
}

static void thread_entry(void *, void *, void *)
{
    /* The whole ULD lifecycle, on one thread, in one place.
     *
     * Per-source bring_up() failures are deliberately not fatal here: one dead cliff sensor must
     * not stop the other three from ranging, and the facts say which one it was. A total failure
     * still produces cycles -- empty ones -- and the health frame is what reports that, which is
     * better than a thread that exits and leaves a consumer to infer why.
     *
     * A non-zero RETURN is a different thing entirely, and used to be discarded with it. It means
     * bring_up() refused before reaching any source at all -- wrong configuration, or an ownership
     * record this thread did not recognise as its own -- so no source was even attempted and
     * running_ was never set. Every cycle after it is then a silent no-op. It is logged and
     * deliberately NOT folded into sensor_fault_mask: no sensor failed, and the heartbeat already
     * withholds cycle_valid while no cycle completes, so a consumer stays fail-closed on it. A
     * named producer-internal wire reason is its own design, not something to smuggle in by
     * reusing a sensor's bit. */
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
    atomic_set(&thread_stop_rc_, stop());
    /* Ownership is released under the same lock that every ownership decision is made under, so a
     * caller cannot observe the release half-applied. Released AFTER stop() returns, never before:
     * the devices this thread owns must be quiesced while it still owns them. */
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    atomic_set(&thread_active_, 0);
    atomic_ptr_set(&owner_, nullptr);
    k_mutex_unlock(&tof_chain_controller::chain_lock());
}

int start(const thread_config &tcfg)
{
    if (!configured_)
        return -EINVAL;
    /* thread_created_ as well as thread_active_: a thread that has exited but has not been joined
     * still owns the kernel object, and creating over it is undefined. A caller that requested a
     * stop and never joined gets -EALREADY, which is the honest answer -- it has not finished
     * stopping. */
    if (atomic_get(&thread_active_) != 0 || thread_created_)
        return -EALREADY;
    /* No defaults, the same rule the periods follow. A join timeout invented here would decide how
     * long commissioning waits before refusing, which is a deployment decision. */
    if (tcfg.stack == nullptr || tcfg.stack_size == 0 || tcfg.join_timeout_ms == 0)
        return -EINVAL;

    tcfg_ = tcfg;
    atomic_set(&stop_requested_, 0);
    k_sem_reset(&stop_sem_);
    /* The stop result belongs to a thread's LIFETIME, so it is initialised where that lifetime
     * begins and nowhere else.
     *
     * try_stop() used to clear it, and that handoff could not be made safe by moving the clear
     * earlier. request_stop() is callable from anywhere -- the shell, teardown(), a previous
     * try_stop() that timed out -- so by the time try_stop() runs, the thread may already have
     * stopped the devices, written a FAILING result and exited. Clearing before its own
     * request_stop() would still overwrite that, and clearing after is worse. The caller then
     * read 0 and reported a quiesce that never happened, which is exactly what lets
     * commissioning drop enable lines on a device that was never stopped.
     *
     * Here there is no such window: no thread exists yet to have written anything. */
    atomic_set(&thread_stop_rc_, 0);

    /* NOT touching next_cycle_seq_. Starting a thread is not the start of an epoch: the contract
     * numbers cycles from 0 per mapping_epoch, begin_epoch() does that as one step of the
     * authority's commit, and a reset here would either renumber a sequence a consumer is part-way
     * through or reissue a (source_id, epoch, cycle_seq) triple that has already been used. */
    /* Installed under the chain lock, so an owner cannot appear between another caller's
     * ownership decision and its use of the devices -- that caller is holding this lock while it
     * decides, so it cannot be mid-decision here. The new thread's first act is bring_up(), which
     * takes the same lock, so it waits for this to finish rather than racing it. */
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);

    /* K_FOREVER, then an explicit start, because the ownership record has to be COMPLETE before
     * the new thread can run a single instruction.
     *
     * With K_NO_WAIT the thread becomes runnable inside k_thread_create(), so a priority higher
     * than the caller's preempts right there -- before the return value has been assigned to
     * owner_. The new thread then calls may_touch_devices_locked(), finds thread_active_ already true
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
    k_tid_t const tid{k_thread_create(&thread_, tcfg_.stack, tcfg_.stack_size, thread_entry, nullptr,
                                      nullptr, nullptr, tcfg_.priority, 0, K_FOREVER)};
    atomic_ptr_set(&owner_, tid);
    thread_created_ = true;

    /* PUBLISHED LAST, and this ordering is for observers OUTSIDE this function.
     *
     * Suspended creation above settles what the NEW thread can see. It does nothing for a
     * caller on a third thread, because thread_running() is deliberately lock-free. With
     * thread_active_ set before k_thread_create(), such a caller could see "running" while
     * thread_ and owner_ were still uninitialised, enter try_stop(), and call k_thread_join()
     * on a kernel object that did not exist yet.
     *
     * Everything a caller reaches through that flag -- the thread object join() waits on, the
     * owner the ownership decision reads, the created flag teardown() checks -- is therefore
     * installed first, and the flag that advertises them is the last write before the unlock. */
    atomic_set(&thread_active_, 1);
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    k_thread_start(tid);
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
    /* Lock-free on purpose. try_stop() asks this before deciding whether it may touch the chain at
     * all, and taking the chain lock to answer would block a shell thread behind a cycle in
     * progress -- the exact wait try_stop() exists to avoid. The atomic read is sound on its own:
     * this reports a fact, it does not authorise touching a device. */
    return atomic_get(&thread_active_) != 0;
}

uint32_t foreign_lifecycle_calls()
{
    return static_cast<uint32_t>(atomic_get(&foreign_calls_));
}

#ifdef CONFIG_ZTEST
k_tid_t thread_id_for_test()
{
    return static_cast<k_tid_t>(atomic_ptr_get(&owner_));
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
    if (thread_running()) {
        request_stop();
        /* NOT cleared here. The thread may already have run its stop, recorded a failure and
         * exited -- request_stop() is callable from anywhere and may have been called long
         * before this. Clearing at any point in this function overwrites a result that is
         * already final. start() initialises it, once, where the thread's lifetime begins. */
        if (int const rc{join(tcfg_.join_timeout_ms)}; rc != 0)
            return rc;
        /* Joined, so the thread has run its stop and recorded the outcome. A thread that
         * exited cleanly but could not stop a device has NOT quiesced the chain, and saying
         * 0 here is what let commissioning drop enable lines on a live sensor. */
        return static_cast<int>(atomic_get(&thread_stop_rc_));
    }

    if (k_mutex_lock(&tof_chain_controller::chain_lock(), K_NO_WAIT) != 0)
        return -EBUSY;   // somebody else owns the chain; nothing touched, so refusing is safe

    const int rc{stop_locked()};
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    publish_snapshot();
    if (rc != 0)
        return rc;

    /* Idle by construction rather than by a second query: the state was set under the lock we
     * just held, and re-reading it through is_idle() would take the chain again with K_FOREVER --
     * reintroducing the block this function exists to remove. */
    return 0;
}

}  // namespace lexxhard::tof_acq

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
