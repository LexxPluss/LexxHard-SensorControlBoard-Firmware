/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

// PROTOTYPE SKELETON - THE INTERFACE IS NOT FROZEN AND THIS IS NOT PRODUCTION READY.
//
// It exists to establish the boundaries and to make the budget measurable with a
// scheduler in the image. Known and deliberate limitations, all of which will change:
//
//   - The data interface is still L4-shaped. read_cliff_sample() hands back a
//     tof_cliff_sample and there is one payload sink, which suits four point sensors and
//     does not suit an 8x8 grid. When the real L7 path lands, the ops table and the sinks
//     both change shape; the grid stub refusing a cliff-shaped read is the visible marker
//     of that debt, not a design.
//   - There is no production caller at this tip. The packer, the publisher, the CAN glue
//     and tof_cliff_runtime::bootstrap() are the layer above and are not in this PR; what
//     is here is a standalone lifecycle driven by tests. The header used to describe the
//     wired-up arrangement as though it already existed, which claimed a reachable
//     production path a reader would then go looking for. Update this when the downstream
//     runtime lands.
//   - PROVEN is unreachable by construction, see effective_mapping_state().
//   - Stack watermark and boot time are unmeasured; both need a run on the board, and the
//     acquisition thread's stack size is a devicetree value chosen without one.
//
// The six-sensor acquisition skeleton: one thread, one cycle at a time, sequential
// over the configured sources. What it produces is a set of NEUTRAL FACTS about the
// cycle. What it deliberately does not contain is any policy.
//
// WHY NO SHARED POLICY
//
// The two features fail in opposite directions. A hanging-object miss should not stop
// the robot, so the grid path's safe answer to "no data" is to say nothing; a cliff
// miss must stop the robot, so the cliff path's safe answer to the same silence is to
// assert a fault. Any shared decision about what a missing sample "means" would be
// wrong for one of them. So this layer records what happened - a sample arrived, a
// transfer failed, metadata was impossible - and each feature's own packer applies its
// own direction to the same facts.
//
// LOCKING
//
// Every device operation happens on ONE thread -- the acquisition thread, created by start() --
// while it holds tof_chain_controller::chain_lock(), because the chain control lines and the bus
// are a single shared resource. The lock is not the whole rule: serialised is not single-owner, and
// the ULD's port keeps one file-scope transport record, so bring_up(), run_cycle() and stop() refuse
// callers other than that thread while it exists. See the ownership rule above bring_up().
//
// Manual commissioning takes the same lock for its whole session, and additionally must not run
// while acquisition is live: enumeration drops enable lines, which re-addresses parts underneath a
// reader. try_stop() is what commissioning calls -- it asks the thread to stop and joins with an
// injected bound, and refuses rather than waiting or killing.
//
// PUBLICATION GATE
//
// Role measurements from either path require mapping_state() == PROVEN: an unproven
// mapping means a reading might be attributed to the wrong position on the robot, which
// is worse than no reading. Cliff HEALTH is the exception and is sent from startup
// onwards, carrying NOT_READY or FAULT - a consumer that hears nothing at all cannot
// distinguish a booting board from a dead one.
//
// THE HEARTBEAT IS NOT ON THE ACQUISITION PATH
//
// Bringing up four L4s is a sequence of blocking device operations, and a part that
// never answers can stall it. If the health heartbeat shared that path, the most
// important safety signal would disappear exactly when something went wrong. So the
// heartbeat is a timer and work item that reads one atomic snapshot the acquisition
// thread publishes, and needs neither the lock nor the thread to make progress.
//
// NO ENABLE OPERATION, HERE EITHER
//
// The source ops carry open, configure, start, read and stop, and nothing else. The L4s
// stay enabled for the whole session and are polled at their assigned addresses.

// Both guards are load-bearing. src/*.cpp is globbed, so this file is a source of every
// build; ENABLE_TOF_CHAIN is what makes the chain exist at all, and ENABLE_TOF_CLIFF_ULD
// is what puts the cliff sensor's headers on the include path. A chain build without the
// ULD compiled this file and failed on vl53l4cx.h.
#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <zephyr/kernel.h>

#include "tof_cliff_sensor.h"

#if defined(ENABLE_TOF_L7_ULD)
/* For grid_source_ops and the grid sample hook. An image without the grid driver has no grid
 * sources to describe, and these types do not exist in it. */
#include "tof_l7_sample.hpp"
#include "tof_l7_status.hpp"
#endif
#include "tof_mapping_state.hpp"

namespace lexxhard::tof_acq {

// Four cliff L4 plus two grid L7.
constexpr int kMaxSources{6};

enum class model : uint8_t {
    l4_cliff,
    l7_grid,
};

// mapping_state now lives in tof_mapping_state.hpp, included above: the authority needs the
// enum without the vendor ULD this header drags in.

// The diagnostic record is shared between models; the POLICY is not. Reusing one struct
// for "where did it fail and with which errno" costs nothing and keeps one triage
// vocabulary. It says nothing about what the failure means.
using op_status = struct tof_cliff_read_status;


/* WHICH VENDOR'S VOCABULARY `stage` IS IN. The two ULDs' enums are unrelated numbers that happen
 * to share a range, so a stage is meaningless without the domain that interprets it. `none` is the
 * state of a source that has not been read this cycle -- not a third device. */
enum class status_domain : uint8_t {
    none,
    l4,
    l7,
};

/* A MODEL-NEUTRAL DIAGNOSTIC SNAPSHOT, and the reason this type exists at all.
 *
 * `source_facts::status` used to be an op_status -- that is, literally a tof_cliff_read_status --
 * for every source including a grid sensor. An L7 failure had to be expressed in L4 words or not
 * at all, and `stage` silently meant a different thing depending on which sensor filled it in.
 * This carries the domain with the number so the two can never be read as the same scale.
 *
 * WHAT IT DOES NOT ABSORB, deliberately. The four error classifications on source_facts stay where
 * they are: this is raw device detail, and they are the per-cycle MEANING, which was decided in
 * review rather than derived from an errno. Nor does it carry rearm_failed or cleanup_pending --
 * those are sticky facts about the device across cycles, and a per-read snapshot is the wrong
 * place for them. The development branch's version of this struct had its own rearm_failed beside
 * source_facts', which is two fields with one name and two lifetimes. */
struct source_status {
    status_domain domain{status_domain::none};
    /* Raw, and interpreted only inside `domain`. */
    uint8_t stage{0};
    int port_errno{0};
    int uld_status{0};
    bool sample_present{false};
};

/* Names the stage in its own domain. Returns "none" for a source nothing has read yet. */
const char *operation_stage_name(const source_status &status);

// One source's device operations. There is no enable, no address change and no reset:
// the enable line is the chain's addressing mechanism and belongs to commissioning.
//
// read_cliff_sample is named for what it actually is. A generic name would hide that this
// table is currently the shape of a point sensor, and the grid path cannot be expressed
// through it: an 8x8 zone frame is not a tof_cliff_sample. The name is the reminder that
// this signature has to change, rather than a claim that it is already general.
struct source_ops {
    int (*open)(void *dev, uint8_t addr_7bit, op_status *st);
    int (*configure)(void *dev, op_status *st);
    // start() takes the stream state because a successful start begins a new numbering
    // and the replay guard's history has to be dropped inside the same call, not by a
    // caller that might forget.
    int (*start)(void *dev, void *stream, op_status *st);
    int (*read_cliff_sample)(void *dev, void *scratch, void *stream,
                             struct tof_cliff_sample *out, op_status *st);
    int (*stop)(void *dev, op_status *st);
};

// The grid ops, every one of which returns -ENOSYS. An explicit stub rather than a null
// pointer or a copy of the cliff ops, so that wiring L7 to the wrong driver is a
// deliberate act rather than an oversight.
const source_ops &l7_stub_ops();

// The cliff ops, bound to the real tof_cliff_sensor functions.
const source_ops &l4_cliff_ops();

/* THE GRID PATH'S OWN TABLE, because an 8x8 zone frame is not a tof_cliff_sample and no amount of
 * naming makes source_ops carry one. The two tables are deliberately not unified: the status types
 * are each vendor's own, and a common one would have to be the union of two unrelated vocabularies.
 *
 * There is no stream argument. The L4 adapter's replay guard compares a stream count against the
 * previous one from the same device; the grid adapter has no such history to keep. */
#if defined(ENABLE_TOF_L7_ULD)
struct grid_source_ops {
    int (*open)(void *dev, uint8_t addr_7bit, tof_l7::operation_status *st);
    int (*configure)(void *dev, uint8_t frequency_hz, tof_l7::operation_status *st);
    int (*start)(void *dev, tof_l7::operation_status *st);
    int (*read_grid_sample)(void *dev, void *scratch, tof_l7::sample *out,
                            tof_l7::operation_status *st);
    int (*stop)(void *dev, tof_l7::operation_status *st);
};
#else
/* An image without the grid driver still has the descriptor field, so a build that does not carry
 * the ULD does not need a different source_desc. The table can only ever be null there. */
struct grid_source_ops;
#endif

struct source_desc {
    model kind{model::l4_cliff};
    uint8_t addr_7bit{0};
    // Opaque to this layer. The mapping owns what a role means; treating it as a number
    // here is what keeps position policy out of the scheduler.
    uint8_t role_id{0};
    /* EXPLICIT FOR A GRID SOURCE, with no scheduler default, for the same reason the cliff
     * cadence has none: a number invented at this level becomes the specification by being the
     * only one available. Zero means the caller has not configured this source, and configure()
     * refuses rather than picking something. Unused by an l4_cliff source. */
    uint8_t grid_frequency_hz{0};
    void *dev{nullptr};      // VL53L4CX_Object_t* for l4_cliff
    void *scratch{nullptr};  // tof_cliff_scratch* for l4_cliff, deliberately SHARED
    // tof_cliff_stream_state* for l4_cliff, and deliberately NOT shared: the replay
    // guard compares a stream count against the previous one from the SAME device, so
    // one instance per source. It lives here rather than in source_facts because
    // source_facts is per cycle and cleared, and a history that is cleared every cycle
    // cannot detect a repeat across cycles - which is the only thing it is for.
    void *stream{nullptr};
    const source_ops *ops{nullptr};
    /* For an l7_grid source, and the only table used for one. A descriptor carrying the wrong one
     * for its kind is a configuration error the bring-up refuses rather than works around. */
    const grid_source_ops *grid_ops{nullptr};
};

// What happened to one source in one cycle. No classification, no reduction, no alarm.
struct source_facts {
    model kind{model::l4_cliff};
    uint8_t addr_7bit{0};
    uint8_t role_id{0};
    bool configured{false};      // a source occupies this slot
    bool started{false};         // start() succeeded during bring-up
    bool sample_produced{false}; // a fresh sample arrived in this cycle

    // Four distinct PER-CYCLE outcomes, because collapsing them would decide health
    // semantics by accident: a stubbed model is not a broken sensor, and a bad call of our
    // own is not a bus fault. At most one is set, all four are cleared together before
    // each read of a started source, and none of them is cleared for a source that never
    // started. rearm_failed below is NOT one of them -- see its own note.
    bool transport_error{false}; // -EIO and friends: the transfer or the driver failed
    bool protocol_error{false};  // -EPROTO: the device's own metadata was impossible
    bool unsupported{false};     // -ENOSYS: this model has no implementation yet
    bool usage_error{false};     // -EINVAL: this firmware called it wrongly

    /* STICKY, and the only field in this struct that is. It describes the DEVICE, not the
     * cycle: "this sensor will not produce another sample". It is set one-way and survives
     * every later cycle, including cycles that fail for some other reason, because a
     * wedged sensor and a merely quiet one are otherwise bit-for-bit identical on the
     * published word -- stopped device, nothing ready, read_once returns 0, no outcome
     * recorded. Only a complete, successful re-bring-up (open -> configure -> start, all
     * three) clears it; a recovery that fails at any stage leaves it standing. It is also
     * OR-ed into the snapshot's per-source fault bit, since that word is the only signal
     * guaranteed to be sent. */
    bool rearm_failed{false};

    /* STICKY TOO, and it answers a different question from `started`.
     *
     * `started` means "this source may be read". `cleanup_pending` means "this source still
     * owes a successful stop". They are usually the same, and the case that separates them is
     * the one this field exists for: the cliff adapter issues StartMeasurement() before its
     * second arming call, and when that second call fails it attempts a best-effort
     * StopMeasurement(). If THAT also fails it reports `ranging_unknown` -- the device was
     * armed and nothing has confirmed it is quiet.
     *
     * Such a source must not be read: it never completed bring-up and has no usable stream, so
     * `started` stays false. But it must also not be skipped by the cleanup, which is what
     * iterating on `started` alone did -- the device was left armed for the rest of the
     * subsystem's life and the next commissioning session re-addressed it.
     *
     * Cleared only by a stop that returns success. Recovery is a stop, never a retried start:
     * a half-armed device has to come down before it can go up. */
    bool cleanup_pending{false};
    /* The last device-level detail recorded for this source, in the vocabulary of whichever model
     * produced it. See source_status: the domain is part of the value because `stage` is a raw
     * vendor number and the two vendors' enums are unrelated. */
    source_status status{};
};

struct cycle_facts {
    // The contract's unit of correlation between a measurement frame and a health
    // frame. Incremented once per cycle by this layer and by nobody else.
    uint32_t cycle_seq{0};
    uint32_t began_ms{0};
    int source_count{0};
    source_facts sources[kMaxSources]{};
};

// Sinks. The payload goes to the model's own sink, so no packer ever has to skip past
// another model's data, and the neutral facts go to both.
struct sinks {
    // Fires ONCE at the top of a cycle, before the first sensor is read.
    //
    // It exists because a cycle can legally produce zero measurements -- the contract allows
    // between zero and four -- and a sink that latched its per-cycle state on the first sample
    // would never latch at all for such a cycle. The consumer must still be told that the cycle
    // happened and produced nothing, otherwise "completed with no samples" and "never happened"
    // are the same silence.
    void (*on_cycle_begin)(uint32_t cycle_seq);
    void (*on_cycle)(const cycle_facts &facts);
    // Raw, unclassified, unreduced. index is the source index in the descriptor table.
    //
    // cycle_seq is passed rather than left to be looked up, because the wire contract
    // correlates a measurement with its health frame by it and this is the only place the
    // sample and its cycle are both in hand. on_cycle fires AFTER every sample, so a sink
    // latching the value from there would stamp the previous cycle; and reading it back
    // through copy_facts() is worse -- this call happens under the chain lock.
    void (*on_cliff_sample)(int index, uint32_t cycle_seq, const source_facts &facts,
                            const struct tof_cliff_sample &sample);
    // Sent from startup, on its own timer, never from the acquisition path.
    void (*on_cliff_health)(uint32_t snapshot, mapping_state state);
#if defined(ENABLE_TOF_L7_ULD)
    /* The grid equivalent of on_cliff_sample, and separate for the same reason the ops tables are:
     * an 8x8 zone frame is not a tof_cliff_sample. Fires under the chain lock, after the read and
     * before on_cycle, so a sink sees one consistent ordering across both models. */
    void (*on_grid_sample)(int index, uint32_t cycle_seq, const source_facts &facts,
                           const tof_l7::sample &sample);
#endif
};

// Both are unresolved symbols in the cliff wire contract, so neither has a default and
// zero is rejected. A placeholder frozen here would become the de facto specification.
struct timing {
    uint32_t cycle_period_ms{0};
    uint32_t health_period_ms{0};
};

struct config {
    const source_desc *sources{nullptr};
    int source_count{0};
    struct timing periods{};
    struct sinks hooks{};
    mapping_state (*mapping_state_provider)(){nullptr};
    uint32_t (*now_ms)(){nullptr};  // injected so tests need no wall clock
};

// Validates the configuration and arms the heartbeat. Returns -EINVAL for a zero
// period, a missing hook, more sources than kMaxSources, or a source without ops.
int init(const config &cfg);

/* THE OWNERSHIP RULE for everything below that touches a sensor.
 *
 * bring_up(), run_cycle() and stop() are the ULD lifecycle. While an acquisition thread exists,
 * ALL THREE belong to that thread and nobody else may call them: they return -EPERM (or, where the
 * signature has no room to say so, return without doing anything and count the attempt in
 * foreign_lifecycle_calls()).
 *
 * This is not tidiness. The ULD's port keeps its transport record in one file-scope variable -- the
 * BSP IO callbacks carry no per-device context -- so a second caller anywhere inside these
 * functions can clear the record belonging to the first, and the failure mode is a transport error
 * reported as a good sample. The chain lock serialises them, but serialised is not the same as
 * single-owner: a second thread holding the lock in turn still interleaves bring-up with cycles.
 *
 * With no thread running, direct calls are allowed and are how the host suites and a single
 * commissioning cycle drive the chain. The rule is "while a thread owns the ULD, nobody else
 * touches it", not "these functions are private".
 */

// Bring-up: open, configure and start every source, in table order, under the lock.
// Failures are recorded per source and do not stop the others - one dead cliff sensor
// must not prevent the other three from ranging.
//
// Returns -EPERM if called from anywhere but the acquisition thread while one is running.
int bring_up();

// Starts a new mapping epoch's cycle numbering: resets cycle_seq to 0.
//
// Only legal while acquisition is idle -- -EBUSY otherwise, and idle means BOTH stopped and
// not mid-cycle. The check and the reset happen together under the chain lock, so a cycle
// cannot start in between: a renumber under a live epoch reissues (source_id, mapping_epoch,
// cycle_seq) triples that have already been used. Returns -EINVAL before init().
//
// The epoch VALUE does not live here. This layer owns the cycle counter and nothing else;
// the authority owns the epoch and calls this as one step of its commit.
int begin_epoch();

// One cycle: read every started source once, sequentially, under the lock. Never waits
// on a source; a source with nothing ready simply has sample_produced false.
//
// Does nothing when called from a thread that is not the acquisition thread while one is running.
void run_cycle();

/* The acquisition thread, and everything it needs, injected.
 *
 * No defaults anywhere in here. The cadence is config::periods.cycle_period_ms, which is already
 * required; these three are the same kind of decision and get the same treatment. A stack size or a
 * join timeout invented in this file would become the specification by being the only number
 * anybody could find -- and a join timeout in particular decides how long commissioning waits
 * before refusing, which is a deployment decision, not a scheduler's.
 *
 * The stack is caller-owned because it has to be: a Zephyr thread stack is a compile-time sized
 * object, so it is defined where its size is known -- tof_cliff_runtime, from a required devicetree
 * property -- and passed in here.
 */
struct thread_config {
    k_thread_stack_t *stack{nullptr};
    size_t stack_size{0};
    // Not range-checkable: every value is a legal Zephyr priority, including 0. The devicetree
    // property being REQUIRED is the whole guarantee that somebody chose it.
    int priority{0};
    // How long a caller's bounded join waits before giving up. Zero is refused.
    uint32_t join_timeout_ms{0};
};

/* ----------------------------------------------------------------- the cadence ------ */
/*
 * WHAT THE LOOP USED TO DO, AND WHY IT WAS NOT A PERIOD. It ran a cycle and then waited a whole
 * `cycle_period_ms`, so the interval between two cycles was the work PLUS the period: at a 20 ms
 * setting and 6 ms of work the real cadence was 26 ms, and it moved with whatever each cycle
 * happened to cost -- a bus retry, a sensor that answered slowly. The configured period was
 * therefore not the rate, and no number anywhere said what the rate was.
 *
 * The rule now is a deadline rather than a delay: each cycle is due one period after the deadline
 * the previous one met, not one period after it finished, so the work happens INSIDE the cadence.
 *
 * A deadline that has already passed is not waited out and is not slept off for a further period
 * either -- at a 20 ms target a cycle taking 20.1 ms would then run at 40.1 ms, about 25 Hz, which
 * is a rate thrown away over a fraction of a millisecond. It yields and re-bases on now, so a late
 * cycle owes no backlog, and it is COUNTED: when the work is longer than the period the best
 * achievable cadence is the work itself, and nothing else in the firmware would say so.
 *
 * A deadline further ahead than one period is not reachable -- each is the previous plus a period,
 * and the loop does not return until it has waited for it -- but a clock that stepped backwards, or
 * a deadline computed before a re-init, could produce one. It is REPAIRED rather than obeyed:
 * waiting it out would stall acquisition for however wrong the number was, silently, with the
 * heartbeat still flowing and nothing to say why the measurements stopped. It is not an overrun,
 * because nothing was late -- and the deadline it rebuilds is one period after the wait it is about
 * to perform, not one period after now, or the cycle that follows the repair would be born already
 * due and counted late. That miscount is bounded to one cycle, since the overrun branch re-bases on
 * now; it is still worth getting right, because the overrun count is the one signal that says the
 * period is too short for the work.
 *
 * `due_ms` and `now_ms` are milliseconds from a monotonic clock. A period of zero is not reachable
 * -- init() refuses one -- and is treated as an overrun rather than splitting the responsibility
 * for that check between two places.
 */
struct schedule_decision {
    /* Meaningful only when yield_only is false, and never zero then. */
    uint32_t wait_ms{0};
    /* Wait the shortest interval the kernel can express instead of wait_ms. Set exactly when the
     * deadline had already passed -- see the overrun rule above. */
    bool yield_only{false};
    int64_t next_due_ms{0};
    bool overran{false};
};

/* Pure, and separated from the loop for that reason: the rule above is the part that can be wrong,
 * and a thread with a sleep in it is not where a rule gets tested. */
schedule_decision next_cycle_due(int64_t due_ms, int64_t now_ms, uint32_t period_ms);

/* The decision turned into the timeout the loop passes to the kernel.
 *
 * Its own function because the interesting property lives here and nowhere else: it must never
 * return K_NO_WAIT. A `yield_only` decision carries no millisecond figure -- a tick is 0.1 ms on
 * this board, and every millisecond value standing for "briefly" rounds to zero -- so a loop that
 * read wait_ms regardless would turn every missed deadline into a spin, and every test of
 * next_cycle_due() would still pass. The loop's own line is then a pass-through with nothing left
 * to get wrong. */
k_timeout_t cadence_timeout(const schedule_decision &step);

/* Cycles that were already past due when they finished.
 *
 * NOT performance telemetry, and deliberately the only number this change adds. A cadence that
 * cannot be met is not an error -- no sensor failed and no frame was lost -- so it is counted
 * rather than logged per cycle. But it is the ONE fact that distinguishes "running at the
 * configured rate" from "running as fast as the work allows", and without it a period set too short
 * for the work degrades silently while every other indicator stays healthy. Cleared by init().
 *
 * READABLE ON A BOARD, which it was not when this counter was added. Review pointed out that
 * nothing outside the tests read it, so the degradation it exists to reveal was still invisible
 * without a debugger -- the counter was the fix's own blind spot. There are two readers now: the
 * loop logs once on the transition from zero, and `tof cliff status` prints the running total
 * beside the period it is measured against. */
uint32_t cycle_overruns();

/* The cadence init() was last given, or 0 when none has been accepted.
 *
 * Exists so a reader does not have to reach into the configuration the acquisition thread is
 * using: this is one atomic word, written only on a successful init(), and a refused init() leaves
 * the previous answer standing. 0 is not a period -- init() rejects it -- so 0 means "no cadence
 * has been configured", which is the distinction a status reader needs first. */
uint32_t configured_cycle_period_ms();

/* Starts the acquisition thread: bring-up, then one cycle per cadence period until asked to stop.
 *
 * Returns -EINVAL before init() or for a config with no stack or a zero join timeout, -EALREADY if
 * a thread is already running.
 *
 * Deliberately does NOT touch the cycle counter. Starting a thread is not the start of a mapping
 * epoch: the contract numbers cycles from 0 per epoch, begin_epoch() is what resets them as one step
 * of the authority's commit, and a reset here would renumber a sequence a consumer is half-way
 * through -- or, worse, restart at 0 under an epoch whose 0 has already been used.
 */
int start(const thread_config &tcfg);

/* Asks the thread to stop. Returns immediately, from any thread, and is idempotent.
 *
 * The thread finishes the cycle it is in, calls stop() itself -- at a cycle boundary, from the
 * thread that owns the ULD -- and exits. A caller that needs to know it has finished calls join().
 */
void request_stop();

/* Waits for the thread to exit, for at most timeout_ms. Returns 0 when it has exited (or was never
 * running), -EBUSY on timeout.
 *
 * A timeout is NOT escalated to anything stronger. There is no way to abort a thread that may be
 * inside a vendor driver holding the chain lock and a half-finished I2C transaction; a caller that
 * cannot get a clean stop must refuse to proceed, which is what commissioning does.
 */
int join(uint32_t timeout_ms);

bool thread_running();

// How many times a foreign thread tried to drive the ULD while the acquisition thread owned it.
// Diagnostics for the ownership rule: it must stay zero.
uint32_t foreign_lifecycle_calls();

#ifdef CONFIG_ZTEST
/* The owning thread's id, so a suite can assert that every recorded ULD call came from it. Test-only
 * because production has no use for it: the rule is enforced by the guard, not by inspection. */
k_tid_t thread_id_for_test();
#endif

// Quiesces acquisition and RELEASES THE CHAIN. Deliberately leaves the health timer running:
// the contract requires health to keep flowing while no acquisition runs, which is exactly when
// a consumer needs to know the subsystem is alive and why it is idle. Use teardown() to stop the
// heartbeat as well.
//
// BLOCKS on the chain lock. Fine for a shutdown path that has nothing better to do; wrong for
// commissioning, which must fail fast rather than queue up behind whoever holds the chain.
// Returns 0 when every started source stopped, the first failing rc otherwise, and -EPERM
// when the caller does not own the chain. A non-zero return means at least one device is
// STILL RANGING and its source_facts still say `started`, which is what stops a later
// cleanup from skipping it.
int stop();

// The commissioning quiesce: the same thing stop() does, except that it never waits for the
// chain.
//
// Returns 0 when acquisition is quiesced and the chain was free to do it in, -EBUSY when the
// chain is held by somebody else -- and in that case NOTHING has been changed, so a caller that
// gets -EBUSY can simply refuse.
//
// This exists because the whole point of the commissioning session taking the chain with
// K_NO_WAIT is defeated if the step BEFORE it blocks. The first version of the shell command
// called stop(), which takes the chain with K_FOREVER: a run that collided with a live chain
// user would hang in the quiesce and never reach the non-blocking acquire it was written to
// rely on. The refusal has to start here or it does not exist.
//
// WITH A THREAD RUNNING it does not touch a device at all: it requests a stop and joins with the
// injected timeout, and the thread is what calls stop(). That is the ownership rule -- the shell
// thread stopping devices out from under a thread that is mid-cycle is exactly the interleaving the
// ULD's single transport record cannot survive. A join timeout returns -EBUSY and nothing is killed.
//
// WITH NO THREAD, it quiesces directly under the chain lock, which is how commissioning and the host
// suites use it: with no owner, there is nobody to interleave with.
int try_stop();

// The real shutdown: stop(), then the heartbeat, then a SYNCHRONOUS cancel of any health work
// already submitted. Separate from stop() so that pausing acquisition and retiring the
// subsystem cannot be confused -- a consumer must be able to tell a controlled pause from a
// silence that looks like a crashed producer.
//
// ON A ZERO RETURN, and only then, no further health frame can be emitted. Stopping the timer
// alone does not give that: a work item submitted by the last tick may still be queued or
// running, so a frame could go out after teardown claimed the subsystem was down.
//
// Returns:
//   0        nothing was configured, or everything stopped and the subsystem is retired.
//   nonzero  stop() refused: at least one device is not confirmed stopped. NOTHING is retired --
//            the configuration, the device state and the heartbeat all stay exactly as they
//            were, deliberately, because that is when a consumer most needs to be told the
//            subsystem is alive and not producing. Call it again once the fault clears.
//
// A caller that ignores the return keeps a live subsystem it believes is retired. The
// difference matters because clearing the configuration is what releases init() from
// -EALREADY: a discarded error let the next init() replace the descriptors and re-address a
// device nobody could confirm had stopped.
//
// Until it returns zero, a second init() is refused with -EALREADY rather than overwriting a
// live configuration underneath a work item that is reading it.
int teardown();

// True only when the chain is genuinely free: not mid-cycle AND not running. The
// distinction matters because commissioning drops enable lines, which re-addresses parts;
// the gap between two cycles is not a safe window, it is simply a short one.
bool is_idle();

// Copies the current facts out under the lock. Bring-up produces no cycle, so its
// per-source outcome would otherwise be readable only as snapshot bits; a caller that
// needs to know which sensors came up - and why the others did not - reads it here.
void copy_facts(cycle_facts &out);

// The snapshot the heartbeat reads. Packed into one 32-bit word so it can be read without
// the chain lock: mapping state in bits 0-1, then one bit per source for sample_produced,
// fault, configured and started.
//
// The fault bit is transport_error, protocol_error OR usage_error - our own bad call is a
// real defect and has to be loud. It deliberately excludes `unsupported`, because a model
// with no implementation is a static property of this build rather than something that
// happened to a sensor this cycle.
//
// A source that never started keeps the fault bit its bring-up produced: that source is
// faulted, and the cycles do not clear the diagnosis of a source they never read.
uint32_t snapshot();

// The state the rest of the system should act on, which is not what the provider returns:
// PROVEN is ALWAYS clamped to NOT_READY, because a chain that cannot be enumerated across
// both boards cannot have a proven mapping.
//
// There is deliberately no build flag to lift this. A conditional bypass of a safety gate
// is one careless -D away from shipping, and nothing about it would appear in a diff of
// the code it disables. Re-enabling PROVEN is an edit to this function in a commit of its
// own, reviewed against the fixed hardware.
mapping_state effective_mapping_state();

// The clamp on its own, for a caller that already holds a state and must not read a second
// one. production_authorisation() needs exactly this: it takes the state and the epoch from
// ONE authority snapshot, and going back through effective_mapping_state() for the state
// would read the authority twice and could pair a state with an epoch that never existed
// together. Clamping here rather than at the caller keeps the clamp in one place.
mapping_state clamp_mapping_state(mapping_state reported);

// True only when a role measurement may be published at all.
bool publication_allowed();

}  // namespace lexxhard::tof_acq

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
