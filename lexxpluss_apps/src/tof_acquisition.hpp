/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
//   - Neutral facts are recorded, but nothing consumes them yet: no packer, no CAN glue,
//     no periodic measurement frame.
//   - No thread is created here. Something has to call bring_up() and run_cycle() on the
//     single acquisition thread; the cycle period is carried but not yet used to pace it.
//   - PROVEN is unreachable by construction, see effective_mapping_state().
//   - Stack watermark and boot time are unmeasured; both need a run on the board.
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
// Every device operation happens on this one thread while it holds
// tof_chain_controller::chain_lock(), because the chain control lines and the bus are a
// single shared resource. Manual commissioning takes the same lock for its whole
// session, and additionally must not run while acquisition is live: enumeration drops
// enable lines, which re-addresses parts underneath a reader. tof_acq_stop() quiesces
// the thread and tof_acq_is_idle() is what the commissioning path checks.
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

namespace lexxhard::tof_acq {

// Four cliff L4 plus two grid L7.
constexpr int kMaxSources{6};

enum class model : uint8_t {
    l4_cliff,
    l7_grid,
};

enum class mapping_state : uint8_t {
    not_ready = 0,
    fault,
    proven,
};

// The diagnostic record is shared between models; the POLICY is not. Reusing one struct
// for "where did it fail and with which errno" costs nothing and keeps one triage
// vocabulary. It says nothing about what the failure means.
using op_status = struct tof_cliff_read_status;

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
    int (*start)(void *dev, op_status *st);
    int (*read_cliff_sample)(void *dev, void *scratch, struct tof_cliff_sample *out,
                             op_status *st);
    int (*stop)(void *dev, op_status *st);
};

// The grid ops, every one of which returns -ENOSYS. An explicit stub rather than a null
// pointer or a copy of the cliff ops, so that wiring L7 to the wrong driver is a
// deliberate act rather than an oversight.
const source_ops &l7_stub_ops();

// The cliff ops, bound to the real tof_cliff_sensor functions.
const source_ops &l4_cliff_ops();

struct source_desc {
    model kind{model::l4_cliff};
    uint8_t addr_7bit{0};
    // Opaque to this layer. The mapping owns what a role means; treating it as a number
    // here is what keeps position policy out of the scheduler.
    uint8_t role_id{0};
    void *dev{nullptr};      // VL53L4CX_Object_t* for l4_cliff
    void *scratch{nullptr};  // tof_cliff_scratch* for l4_cliff
    const source_ops *ops{nullptr};
};

// What happened to one source in one cycle. No classification, no reduction, no alarm.
struct source_facts {
    model kind{model::l4_cliff};
    uint8_t addr_7bit{0};
    uint8_t role_id{0};
    bool configured{false};      // a source occupies this slot
    bool started{false};         // start() succeeded during bring-up
    bool sample_produced{false}; // a fresh sample arrived in this cycle

    // Four distinct outcomes, because collapsing them would decide health semantics by
    // accident: a stubbed model is not a broken sensor, and a bad call of our own is not
    // a bus fault. At most one is set, all four are cleared together before each read of
    // a started source, and none of them is cleared for a source that never started.
    bool transport_error{false}; // -EIO and friends: the transfer or the driver failed
    bool protocol_error{false};  // -EPROTO: the device's own metadata was impossible
    bool unsupported{false};     // -ENOSYS: this model has no implementation yet
    bool usage_error{false};     // -EINVAL: this firmware called it wrongly

    bool rearm_failed{false};    // this sample arrived but the next one will not
    op_status status{};
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
    void (*on_cycle)(const cycle_facts &facts);
    // Raw, unclassified, unreduced. index is the source index in the descriptor table.
    void (*on_cliff_sample)(int index, const source_facts &facts,
                            const struct tof_cliff_sample &sample);
    // Sent from startup, on its own timer, never from the acquisition path.
    void (*on_cliff_health)(uint32_t snapshot, mapping_state state);
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

// Bring-up: open, configure and start every source, in table order, under the lock.
// Failures are recorded per source and do not stop the others - one dead cliff sensor
// must not prevent the other three from ranging.
int bring_up();

// One cycle: read every started source once, sequentially, under the lock. Never waits
// on a source; a source with nothing ready simply has sample_produced false.
void run_cycle();

void stop();

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

// True only when a role measurement may be published at all.
bool publication_allowed();

}  // namespace lexxhard::tof_acq

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
