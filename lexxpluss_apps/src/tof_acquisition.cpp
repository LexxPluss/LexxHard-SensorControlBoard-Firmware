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
bool running_{false};
bool in_cycle_{false};
cycle_facts facts_;
atomic_t snapshot_{ATOMIC_INIT(0)};
k_timer health_timer_;
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
int l7_start(void *, op_status *st)
{
    memset(st, 0, sizeof(*st));
    return -ENOSYS;
}
int l7_read_cliff_sample(void *, void *, struct tof_cliff_sample *out, op_status *st)
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

// Records an operation's outcome as a neutral fact. The only interpretation performed
// here is the mechanical one: which shape of failure occurred. The four are kept apart
// because folding them together would decide health semantics by accident - a stubbed
// model would arrive at the cliff health frame as a broken sensor, and a bug in our own
// call would arrive as a bus fault.
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
    f.rearm_failed = st.rearm_failed;
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
    const mapping_state reported{cfg_.mapping_state_provider != nullptr
                                     ? cfg_.mapping_state_provider()
                                     : mapping_state::not_ready};

    if (reported == mapping_state::proven) {
        // Two boards' worth of enable chain cannot be enumerated end to end yet, so a
        // proven mapping is not something this firmware is entitled to claim. Reporting
        // NOT_READY keeps the consumer's own fail-safe path in charge.
        //
        // Unconditional, with no build flag to lift it. A conditional safety bypass is
        // one careless -D away from shipping and would not show up in a diff of the code
        // it disables; lifting this is an edit here, in its own commit, reviewed against
        // the fixed hardware.
        static bool warned{false};
        if (!warned) {
            warned = true;
            LOG_WRN("mapping reported PROVEN; clamped to NOT_READY until the "
                    "two-board enable chain is fixed");
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
    return !running_ && !in_cycle_;
}

void copy_facts(cycle_facts &out)
{
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    out = facts_;
    k_mutex_unlock(&tof_chain_controller::chain_lock());
}

int init(const config &cfg)
{
    if (cfg.source_count <= 0 || cfg.source_count > kMaxSources)
        return -EINVAL;
    if (cfg.sources == nullptr || cfg.now_ms == nullptr ||
        cfg.mapping_state_provider == nullptr)
        return -EINVAL;
    if (cfg.hooks.on_cycle == nullptr || cfg.hooks.on_cliff_sample == nullptr ||
        cfg.hooks.on_cliff_health == nullptr)
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
        if (d.kind == model::l4_cliff && (d.dev == nullptr || d.scratch == nullptr))
            return -EINVAL;
    }

    cfg_ = cfg;
    configured_ = true;
    running_ = false;
    in_cycle_ = false;

    facts_ = cycle_facts{};
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
    k_work_init(&health_work_, health_work_handler);
    k_timer_init(&health_timer_, health_timer_handler, nullptr);
    k_timer_start(&health_timer_, K_MSEC(cfg.periods.health_period_ms),
                  K_MSEC(cfg.periods.health_period_ms));
    return 0;
}

int bring_up()
{
    if (!configured_)
        return -EINVAL;

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    in_cycle_ = true;

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};
        op_status st{};
        int rc;

        f.started = false;
        f.transport_error = false;
        f.protocol_error = false;

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

        rc = d.ops->start(d.dev, &st);
        if (rc != 0) {
            record(f, rc, st);
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
    if (!running_)
        return;

    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
    in_cycle_ = true;

    ++facts_.cycle_seq;
    facts_.began_ms = now();

    for (int i{0}; i < facts_.source_count; ++i) {
        const source_desc &d{cfg_.sources[i]};
        source_facts &f{facts_.sources[i]};
        struct tof_cliff_sample sample{};
        op_status st{};
        int rc;

        f.sample_produced = false;
        f.transport_error = false;
        f.protocol_error = false;
        f.rearm_failed = false;
        if (!f.started)
            continue;

        rc = d.ops->read_cliff_sample(d.dev, d.scratch, &sample, &st);
        record(f, rc, st);
        f.sample_produced = sample.fresh;

        // The payload goes to the model's own sink, so neither packer ever has to step
        // over the other model's data, and the fail direction stays out of here.
        if (f.sample_produced && d.kind == model::l4_cliff)
            cfg_.hooks.on_cliff_sample(i, f, sample);
    }

    in_cycle_ = false;
    k_mutex_unlock(&tof_chain_controller::chain_lock());

    publish_snapshot();
    cfg_.hooks.on_cycle(facts_);
}

void stop()
{
    if (!configured_)
        return;

    // Quiesce, then release. Commissioning may only enumerate once this returns: the
    // enumeration drops enable lines, which re-addresses parts underneath a reader.
    k_mutex_lock(&tof_chain_controller::chain_lock(), K_FOREVER);
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
    k_timer_stop(&health_timer_);
    k_mutex_unlock(&tof_chain_controller::chain_lock());
    publish_snapshot();
}

}  // namespace lexxhard::tof_acq

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
