/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The B6 build point's bridge from the C probe into the C++ acquisition skeleton. Not
 * production code: it exists so that one build flag makes the scheduler reachable and
 * the signed image can be measured with it in.
 */

#if defined(ENABLE_TOF_CHAIN) && defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 6

#include "tof_acquisition.hpp"

#include <zephyr/kernel.h>

namespace {

namespace acq = lexxhard::tof_acq;

volatile uint32_t sink_cycles;
volatile int16_t sink_mm;
volatile uint32_t sink_health;

void on_cycle(const acq::cycle_facts &f)
{
    sink_cycles = f.cycle_seq;
}

void on_cliff_sample(int, const acq::source_facts &, const struct tof_cliff_sample &s)
{
    sink_mm = s.entries[0].range_mm;
}

void on_cliff_health(uint32_t snapshot, acq::mapping_state)
{
    sink_health = snapshot;
}

acq::mapping_state mapping_provider()
{
    // Never PROVEN from a probe: the clamp would refuse it anyway, and pretending
    // otherwise would make the measurement cover a path production cannot take.
    return acq::mapping_state::not_ready;
}

uint32_t now_ms()
{
    return k_uptime_get_32();
}

acq::source_desc descs[acq::kMaxSources];

}  // namespace

extern "C" int tof_cliff_budget_walk_scheduler(void *objs, void *scratch, int stride)
{
    acq::config cfg{};
    auto *base = static_cast<uint8_t *>(objs);

    for (int i = 0; i < acq::kMaxSources; ++i) {
        const bool cliff{i < 4};

        descs[i].kind = cliff ? acq::model::l4_cliff : acq::model::l7_grid;
        // Inside the 7-bit unicast range the sensor layer enforces.
        descs[i].addr_7bit = static_cast<uint8_t>(0x30 + i);
        descs[i].role_id = static_cast<uint8_t>(i);
        descs[i].dev = cliff ? static_cast<void *>(base + i * stride) : nullptr;
        descs[i].scratch = cliff ? scratch : nullptr;
        descs[i].ops = cliff ? &acq::l4_cliff_ops() : &acq::l7_stub_ops();
    }

    cfg.sources = descs;
    cfg.source_count = acq::kMaxSources;
    // Placeholders for the measurement only, and non-zero because init refuses zero.
    // Both remain unresolved symbols in the wire contract.
    cfg.periods.cycle_period_ms = 50;
    cfg.periods.health_period_ms = 100;
    cfg.hooks.on_cycle = on_cycle;
    cfg.hooks.on_cliff_sample = on_cliff_sample;
    cfg.hooks.on_cliff_health = on_cliff_health;
    cfg.mapping_state_provider = mapping_provider;
    cfg.now_ms = now_ms;

    const int rc{acq::init(cfg)};

    if (rc != 0)
        return rc;
    (void)acq::bring_up();
    acq::run_cycle();
    acq::stop();
    return 0;
}

#endif
