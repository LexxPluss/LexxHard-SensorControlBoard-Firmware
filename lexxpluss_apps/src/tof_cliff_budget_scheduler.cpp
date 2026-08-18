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
#include "tof_chain_spec.hpp"
#include "tof_cliff_can.hpp"
#include "tof_cliff_publisher.hpp"
#include "tof_mapping_authority.hpp"

#include <zephyr/kernel.h>

namespace {

namespace acq = lexxhard::tof_acq;
namespace au = lexxhard::tof_authority;
namespace can = lexxhard::tof_cliff_can;
namespace pub = lexxhard::tof_cliff_pub;

/* The chain the authority compares a proof against. Static because the authority keeps the
 * pointer: a stack copy would dangle the moment this function returned. */
lexxhard::tof_enum::chain_spec runtime_spec{lexxhard::tof_chain::dasher_spec()};

/* The sinks are now the real publisher's, not local stubs. That is the point of this build
 * point: the whole path -- acquisition, publisher, packer, CAN glue -- has to be reachable
 * for the signed image to measure what it actually costs. Local stubs measured the
 * acquisition layer and nothing downstream of it. */
volatile uint32_t sink_counters;



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
    cfg.hooks.on_cycle = pub::on_cycle_complete;
    cfg.hooks.on_cliff_sample = pub::on_cliff_sample;
    cfg.hooks.on_cliff_health = pub::on_cliff_health;
    cfg.mapping_state_provider = au::state_provider;
    cfg.now_ms = now_ms;

    /* The authority before the publisher: production_authorisation() reads its epoch, and an
     * uninitialised authority would report one that no proof issued. It reports UNKNOWN until
     * a proof commits, and nothing in this probe commits one -- there is no path to PROVEN
     * from here, which is the point. */
    au::config acfg{};
    acfg.runtime_spec = &runtime_spec;
    acfg.begin_epoch = acq::begin_epoch;
    acfg.acquisition_idle = acq::is_idle;
    if (const int arc{au::init(acfg)}; arc != 0)
        return arc;

    /* The glue is allowed to fail here: on a board where can2 is not ready the measurement
     * still has to include the code, and the publisher will simply count send failures. */
    (void)can::init();

    pub::config pcfg{};
    pcfg.sink = can::sink();
    pcfg.sources = descs;
    pcfg.source_count = acq::kMaxSources;
    /* The production wiring, not a probe-local one. A permissive gate here would make the
     * measurement cover a path production cannot take. */
    pcfg.authorise = can::production_authorisation;
    const int prc{pub::init(pcfg)};

    if (prc != 0)
        return prc;

    const int rc{acq::init(cfg)};

    if (rc != 0)
        return rc;
    (void)acq::bring_up();
    acq::run_cycle();
    acq::stop();

    /* Read the counters through a volatile so nothing above can be collected. */
    pub::counters c{};
    pub::copy_counters(c);
    sink_counters = c.suppressed_not_proven + c.health_sent + c.send_failed_measurement;
    return 0;
}

#endif
