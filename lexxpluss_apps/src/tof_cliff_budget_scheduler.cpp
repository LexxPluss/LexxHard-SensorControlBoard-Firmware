/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The B6 build point's bridge from the C probe into the C++ subsystem. Not production code: it
 * exists so that one build flag makes the whole path reachable and the signed image can be measured
 * with it in.
 *
 * It used to wire the subsystem up itself -- its own chain_spec, its own descriptor table, its own
 * calls to authority, CAN, publisher and acquisition init. That made the measured image a DIFFERENT
 * machine from the product image: a second tof_authority::init() clears the installed mapping and
 * any open attempt, the spec it compared proofs against was a different object from the shell's, and
 * its descriptors used addresses the chain spec never assigns. A budget build is worth nothing if it
 * is not the same wiring, so this file now measures the production bootstrap and adds nothing.
 */

#if defined(ENABLE_TOF_CHAIN) && defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 6

#include "tof_acquisition.hpp"
#include "tof_cliff_publisher.hpp"
#include "tof_cliff_runtime.hpp"

#include <zephyr/kernel.h>

namespace {

namespace acq = lexxhard::tof_acq;
namespace pub = lexxhard::tof_cliff_pub;
namespace rt = lexxhard::tof_cliff_runtime;

/* Read through a volatile so nothing on the path can be proven unused and collected. */
volatile uint32_t sink_counters;

}  // namespace

extern "C" int tof_cliff_budget_walk_scheduler(void)
{
    /* Does NOT bootstrap. The chain controller has already done that, from the one call site
     * production uses, after configuring the control lines -- and this file calling bootstrap() as
     * well is precisely the defect that made the B6 image a different machine: two calls, the second
     * refused with -EALREADY and logged as a failure, and a walk that ran before the pins were
     * configured because its SYS_INIT fired before main().
     *
     * Refusing when the subsystem is not ready rather than bootstrapping it: if this is ever wired
     * before the bootstrap again, the measurement fails loudly instead of quietly wiring a second
     * time. */
    if (!rt::ready())
        return -EPERM;

    /* One cycle, driven directly, with no mapping applied and therefore no role keys. That is the
     * honest measurement of what this image can do: every measurement frame is suppressed by the
     * publication gate, and what gets measured is the code being reachable rather than a chain being
     * read. bring_up() is called here rather than through rt::start_acquisition(), which refuses
     * without a proven mapping -- and refusing is correct, so the probe steps around it deliberately
     * instead of weakening the gate. */
    (void)acq::bring_up();
    acq::run_cycle();
    acq::stop();

    pub::counters c{};
    pub::copy_counters(c);
    sink_counters = c.suppressed_not_proven + c.health_sent + c.send_failed_measurement;
    return 0;
}

#endif
