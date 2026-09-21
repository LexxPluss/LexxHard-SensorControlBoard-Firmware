/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commission_wiring.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

#include <zephyr/sys/atomic.h>

namespace lexxhard::tof_commission_wiring {

namespace {

/* Zephyr's atomic, not std::atomic: this TU is linked by a native_sim suite, whose minimal C++
 * library has no <atomic>. */
atomic_t configure_status_{ATOMIC_INIT(-EAGAIN)};

} // namespace

report boot(const tof_cliff_runtime::config &cfg, const inputs &in)
{
    report r{};

    r.bootstrap_rc = tof_cliff_runtime::bootstrap(cfg);

    /* -EALREADY is accepted only when the runtime says it is actually ready. Host tests use this
     * path after exercising bootstrap itself; on the board boot() is called once and gets zero.
     * Every other bootstrap failure stops here. spec() is statically initialised with six positions,
     * so handing it to commissioning::init() after a failure would otherwise succeed while the
     * authority/publisher/acquisition stack was incomplete. */
    if (r.bootstrap_rc != 0 &&
        (r.bootstrap_rc != -EALREADY || !tof_cliff_runtime::ready())) {
        r.configure_rc = r.bootstrap_rc;
        atomic_set(&configure_status_, r.configure_rc);
        return r;
    }

    /* Refused here rather than handed to init() as a config full of null pointers, so an image wired
     * with a piece missing says which contract it broke. init() would also refuse it, with -EINVAL
     * for any of five different reasons. */
    if (in.chain == nullptr || in.ops == nullptr || in.set_bus_speed == nullptr ||
        in.quiesce == nullptr) {
        r.configure_rc = -EINVAL;
        atomic_set(&configure_status_, r.configure_rc);
        return r;
    }

    tof_commissioning::config tc{};
    tc.chain = in.chain;
    tc.ops = in.ops;
    /* THE spec, not a copy of it. The authority compares every proof against this same object; a
     * copy here would mean commissioning walked one chain description while the authority checked
     * the result against another. */
    tc.spec = &tof_cliff_runtime::spec();
    tc.quiesce = in.quiesce;
    tc.set_bus_speed = in.set_bus_speed;

    r.configure_rc = tof_commissioning::init(tc);
    atomic_set(&configure_status_, r.configure_rc);
    return r;
}

int configure_status()
{
    return static_cast<int>(atomic_get(&configure_status_));
}

} // namespace lexxhard::tof_commission_wiring

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
