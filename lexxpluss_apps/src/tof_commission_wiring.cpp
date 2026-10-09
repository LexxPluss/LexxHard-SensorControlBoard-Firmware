/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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

/* Plain, not atomic, and that is a statement about the caller rather than an oversight: boot() runs
 * from main() before any per-feature thread starts, which is the same single-context rule
 * tof_acq's configured_/active_ rest on. A second caller from another thread would be a different
 * defect than the one this latch is for. */
bool run_{false};

} // namespace

report boot(const tof_cliff_runtime::config &cfg, const inputs &in)
{
    report r{};

    if (run_) {
        r.already_run = true;
        r.configure_rc = configure_status();
        return r;
    }
    run_ = true;

    r.bootstrap_rc = tof_cliff_runtime::bootstrap(cfg);

    /* -EALREADY is accepted only when the runtime says it is actually ready. Host tests reach this
     * path after exercising bootstrap itself; on the board boot() is called once and gets zero.
     * Every other bootstrap failure skips the configuration -- see the header on why a statically
     * populated spec makes a configure against a partial runtime look like a success. */
    const bool bootstrapped{r.bootstrap_rc == 0 ||
                            (r.bootstrap_rc == -EALREADY && tof_cliff_runtime::ready())};

    if (!bootstrapped) {
        r.configure_rc = r.bootstrap_rc;
    } else if (in.chain == nullptr || in.ops == nullptr || in.set_bus_speed == nullptr ||
               in.quiesce == nullptr) {
        /* Refused here rather than handed to init() as a config full of null pointers, so an image
         * wired with a piece missing says which contract it broke. init() would also refuse it,
         * with -EINVAL for any of five different reasons. */
        r.configure_rc = -EINVAL;
    } else {
        tof_commissioning::config tc{};

        tc.chain = in.chain;
        tc.ops = in.ops;
        /* THE spec, not a copy of it. The authority compares every proof against this same object;
         * a copy here would mean commissioning walked one chain description while the authority
         * checked the result against another. */
        tc.spec = &tof_cliff_runtime::spec();
        tc.quiesce = in.quiesce;
        tc.set_bus_speed = in.set_bus_speed;

        r.configure_rc = tof_commissioning::init(tc);
    }
    atomic_set(&configure_status_, r.configure_rc);

    /* UNCONDITIONALLY, including after a bootstrap or a configuration that failed. See the header:
     * a board that cannot be addressed is worse than one that answers `misconfigured`. */
    if (in.start_downlink != nullptr) {
        r.downlink_attempted = true;
        r.downlink_rc = in.start_downlink(in.ctx);
    }

    return r;
}

int configure_status()
{
    return static_cast<int>(atomic_get(&configure_status_));
}

#ifdef CONFIG_ZTEST
void reset_for_test()
{
    run_ = false;
    atomic_set(&configure_status_, -EAGAIN);
}
#endif

} // namespace lexxhard::tof_commission_wiring

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
