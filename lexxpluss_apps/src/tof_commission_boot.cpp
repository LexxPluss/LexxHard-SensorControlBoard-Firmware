/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_commission_boot.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

namespace lexxhard::tof_commission_boot {

namespace {

/* Plain, not atomic, and that is a statement about the caller rather than an oversight: this runs
 * from main() before any per-feature thread is started, which is the same single-context rule
 * tof_acq's configured_/active_ rest on. A second caller from another thread would be a different
 * defect than the one this latch is for. */
bool run_{false};

} // namespace

report run(const steps &s)
{
    report r{};

    if (run_) {
        r.already_run = true;
        return r;
    }
    run_ = true;

    if (s.bootstrap_cliff != nullptr) {
        r.cliff_attempted = true;
        r.cliff_rc = s.bootstrap_cliff(s.ctx);
    }

    /* UNCONDITIONALLY, including after a bootstrap that failed. See the header: a board that cannot
     * be addressed is worse than one that answers with a failed transaction. */
    if (s.start_downlink != nullptr) {
        r.downlink_attempted = true;
        r.downlink_rc = s.start_downlink(s.ctx);
    }

    return r;
}

void reset_for_test()
{
    run_ = false;
}

} // namespace lexxhard::tof_commission_boot

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
