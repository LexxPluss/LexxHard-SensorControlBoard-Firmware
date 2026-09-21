/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_cliff_runtime.hpp"
#include "tof_commissioning.hpp"
#include "tof_enumerator.hpp"

/*
 * THE CLIFF SUBSYSTEM'S BOOT ORDER, IN A TRANSLATION UNIT A HOST SUITE CAN LINK.
 *
 * It exists because of a defect this file's absence caused. The commissioning transaction was
 * configured inside the `tof cliff prove` shell command, so it was ready only on the path that goes
 * through a keyboard. The automatic downlink calls tof_commissioning::prove() directly: on dasher2
 * on 2026-09-21, image 3.6.0-140-g741db238 announced a session on 0x219, accepted the host's 0x218
 * request, and answered `misconfigured / not_started` within three frames, having touched neither
 * I2C nor the enable chain. Two test suites covered the protocol across that seam and neither could
 * see it, because both replaced the real prove() with a fake that succeeds -- so what was never
 * tested was the one thing that was wrong: whether a production boot configures the real
 * transaction at all.
 *
 * Putting the sequence here rather than in tof_chain_controller.cpp is the whole point. That file
 * needs the shell, the devicetree and a real I2C controller, so no host test links it and a missing
 * call inside it is invisible. What remains untestable is one line -- the controller calling boot()
 * -- instead of the wiring itself.
 */
namespace lexxhard::tof_commission_wiring {

/* The pieces only the Zephyr image can supply. Passed in rather than reached for, so the test drives
 * the same sequence with its own chain fake. */
struct inputs {
    k_mutex *chain{nullptr};
    tof_enum::chain_ops *ops{nullptr};
    int (*set_bus_speed)(tof_commissioning::bus_speed){nullptr};
    /* Stops acquisition and does not return until it has. Production passes tof_acq::try_stop. */
    int (*quiesce)(){nullptr};
};

struct report {
    int bootstrap_rc{-EAGAIN};
    /* 0 once tof_commissioning::init() has accepted the configuration. Anything else means BOTH
     * entry points refuse: the shell says which rc, the downlink answers `misconfigured`. */
    int configure_rc{-EAGAIN};
};

/* Bootstrap the cliff runtime, then configure the commissioning transaction against the spec that
 * bootstrap filled in. The order is load-bearing rather than tidy: init() refuses a spec with no
 * positions.
 *
 * A failed bootstrap is a hard stop. The runtime spec has static storage and already contains the
 * product positions before bootstrap, so its non-zero size is NOT evidence that the authority,
 * publisher and acquisition layers came up. Configuring against that partial runtime would turn a
 * boot failure into a transaction that looked ready until its first request.
 */
report boot(const tof_cliff_runtime::config &cfg, const inputs &in);

/* The last configure_rc, for callers that only need to know whether to refuse. -EAGAIN before
 * boot() has run: an image that never reached the call site must not read as configured. */
int configure_status();

} // namespace lexxhard::tof_commission_wiring

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
