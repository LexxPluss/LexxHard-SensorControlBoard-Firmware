/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * THE CLIFF SUBSYSTEM'S BOOT SEQUENCE, IN A TRANSLATION UNIT A HOST SUITE CAN LINK.
 *
 * It exists because of a defect its absence caused. The commissioning transaction was configured
 * inside the `tof cliff prove` shell command, so it was ready only on the path that goes through a
 * keyboard. The automatic downlink calls tof_commissioning::prove() directly: on dasher2 on
 * 2026-09-21, image 3.6.0-140-g741db238 announced a session on 0x219, accepted the host's 0x218
 * request, and answered `misconfigured / not_started` within three frames, having touched neither
 * I2C nor the enable chain. Two suites covered the protocol across that seam and neither could see
 * it, because both replaced the real prove() with a fake that succeeds -- so what was never tested
 * was the one thing that was wrong: whether a production boot configures the real transaction.
 *
 * Putting the sequence here rather than in tof_chain_controller.cpp is the whole point. That file
 * needs the shell, the devicetree and a real I2C controller, so no host test links it and a missing
 * call inside it is invisible. What remains untestable is one line -- the controller calling boot()
 * -- instead of the wiring itself.
 *
 * THREE STEPS, AND EACH GATE IS DIFFERENT:
 *
 * The transaction is configured only if the bootstrap SUCCEEDED. The runtime spec has static
 * storage and already holds the product positions before bootstrap, so its non-zero size is not
 * evidence that the authority, publisher and acquisition layers came up. Configuring against that
 * partial runtime would turn a boot failure into a transaction that looked ready until its first
 * request.
 *
 * The downlink is started EITHER WAY. A board whose bootstrap failed still has to answer: a host
 * holding a durable pending request from a previous boot retransmits into silence for ever
 * otherwise, and `misconfigured` is a terminal answer that is then the truth about this boot. That
 * is the same symptom as the dasher2 incident and, once the transaction is configured at boot, it
 * means what it says instead of pointing at a hole in the wiring. Refusing to install the filter
 * would turn a recoverable fault into an unreachable board.
 *
 * ONCE PER BOOT, and enforced rather than asserted. The downlink runtime states its own
 * once-per-boot rule as an unlocked precondition, on the grounds that no caller in this firmware can
 * get it wrong; this is that caller, and what it holds is a BUS resource. A second
 * can_add_rx_filter() on the request identifier does not fail -- it delivers every request twice and
 * the board answers twice, which is invisible inside the runtime and silent on the wire.
 *
 * WHAT IS NOT DECIDED HERE: whether the board may entertain a commissioning request at all, and
 * whether it may re-enumerate the chain when it gets one. Both are deployment acts, both are off
 * unless an image says so, and both enter through the binding as the devicetree properties
 * commission-profile-enabled and commission-enumeration-permitted.
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include "tof_cliff_runtime.hpp"
#include "tof_commissioning.hpp"
#include "tof_enumerator.hpp"

namespace lexxhard::tof_commission_wiring {

/* The pieces only the Zephyr image can supply. Passed in rather than reached for, so a test drives
 * the same sequence with its own chain fake. */
struct inputs {
    k_mutex *chain{nullptr};
    tof_enum::chain_ops *ops{nullptr};
    int (*set_bus_speed)(tof_commissioning::bus_speed){nullptr};
    /* Stops acquisition and does not return until it has. Production passes tof_acq::try_stop. */
    int (*quiesce)(){nullptr};
    /* Installs the receive filter, draws the session token and starts the worker. Null in an image
     * built without ENABLE_TOF_AUTO_COMMISSION, which is the ordinary case: the downlink is simply
     * not in that image, and the sequence runs the half it has. */
    int (*start_downlink)(void *ctx){nullptr};
    void *ctx{nullptr};
};

struct report {
    int bootstrap_rc{-EAGAIN};
    /* 0 once tof_commissioning::init() has accepted the configuration. Anything else means BOTH
     * entry points refuse: the shell says which rc, the downlink answers `misconfigured`. */
    int configure_rc{-EAGAIN};
    bool downlink_attempted{false};
    /* Whatever start_downlink returned, carried through unexamined. Deciding what the binding's
     * three outcomes mean is the caller's business and not an ordering question. */
    int downlink_rc{0};
    /* A second call. Nothing was run, and this says so rather than a silently repeated boot. */
    bool already_run{false};
};

report boot(const tof_cliff_runtime::config &cfg, const inputs &in);

/* The last configure_rc, for callers that only need to know whether to refuse. -EAGAIN before
 * boot() has run: an image that never reached the call site must not read as configured. */
int configure_status();

#ifdef CONFIG_ZTEST
/* Clears the once-per-boot latch and the status. For tests only, and compiled out of a product
 * image rather than merely documented as such: there is no production path that unboots a board,
 * and an entry point that clears the latch IS the mistake the latch exists for. */
void reset_for_test();
#endif

} // namespace lexxhard::tof_commission_wiring

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
