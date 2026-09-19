/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_commission_boot.hpp"

#if defined(ENABLE_TOF_AUTO_COMMISSION)

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include "tof_commission_entropy.hpp"

LOG_MODULE_REGISTER(tof_commission_boot);

namespace lexxhard::tof_commission_boot {

namespace bind = tof_commission_bind;

namespace {

/* THE TWO BUILD DECISIONS, and they are deliberately not one. The profile being on says the board
 * will entertain a commissioning request at all; the release condition says it may re-enumerate the
 * chain when it does. A bench image needs both, and separating them means an image can be built that
 * announces a session and refuses every proof -- which is what you want while checking the transport
 * without touching the sensors. */
constexpr bool kPermitEnumerationAtBuild{
#if defined(TOF_AUTO_COMMISSION_PERMIT_ENUMERATION)
    true
#else
    false
#endif
};

constexpr bool kProfileEnabledAtBuild{
#if defined(TOF_AUTO_COMMISSION_PROFILE)
    true
#else
    false
#endif
};

/* Budgets for one boot. Non-zero because a profile that is on with a zero budget is a machine that
 * can never finish, which the sequencer reports as misconfigured rather than as a quiet nothing. */
constexpr uint8_t kMaxProofAttempts{3};
constexpr uint8_t kMaxStartAttempts{3};
constexpr uint32_t kAnnouncePeriodMs{1000};
constexpr uint32_t kPollMs{20};

/* LATCHED AT BOOT. Read once in start(), never written again, and the hook can only read it -- so
 * what the board will do cannot change while it is running. */
bool permit_latched_{false};
bool latched_{false};
report last_{};

bool permitted(void *)
{
    return permit_latched_;
}

} // namespace

report start()
{
    if (latched_) {
        /* Already decided. Re-reading would let a second caller change what the first established,
         * and the worker it created cannot be created twice anyway. */
        return last_;
    }

    permit_latched_ = kPermitEnumerationAtBuild;
    latched_ = true;

    report r{};
    r.permit_enumeration = permit_latched_;
    r.profile_enabled = kProfileEnabledAtBuild;

    /* can2 is brought up by zcan_main and looked up by the cliff bootstrap, both of which have run
     * by the time this is called. Reported rather than assumed: a board with no CAN is refused by
     * the binding, and knowing which of the two was missing is the difference between a wiring
     * problem and a build problem. */
    const struct device *can{DEVICE_DT_GET(DT_NODELABEL(can2))};
    r.can_ready = device_is_ready(can);
    r.entropy_ready = tof_commission_entropy::available();

    bind::config cfg{};
    cfg.profile_enabled = r.profile_enabled;
    cfg.max_proof_attempts = kMaxProofAttempts;
    cfg.max_start_attempts = kMaxStartAttempts;
    cfg.announce_period_ms = kAnnouncePeriodMs;
    cfg.poll_ms = kPollMs;
    cfg.enumeration_permitted = permitted;
    cfg.ctx = nullptr;

    const bind::result br{bind::start(can, cfg)};
    r.state = br.state;
    r.rc = br.rc;
    r.filter_installed = br.filter_installed;
    r.worker_started = br.worker_started;
    last_ = r;

    switch (r.state) {
    case bind::outcome::running:
        LOG_INF("commissioning downlink up: profile %s, enumeration %s, worker %s",
                r.profile_enabled ? "on" : "off", r.permit_enumeration ? "permitted" : "REFUSED",
                r.worker_started ? "running" : "not started");
        break;
    case bind::outcome::answering_only:
        /* The one that is easy to misread as working: the board is on the bus and will answer every
         * request, and it will never commission anything. */
        LOG_ERR("commissioning downlink has NO SESSION (entropy %s): answering no_session only",
                r.entropy_ready ? "ready" : "not ready");
        break;
    case bind::outcome::refused:
        LOG_ERR("commissioning downlink not installed (rc=%d, can %s)", r.rc,
                r.can_ready ? "ready" : "not ready");
        break;
    }
    return r;
}

report last()
{
    return last_;
}

} // namespace lexxhard::tof_commission_boot

#endif // ENABLE_TOF_AUTO_COMMISSION
