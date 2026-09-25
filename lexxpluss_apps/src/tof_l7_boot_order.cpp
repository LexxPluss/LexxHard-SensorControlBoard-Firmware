/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_l7_boot_order.hpp. The order in this function is the deliverable.
 */

#include "tof_l7_boot_order.hpp"

namespace lexxhard::tof_l7_boot_order {

report run(const steps &s)
{
    report r{};

    /* Everything down to the pins is best-effort, and the reason each failure only sets a flag is
     * that the alternative is a boot which refuses to bring the chain up because it could not do
     * something optional to it. */
    bool may_recover{true};

    if (s.set_recovery_speed != nullptr) {
        if (s.set_recovery_speed(s.ctx) == 0)
            r.speed_set = true;
        else
            may_recover = false;
    }

    if (may_recover && s.read_back_speed != nullptr) {
        bool matches{false};
        if (s.read_back_speed(s.ctx, &matches) == 0 && matches)
            r.speed_readback_ok = true;
        else
            may_recover = false;
    }

    /* THE POINT OF THE WHOLE FILE: this is before either pin. */
    if (may_recover && s.recover_survivors != nullptr) {
        r.recovery_ran = true;
        r.recovery_rc = s.recover_survivors(s.ctx);
    }

    /* Restored whether or not recovery ran, because what changed the speed was the attempt, not the
     * success, and everything downstream was promised the devicetree's speed. */
    if (r.speed_set && s.restore_product_speed != nullptr)
        r.product_speed_restored = s.restore_product_speed(s.ctx) == 0;

    if (s.configure_data_pin != nullptr) {
        if (const int rc{s.configure_data_pin(s.ctx)}; rc != 0) {
            r.rc = rc;
            r.failed_at = step::configure_data_pin;
            return r;
        }
    }

    if (s.configure_clock_pin != nullptr) {
        if (const int rc{s.configure_clock_pin(s.ctx)}; rc != 0) {
            r.rc = rc;
            r.failed_at = step::configure_clock_pin;
            return r;
        }
    }

    r.pins_configured = true;
    return r;
}

const char *step_name(step v)
{
    switch (v) {
    case step::none:                  return "none";
    case step::set_recovery_speed:    return "set_recovery_speed";
    case step::read_back_speed:       return "read_back_speed";
    case step::recover:               return "recover";
    case step::restore_product_speed: return "restore_product_speed";
    case step::configure_data_pin:    return "configure_data_pin";
    case step::configure_clock_pin:   return "configure_clock_pin";
    }
    return "unknown";
}

}  // namespace lexxhard::tof_l7_boot_order
