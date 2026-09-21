/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <stdint.h>

#include "tof_commissioning.hpp"

/*
 * WHICH BUS SPEED, AND WHETHER THE CONTROLLER ACTUALLY GAVE IT.
 *
 * The transaction knows two speeds by ROLE -- the one the enable-chain walk is reliable at, and the
 * one the acquisition schedule is read at -- and deliberately not their frequencies. The frequencies
 * are configuration, and this is where configuration becomes a Zephyr speed code.
 *
 * IT IS ITS OWN TRANSLATION UNIT BECAUSE tof_chain_controller.cpp CANNOT BE HOST TESTED. That file
 * needs the shell, the devicetree and a real I2C controller, so a decision made inside it is a
 * decision no suite can reach -- which is exactly how the commissioning transaction shipped
 * unconfigured on 2026-09-21. Every decision about speed lives here; what is left at the call site
 * is i2c_configure(), hardware-specific read-back, and passing the speed/drive state to
 * verify_applied().
 *
 * THERE IS NO FALLBACK. A controller that cannot do the configured product speed must refuse, not
 * quietly run at 400 kHz: a mapping proven at one speed and read at another is the failure the
 * product-speed re-check exists to prevent, and a silent fallback would reintroduce it while every
 * counter said the run had succeeded.
 */
namespace lexxhard::tof_i2c_speed {

/* The Zephyr I2C_SPEED_* code, or `refused`. Returned rather than applied, so the mapping can be
 * checked without an I2C controller. */
struct resolution {
    uint32_t code{0};
    /* STM32 Fast-mode Plus is two settings, not one: TIMINGR selects the clock and SYSCFG enables
     * the stronger I2C2 drive capability. The latter must also be removed when returning to the
     * proof speed, so it is part of the resolved request rather than an implied side effect. */
    bool fast_mode_plus{false};
    /* 0, or the errno the caller must return. Non-zero means `code` is meaningless. */
    int rc{0};

    bool ok() const { return rc == 0; }
};

/* The frequencies this harness has a speed code for. Anything else is refused rather than rounded:
 * a number nobody validated on this chain must not become a bus speed by being close to one. */
resolution resolve_hz(uint32_t hz);

/* The frequency for a role, from the two configured values, then resolved. The proof speed is
 * configuration too -- 100 kHz is what the enable chain was characterised at, and it is passed in
 * rather than assumed so that a board which needs another one is a configuration change and not a
 * patch. */
resolution resolve_role(tof_commissioning::bus_speed role, uint32_t proof_hz, uint32_t product_hz);

/* Is the driver's selected speed consistent with the hardware drive state?
 *
 * STM32's i2c_get_config() returns cached driver state, not TIMINGR, so the target glue separately
 * checks the hardware timing register. This pure check still owns the cross-field rule: 1 MHz must
 * have Fast-mode Plus drive enabled, and 100/400 kHz must have it disabled. Mismatch is -ENOTSUP. */
int verify_applied(const resolution &requested, uint32_t applied_code, bool applied_fast_mode_plus);

} // namespace lexxhard::tof_i2c_speed

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
