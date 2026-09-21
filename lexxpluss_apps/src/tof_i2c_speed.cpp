/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_i2c_speed.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>

#include <zephyr/drivers/i2c.h>

namespace lexxhard::tof_i2c_speed {

resolution resolve_hz(uint32_t hz)
{
    switch (hz) {
    case 100000:
        return {I2C_SPEED_STANDARD, false, 0};
    case 400000:
        return {I2C_SPEED_FAST, false, 0};
    case 1000000:
        /* Fast-mode Plus. Named here rather than left to whoever writes the devicetree, because
         * "1 MHz" and "the code Zephyr wants for 1 MHz" are different facts and only one of them is
         * in the overlay. */
        return {I2C_SPEED_FAST_PLUS, true, 0};
    default:
        return {0, false, -ENOTSUP};
    }
}

resolution resolve_role(tof_commissioning::bus_speed role, uint32_t proof_hz, uint32_t product_hz)
{
    switch (role) {
    case tof_commissioning::bus_speed::proof:
        return resolve_hz(proof_hz);
    case tof_commissioning::bus_speed::product:
        return resolve_hz(product_hz);
    }
    return {0, false, -EINVAL};
}

int verify_applied(const resolution &requested, uint32_t applied_code, bool applied_fast_mode_plus)
{
    if (!requested.ok())
        return requested.rc;
    return requested.code == applied_code &&
                   requested.fast_mode_plus == applied_fast_mode_plus
               ? 0
               : -ENOTSUP;
}

} // namespace lexxhard::tof_i2c_speed

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
