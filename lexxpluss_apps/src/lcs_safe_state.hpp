/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Constants of the safe-state store on the fatal error path of the load cell RS-485 driver
 * enable pin.
 */

#pragma once

#include <cstdint>

namespace lexxhard::lcs {

/* GPIOx_BSRR of the STM32F7: bits 0..15 set a pin, bits 16..31 reset it. */
inline constexpr uint32_t kGpioBsrrOffset{0x18};

/* The BSRR word that drives pin low in one store (used on the fatal error path, where no driver call is safe). */
constexpr uint32_t bsrr_reset_word(const uint32_t pin)
{
    return 1U << (16U + pin);
}

}
