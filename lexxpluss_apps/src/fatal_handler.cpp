/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifdef ENABLE_LCS

#include <zephyr/devicetree.h>
#include <zephyr/fatal.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/toolchain.h>
#include <cstdint>
#include "lcs_safe_state.hpp"

namespace {

/* Resolved at build time so that the fatal path needs no driver or kernel call. */
constexpr uintptr_t kDeBase{DT_REG_ADDR(DT_GPIO_CTLR(DT_NODELABEL(pgv_en), gpios))};
constexpr uint32_t kDePin{DT_GPIO_PIN(DT_NODELABEL(pgv_en), gpios)};
constexpr uintptr_t kReBase{DT_REG_ADDR(DT_GPIO_CTLR(DT_NODELABEL(pgv_en_n), gpios))};
constexpr uint32_t kRePin{DT_GPIO_PIN(DT_NODELABEL(pgv_en_n), gpios)};

}

/* The weak default returns, which would abort only the faulting thread and let the system run on. */
void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
    ARG_UNUSED(esf);

    /* Safe state first: one store each, both pins active high so a logical low is a physical low. */
    volatile uint32_t &de{*reinterpret_cast<volatile uint32_t *>(kDeBase + lexxhard::lcs::kGpioBsrrOffset)};
    de = lexxhard::lcs::bsrr_reset_word(kDePin);
    volatile uint32_t &re{*reinterpret_cast<volatile uint32_t *>(kReBase + lexxhard::lcs::kGpioBsrrOffset)};
    re = lexxhard::lcs::bsrr_reset_word(kRePin);

    LOG_PANIC();

    k_fatal_halt(reason);
    CODE_UNREACHABLE;
}

#endif
