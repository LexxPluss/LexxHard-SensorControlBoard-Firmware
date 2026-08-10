/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * C, not C++: PINCTRL_DT_DEFINE expands to designated initializers in an
 * order C++ rejects, so the pinctrl handle lives in this translation unit
 * and tof_diag.cpp calls the restore function through the C ABI.
 *
 * Compiled only by the TOF_I2C_DIAG build (see lexxpluss_apps/CMakeLists.txt);
 * the node comes from overlays/tof_i2c_diag.overlay.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <stm32_ll_gpio.h>

#if !DT_NODE_EXISTS(DT_PATH(tof_diag_pinctrl))
#error "TOF_I2C_DIAG requires overlays/tof_i2c_diag.overlay (node /tof_diag_pinctrl missing)"
#endif
#if !DT_NODE_EXISTS(DT_PATH(tof_diag_spare_pinctrl))
#error "TOF_I2C_DIAG requires overlays/tof_i2c_diag.overlay (node /tof_diag_spare_pinctrl missing)"
#endif

PINCTRL_DT_DEFINE(DT_PATH(tof_diag_pinctrl));
PINCTRL_DT_DEFINE(DT_PATH(tof_diag_spare_pinctrl));

/* Reapplies the I2C2 alternate function to PF0/PF1 after a bit-bang session. */
int tof_diag_pinctrl_restore(void)
{
    return pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_PATH(tof_diag_pinctrl)),
                               PINCTRL_STATE_DEFAULT);
}

/*
 * SparePinGPIO1..4 (PI15/PJ1/PJ2/PJ3): raised to very-high-speed by the
 * pinctrl state above, but gpio_pin_configure_dt() (called by `lpn use`)
 * rewrites OSPEEDR back to reset/low speed as a side effect on every call
 * (the STM32 driver's flags-to-pincfg translation never sets a speed bit,
 * see gpio_stm32_flags_to_conf()/gpio_stm32_configure_raw()). This must be
 * called again immediately after every gpio_pin_configure_dt() on these
 * pins, not just once at boot.
 */
int tof_diag_spare_gpio_speed_apply(void)
{
    return pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_PATH(tof_diag_spare_pinctrl)),
                               PINCTRL_STATE_DEFAULT);
}

struct tof_diag_spare_gpio_reg {
    GPIO_TypeDef *port;
    uint32_t pin;
};

/* Order matches spare_pins[] in tof_diag.cpp: SparePinGPIO1..4. */
static const struct tof_diag_spare_gpio_reg tof_diag_spare_gpio_regs[4] = {
    {GPIOI, LL_GPIO_PIN_15},
    {GPIOJ, LL_GPIO_PIN_1},
    {GPIOJ, LL_GPIO_PIN_2},
    {GPIOJ, LL_GPIO_PIN_3},
};

/*
 * Reads OSPEEDR back for `tof_diag info`, rather than printing an assumed
 * state: if tof_diag_spare_gpio_speed_apply() were ever skipped or failed,
 * this must show the true (low) speed, not the intended one.
 */
const char *tof_diag_spare_gpio_speed_label(int index)
{
    if (index < 0 || index >= 4)
        return "?";
    uint32_t const speed = LL_GPIO_GetPinSpeed(tof_diag_spare_gpio_regs[index].port,
                                                tof_diag_spare_gpio_regs[index].pin);
    switch (speed) {
    case LL_GPIO_SPEED_FREQ_LOW:       return "low";
    case LL_GPIO_SPEED_FREQ_MEDIUM:    return "medium";
    case LL_GPIO_SPEED_FREQ_HIGH:      return "high";
    case LL_GPIO_SPEED_FREQ_VERY_HIGH: return "very-high";
    default:                          return "?";
    }
}
