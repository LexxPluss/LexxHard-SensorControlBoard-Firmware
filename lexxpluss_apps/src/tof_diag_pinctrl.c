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

#if !DT_NODE_EXISTS(DT_PATH(tof_diag_pinctrl))
#error "TOF_I2C_DIAG requires overlays/tof_i2c_diag.overlay (node /tof_diag_pinctrl missing)"
#endif

PINCTRL_DT_DEFINE(DT_PATH(tof_diag_pinctrl));

/* Reapplies the I2C2 alternate function to PF0/PF1 after a bit-bang session. */
int tof_diag_pinctrl_restore(void)
{
    return pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_PATH(tof_diag_pinctrl)),
                               PINCTRL_STATE_DEFAULT);
}
