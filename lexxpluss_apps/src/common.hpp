/*
 * Copyright (c) 2022, LexxPluss Inc.
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

#pragma once
#include <zephyr/drivers/gpio.h>

#define M_PI 3.14159265358979323846

/**
 * @brief Equivalent to GPIO_DT_SPEC_GET(node_id, gpios).
 *
 * @param node_id devicetree node identifier
 * @return static initializer for a struct gpio_dt_spec for the property
 * @see GPIO_DT_SPEC_GET(DT_NODELABEL())
 */
#define GET_GPIO(node_id) \
    GPIO_DT_SPEC_GET(DT_NODELABEL(node_id), gpios)

/**
 * @brief Equivalent to GPIO_DT_SPEC_GET(node_id, gpios).
 *
 * @param node_id devicetree node identifier
 * @return static initializer for a struct gpio_dt_spec for the property
 * @see DEVICE_DT_GET(DT_NODELABEL())
 */
#define GET_DEV(node_id) \
    DEVICE_DT_GET(DT_NODELABEL(node_id))
