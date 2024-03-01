/*
 * Copyright (c) 2024, LexxPluss Inc.
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

#include <zephyr/kernel.h>
#include <algorithm>
#include "led_test_v7.hpp"

namespace {

constexpr led_rgb black{0, 0, 0}, white{32, 32, 32};

}

namespace lexxhard {

int led_test_v7::init()
{
    dev = DEVICE_DT_GET(DT_NODELABEL(led_strip2));
    if (!device_is_ready(dev))
        return -1;
    update(black);
    prev = k_cycle_get_32();
    return 0;
}

int led_test_v7::poll()
{
    if (!device_is_ready(dev))
        return -1;
    if (auto now{k_cycle_get_32()}; k_cyc_to_ms_near32(now - prev) > 1'000) {
        prev = now;
        update(pixeldata[0].r == 0 ? white : black);
    }
    return 0;
}

void led_test_v7::update(const led_rgb &color)
{
    std::fill(pixeldata, pixeldata + PIXELS, color);
    led_strip_update_rgb(dev, pixeldata, PIXELS);
}

}

// vim: set expandtab shiftwidth=4:
