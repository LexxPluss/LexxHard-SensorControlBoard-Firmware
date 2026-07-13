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

#pragma once

#include <zephyr/kernel.h>

namespace lexxhard::shutter_limit_switch {

// The raw confirmed switch signals -- exactly
// shutter_limit_detector::detector::get_open_bit()/get_closed_bit() --
// carried straight through to CAN transmission (see zcan_gpio.hpp) without
// re-deriving them from state. Debug/display use (e.g.
// `shutter_limit_switch info`) reads detector::get_state() directly instead
// of going through this msgq, so no decoded state is carried here.
struct msg {
    bool open_bit;
    bool closed_bit;
} __attribute__((aligned(4)));

// No dedicated thread/stack: this module is driven from the caller's own
// poll loop (currently actuator_controller, since the Shutter motor is the
// Center axis and any future stop-on-limit-switch logic will live there
// too). init() still owns GPIO/EXTI setup and must run once at boot.
void init();
void poll();
extern k_msgq msgq;
}

// vim: set expandtab shiftwidth=4:
