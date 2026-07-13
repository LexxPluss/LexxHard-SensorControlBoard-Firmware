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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "shutter_limit_switch.hpp"
#include "common.hpp"

namespace lexxhard::shutter_limit_switch {

LOG_MODULE_REGISTER(shutter_limit_switch);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

// ISR trampolines must be free functions to hand to gpio_init_callback();
// forward-declared here so shutter_limit_switch_impl::init() can take their
// address before their (later) definition in this file.
static void on_open_edge(const device *dev, gpio_callback *cb, uint32_t pins);
static void on_closed_edge(const device *dev, gpio_callback *cb, uint32_t pins);

class shutter_limit_switch_impl {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        if (!gpio_is_ready_dt(&open_dev) || !gpio_is_ready_dt(&closed_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed");
            return -1;
        }
        gpio_pin_configure_dt(&open_dev, GPIO_INPUT);
        gpio_pin_configure_dt(&closed_dev, GPIO_INPUT);
        gpio_init_callback(&open_cb, on_open_edge, BIT(open_dev.pin));
        gpio_init_callback(&closed_cb, on_closed_edge, BIT(closed_dev.pin));
        gpio_add_callback(open_dev.port, &open_cb);
        gpio_add_callback(closed_dev.port, &closed_cb);
        start_time = k_uptime_get();
        return 0;
    }

    // Called once per iteration of the caller's own loop (currently
    // actuator_controller's ~10ms cycle) -- no sleep/loop of its own here.
    void poll() {
        auto const elapsed{static_cast<uint32_t>(k_uptime_get() - start_time)};
        if (!interrupts_enabled) {
            if (!shutter_limit_detector::is_power_on_masked(elapsed)) {
                // Mask window elapsed: enable EXTI, then read the level
                // directly once to seed the state (DESIGN doc "Post-mask
                // state init"), independent of any edge having fired.
                gpio_pin_interrupt_configure_dt(&open_dev, GPIO_INT_EDGE_BOTH);
                gpio_pin_interrupt_configure_dt(&closed_dev, GPIO_INT_EDGE_BOTH);
                interrupts_enabled = true;
                detector.on_edge_isr();
                detector.poll(read(open_dev), read(closed_dev));
            }
        } else {
            detector.poll(read(open_dev), read(closed_dev));
        }
        msg const m{.state = detector.get_state()};
        while (k_msgq_put(&msgq, &m, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq);
    }

    void on_edge_isr() { detector.on_edge_isr(); }

private:
    static bool read(const gpio_dt_spec &dev) { return gpio_pin_get_dt(&dev) > 0; }

    gpio_dt_spec open_dev = GET_GPIO(shutter_limit_open);
    gpio_dt_spec closed_dev = GET_GPIO(shutter_limit_closed);
    gpio_callback open_cb{};
    gpio_callback closed_cb{};
    shutter_limit_detector::detector detector;
    bool interrupts_enabled{false};
    int64_t start_time{0};
} impl;

// Kept minimal per DESIGN doc: no GPIO reads or logging in ISR context, only
// flag the detector so the poll loop re-reads the level directly.
void on_open_edge(const device *dev, gpio_callback *cb, uint32_t pins)
{
    impl.on_edge_isr();
}

void on_closed_edge(const device *dev, gpio_callback *cb, uint32_t pins)
{
    impl.on_edge_isr();
}

void init()
{
    impl.init();
}

void poll()
{
    impl.poll();
}

k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
