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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "uss_controller.hpp"

namespace lexxhard::uss_controller {

LOG_MODULE_REGISTER(uss);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

// One entry per fetcher slot: which uss labels it drives, and which message
// field(s) its distance value(s) are packed into (field1 is null when the
// slot only drives one sensor). This table's size is the number of fetchers
// actually needed, so it tracks devicetree status directly: on a board
// where uss4/back is unpopulated (status "disabled"), it simply has no
// entry, rather than an entry that's expected to fail. A message field with
// no entry here (only ever "back" today) keeps whatever run()/info()
// zero-initialize it to.
struct fetcher_cfg_entry { int label0, label1; uint32_t msg::*field0, msg::*field1; };
static constexpr fetcher_cfg_entry fetcher_cfg[] = {
    {0, 1, &msg::front_left, &msg::front_right}, // front: uss0 (left) + uss1 (right)
    {2, -1, &msg::left, nullptr},                // left: uss2
    {3, -1, &msg::right, nullptr},               // right: uss3
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uss4), okay)
    {4, -1, &msg::back, nullptr},                // back: uss4
#endif
};

class uss_fetcher {
public:
    // k_mutex has no "is initialized" query, so lock must never be reachable
    // in an uninitialized state. init() can return early on any of several
    // paths (invalid label, device not ready), so the constructor -- which
    // always runs for this namespace-scope array before main() -- is the
    // only place that can make that guarantee unconditionally.
    uss_fetcher() { k_mutex_init(&lock); }
    int init(int label0, int label1) {
        switch (label0) {
            case 0:
                dev[0] = DEVICE_DT_GET(DT_NODELABEL(uss0));
                break;
            case 1:
                dev[0] = DEVICE_DT_GET(DT_NODELABEL(uss1));
                break;
            case 2:
                dev[0] = DEVICE_DT_GET(DT_NODELABEL(uss2));
                break;
            case 3:
                dev[0] = DEVICE_DT_GET(DT_NODELABEL(uss3));
                break;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uss4), okay)
            case 4:
                dev[0] = DEVICE_DT_GET(DT_NODELABEL(uss4));
                break;
#endif
            default:
                return -1;
        }
        if (!device_is_ready(dev[0])) {
            return -1;
        }

        if (label1 != -1) {
            switch (label1) {
                case 0:
                    dev[1] = DEVICE_DT_GET(DT_NODELABEL(uss0));
                    break;
                case 1:
                    dev[1] = DEVICE_DT_GET(DT_NODELABEL(uss1));
                    break;
                case 2:
                    dev[1] = DEVICE_DT_GET(DT_NODELABEL(uss2));
                    break;
                case 3:
                    dev[1] = DEVICE_DT_GET(DT_NODELABEL(uss3));
                    break;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uss4), okay)
                case 4:
                    dev[1] = DEVICE_DT_GET(DT_NODELABEL(uss4));
                    break;
#endif
                default:
                    return -1;
            }
            if (!device_is_ready(dev[1])) {
                return -1;
            }
        }

        return 0;
    }
    void get_distance(uint32_t (&distance)[2]) const {
        if (k_mutex_lock(&this->lock, K_MSEC(100)) == 0) {
            distance[0] = this->distance[0];
            distance[1] = this->distance[1];
            k_mutex_unlock(&this->lock);
        } else {
            LOG_WRN("Failed to acquire mutex in get_distance");
            return;
        }
    }
    void update_once() {
        if (!device_is_ready(dev[0])) {
            return;
        }

        if (sensor_sample_fetch_chan(dev[0], SENSOR_CHAN_ALL) == 0) {
            sensor_value v;
            sensor_channel_get(dev[0], SENSOR_CHAN_DISTANCE, &v);
            int32_t value{v.val1 * 1000 + v.val2 / 1000};

            if (k_mutex_lock(&this->lock, K_MSEC(100)) == 0) {
                distance[0] = distance[0] / 4 + value * 3 / 4;
                k_mutex_unlock(&this->lock);
            } else {
                LOG_WRN("Failed to acquire mutex for store distance[0]");
            }
        }
        if (device_is_ready(dev[1])) {
            if (sensor_sample_fetch_chan(dev[1], SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev[1], SENSOR_CHAN_DISTANCE, &v);
                int32_t value{v.val1 * 1000 + v.val2 / 1000};

                if (k_mutex_lock(&this->lock, K_MSEC(100)) == 0) {
                    distance[1] = distance[1] / 4 + value * 3 / 4;
                    k_mutex_unlock(&this->lock);
                } else {
                    LOG_WRN("Failed to acquire mutex for store distance[1]");
                }

            }
        }
    }
private:
    const device *dev[2]{nullptr, nullptr};
    uint32_t distance[2]{0, 0};
    mutable struct k_mutex lock;
} fetcher[ARRAY_SIZE(fetcher_cfg)];

static uint32_t fetch_delay_ms = 1; // Default delay of 1ms

void fetch_thread(void *, void *, void *)
{
    while (true) {
        for (auto &f : fetcher) {
            f.update_once();
            k_msleep(fetch_delay_ms);
        }
    }
}
K_THREAD_STACK_DEFINE(fetch_stack, 2048);
static k_thread fetch_thr;

int info(const shell *shell, size_t argc, char **argv)
{
    msg message{};
    for (size_t i = 0; i < ARRAY_SIZE(fetcher_cfg); ++i) {
        uint32_t distance[2];
        fetcher[i].get_distance(distance);
        message.*(fetcher_cfg[i].field0) = distance[0];
        if (fetcher_cfg[i].field1) {
            message.*(fetcher_cfg[i].field1) = distance[1];
        }
    }
    shell_print(shell, "FL:%umm FR:%umm L:%umm R:%umm B:%umm\n",
                message.front_left, message.front_right,
                message.left, message.right, message.back);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(info, NULL, "USS information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(uss, &sub, "USS commands", NULL);

void init()
{
    k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
    for (size_t i = 0; i < ARRAY_SIZE(fetcher_cfg); ++i) {
        fetcher[i].init(fetcher_cfg[i].label0, fetcher_cfg[i].label1);
    }
}

void run_fetch()
{
    k_thread_create(&fetch_thr,
                    fetch_stack, K_THREAD_STACK_SIZEOF(fetch_stack),
                    fetch_thread,
                    nullptr, nullptr, nullptr,
                    3, K_FP_REGS, K_NO_WAIT);
}

void run(void *p1, void *p2, void *p3)
{

    run_fetch();
    while (true) {
        msg message{}; // fields with no fetcher (only "back", on boards without uss4) stay 0
        for (size_t i = 0; i < ARRAY_SIZE(fetcher_cfg); ++i) {
            uint32_t distance[2];
            fetcher[i].get_distance(distance);
            message.*(fetcher_cfg[i].field0) = distance[0];
            if (fetcher_cfg[i].field1) {
                message.*(fetcher_cfg[i].field1) = distance[1];
            }
        }
        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0) {
            k_msgq_purge(&msgq);
        }
        k_msleep(100);
    }
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
