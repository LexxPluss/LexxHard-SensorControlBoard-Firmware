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

class uss_fetcher {
public:
    int init(const char *label0, const char *label1) {
        // dev[0] = device_get_binding(label0);
        dev[0] = DEVICE_DT_GET(DT_NODELABEL(label0));
        if (!device_is_ready(dev[0]))
            return -1;
        if (label1 != nullptr) {
            // dev[1] = device_get_binding(label1);
            dev[1] = DEVICE_DT_GET(DT_NODELABEL(label1));
            if (!device_is_ready(dev[1]))
                return -1;
        }
        return 0;
    }
    void get_distance(uint32_t (&distance)[2]) const {
        distance[0] = this->distance[0];
        distance[1] = this->distance[1];
    }
    static void runner(void *p1, void *p2, void *p3) {
        uss_fetcher *self{static_cast<uss_fetcher*>(p1)};
        self->run();
    }
    k_thread thread;
private:
    void run() {
        if (!device_is_ready(dev[0]))
            return;
        while (true) {
            if (sensor_sample_fetch_chan(dev[0], SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev[0], SENSOR_CHAN_DISTANCE, &v);
                int32_t value{v.val1 * 1000 + v.val2 / 1000};
                distance[0] = distance[0] / 4 + value * 3 / 4;
            }
            if (device_is_ready(dev[1])) {
                if (sensor_sample_fetch_chan(dev[1], SENSOR_CHAN_ALL) == 0) {
                    sensor_value v;
                    sensor_channel_get(dev[1], SENSOR_CHAN_DISTANCE, &v);
                    int32_t value{v.val1 * 1000 + v.val2 / 1000};
                    distance[1] = distance[1] / 4 + value * 3 / 4;
                }
            }
            k_msleep(1);
        }
    }
    const device *dev[2]{nullptr, nullptr};
    uint32_t distance[2]{0, 0};
} fetcher[4];

K_THREAD_STACK_DEFINE(fetcher_stack_0, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_1, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_2, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_3, 2048);

#define RUN(x) \
    k_thread_create(&fetcher[x].thread, fetcher_stack_##x, K_THREAD_STACK_SIZEOF(fetcher_stack_##x), \
                    &uss_fetcher::runner, &fetcher[x], nullptr, nullptr, 3, K_FP_REGS, K_NO_WAIT);

int info(const shell *shell, size_t argc, char **argv)
{
    uint32_t front[2], left[2], right[2], back[2];
    fetcher[0].get_distance(front);
    fetcher[1].get_distance(left);
    fetcher[2].get_distance(right);
    fetcher[3].get_distance(back);
    shell_print(shell, "FL:%umm FR:%umm L:%umm R:%umm B:%umm\n",
                front[0], front[1],
                left[0], right[0], back[0]);
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
    fetcher[0].init("MB1604_0", "MB1604_1");
    fetcher[1].init("MB1604_2", nullptr);
    fetcher[2].init("MB1604_3", nullptr);
    fetcher[3].init("MB1604_4", nullptr);
}

void run(void *p1, void *p2, void *p3)
{
    RUN(0);
    RUN(1);
    RUN(2);
    RUN(3);
    while (true) {
        msg message;
        uint32_t distance[2];
        fetcher[0].get_distance(distance);
        message.front_left = distance[0];
        message.front_right = distance[1];
        fetcher[1].get_distance(distance);
        message.left = distance[0];
        fetcher[2].get_distance(distance);
        message.right = distance[0];
        fetcher[3].get_distance(distance);
        message.back = distance[0];
        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq);
        k_msleep(100);
    }
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
