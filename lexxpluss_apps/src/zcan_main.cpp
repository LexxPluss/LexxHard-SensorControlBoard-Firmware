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

#include <zephyr/logging/log.h>
#include "zcan_actuator.hpp"
#include "zcan_bmu.hpp"
#include "zcan_board.hpp"
#include "zcan_imu.hpp"
#include "zcan_led.hpp"
#include "zcan_pgv.hpp"
#include "zcan_uss.hpp"
#include "zcan_main.hpp"

namespace lexxhard::zcan_main {

LOG_MODULE_REGISTER(zcan_main);

class {
public:
    int init() {
        // CAN baudrate setting
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)) {
            LOG_INF("CAN_2 is not ready");
            return -1;
        }

        can_set_bitrate(dev, 1000000);
        can_set_mode(dev, CAN_MODE_NORMAL);
        can_start(dev);

        bmu.init();
        // board.init();
        act.init();
        imu.init();
        led.init();
        pgv.init();
        uss.init();
        return 0;
    }
    void run() {
        while (true) {
            bmu.poll();
            // board.poll();
            act.poll();
            imu.poll();
            led.poll();
            pgv.poll();
            uss.poll();
            k_usleep(1);
        }
    }
private:
    const device *dev{nullptr};
    lexxhard::zcan_bmu::zcan_bmu bmu;
    // lexxhard::zcan_board::zcan_board board;
    // lexxhard::zcan_dfu::zcan_dfu dfu;
    lexxhard::zcan_actuator::zcan_actuator act;
    lexxhard::zcan_imu::zcan_imu imu;
    lexxhard::zcan_led::zcan_led led;
    lexxhard::zcan_pgv::zcan_pgv pgv;
    lexxhard::zcan_uss::zcan_uss uss;
} impl;

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;

}

// vim: set expandtab shiftwidth=4:
