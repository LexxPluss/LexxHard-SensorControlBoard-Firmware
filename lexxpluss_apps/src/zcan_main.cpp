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

#include "rosserial_hardware_zephyr.hpp"
#include "zcan_bmu.hpp"
#include "zcan_board.hpp"
#include "zcan_imu.hpp"
#include "zcan_uss.hpp"
#include "zcan_main.hpp"

namespace lexxhard::rosserial {

class {
public:
    int init() {
        nh.getHardware()->set_baudrate(921600);
        nh.initNode(const_cast<char*>("UART_6"));
        // bmu.init(nh);
        board.init(nh);
        imu.init();
        uss.init();
        return 0;
    }
    void run() {
        while (true) {
            nh.spinOnce();
            // bmu.poll();
            board.poll();
            imu.poll();
            uss.poll();
            k_usleep(1);
        }
    }
private:
    ros::NodeHandle nh;
    // ros_bmu bmu;
    ros_board board;
    // ros_dfu dfu;
    zcan_imu imu;
    // ros_interlock interlock;
    // ros_tof tof;
    ros_uss uss;
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
