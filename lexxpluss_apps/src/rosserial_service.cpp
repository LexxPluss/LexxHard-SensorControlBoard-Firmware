// /*
//  * Copyright (c) 2022, LexxPluss Inc.
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
//  * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

// #include "rosserial_hardware_zephyr.hpp"
// #include "rosserial_actuator_service.hpp"
// #include "rosserial_service.hpp"

// namespace lexxhard::rosserial_service {

// class {
// public:
//     int init() {
//         nh.initNode(const_cast<char*>("UART_2"));
//         actuator_service.init(nh);
//         return 0;
//     }
//     void run() {
//         while (true) {
//             nh.spinOnce();
//             k_usleep(1);
//         }
//     }
// private:
//     ros::NodeHandle nh;
//     ros_actuator_service actuator_service;
// } impl;

// void init()
// {
//     impl.init();
// }

// void run(void *p1, void *p2, void *p3)
// {
//     impl.run();
// }

// k_thread thread;

// }

// // vim: set expandtab shiftwidth=4:
