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

#include <zephyr.h>
#include <logging/log.h>
#include <cstdio>
#include <cmath>
#include <queue>
#include "common.hpp"
#include "runaway_detector.hpp"

namespace {

LOG_MODULE_REGISTER(runaway_detector);

class yaw_checker {
public:
    void new_topic(float vz, uint32_t current_cycle) {
        bool detected{yaw_error_detected(vz, current_cycle)};
        if (!last_detected && detected) {
            snprintf(log_buffer, sizeof log_buffer, "Emergency! ACC:%4.2f VEL:%4.2f DELTA:%4.2f\n", avg_yaw_accel, avg_yaw_velocity, sum_yaw_delta_theta);
            LOG_ERR("%s", log_strdup(log_buffer));
            // @@ do something. (emergency stop, etc.)
        }
        last_detected = detected;
    }
private:
    bool yaw_error_detected(float vz, uint32_t current_cycle) {
        if (!init_done)
            init();
        delta_theta_calculator(vz, current_cycle);
        yaw_accel_calculator(vz, current_cycle);
        yaw_velocity_calculator(vz, current_cycle);
        if (sum_yaw_delta_theta > YAW_DELTA_THETA_LIMIT)
            return true;
        else if ((avg_yaw_accel > 1 && avg_yaw_velocity > 1) || (avg_yaw_accel < -1 && avg_yaw_velocity < -1))//In other words, "if speeding up "... values are compared to 1 in order to ignore small random inputs
            return fabsf(avg_yaw_accel) > YAW_ACCEL_LIMIT;
        else
            return false;
    }
    void init() {
        for (int i{0}; i < SIZE_OF_TOPICS_QUEUE; ++i)
            topics.push(topic{.vz{0.0f}, .cycle{0U}});
        for (int i{0}; i < SIZE_OF_YAW_ACCEL_QUEUE; ++i)
            yaw_accel.push(0.0f);
        for (int i{0}; i < SIZE_OF_YAW_VELOCITY_QUEUE; ++i)
            yaw_velocity.push(0.0f);
        for (int i{0}; i < SIZE_OF_YAW_DELTA_THETA_QUEUE; ++i)
            yaw_delta_theta.push(0.0f);
        init_done = true;
    }
    void yaw_accel_calculator(float vz, uint32_t current_cycle) {//here, can calculate whether the robot is speeding up (rotation wise) using both velocity and acceleration
        topics.pop();
        topics.push(topic{.vz{vz}, .cycle{current_cycle}});
        topic temp_a{topics.front()}, temp_b{topics.back()};
        float dt {static_cast<float>(k_cyc_to_ms_near32(temp_b.cycle - temp_a.cycle)) / 1000.0f};
        avg_yaw_accel -= yaw_accel.front();
        yaw_accel.pop();
        yaw_accel.push(((temp_b.vz - temp_a.vz) / dt) / SIZE_OF_YAW_ACCEL_QUEUE);//1000.f to convert ms to sec
        avg_yaw_accel += yaw_accel.back();
    }
    void yaw_velocity_calculator(float vz, uint32_t current_cycle) {
        avg_yaw_velocity -= yaw_velocity.front();
        yaw_velocity.pop();
        yaw_velocity.push(vz / SIZE_OF_YAW_VELOCITY_QUEUE);
        avg_yaw_velocity += yaw_velocity.back();
    }
    void delta_theta_calculator(float vz, uint32_t current_cycle) {
        float dt{static_cast<float>(k_cyc_to_ms_near32(current_cycle - prev_cycle)) / 1000.0f};
        sum_yaw_delta_theta -= yaw_delta_theta.front();
        yaw_delta_theta.pop();
        yaw_delta_theta.push(fabsf(vz * dt));
        sum_yaw_delta_theta += yaw_delta_theta.back();
        prev_cycle = current_cycle;
    }
    struct topic {
        float vz;
        uint32_t cycle;
    };
    std::queue<topic> topics;
    std::queue<float> yaw_accel;
    std::queue<float> yaw_velocity;
    std::queue<float> yaw_delta_theta;
    bool init_done{false};
    uint32_t prev_cycle{0U};
    float avg_yaw_accel{0.0f};
    float avg_yaw_velocity{0.0f};
    float sum_yaw_delta_theta{0.0f};
    char log_buffer[256]{0};
    bool last_detected{false};
    static constexpr float YAW_ACCEL_LIMIT{1.5f * M_PI};
    static constexpr float YAW_DELTA_THETA_LIMIT{2.5f * M_PI};
    static constexpr uint8_t SIZE_OF_TOPICS_QUEUE{10U};//average of ~200ms at 50Hz
    static constexpr uint8_t SIZE_OF_YAW_ACCEL_QUEUE{50U};//average of ~1000ms at 50Hz
    static constexpr uint8_t SIZE_OF_YAW_VELOCITY_QUEUE{50U};//average of ~1000ms at 50Hz
    static constexpr uint8_t SIZE_OF_YAW_DELTA_THETA_QUEUE{125U};//average of ~2500ms at 50Hz
};

}

namespace lexxhard::runaway_detector {

class {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        return 0;
    }
    void run() {
        while (true) {
            if (msg message; k_msgq_get(&msgq, &message, K_MSEC(100)) == 0) {
                uint32_t current_cycle{k_cycle_get_32()};
                yaw.new_topic(message.gyro[2], current_cycle);
            }
        }
    }
private:
    yaw_checker yaw;
    char __aligned(4) msgq_buffer[8 * sizeof (msg)]{0};
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
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
