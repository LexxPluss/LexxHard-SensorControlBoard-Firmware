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

#include <zephyr.h>
#include <cstdlib>
#include <cstring>

namespace lexxhard::led_controller {

struct msg {
    msg() : pattern(NONE), interrupt_ms(0) {}
    msg(uint32_t pattern, uint32_t interrupt_ms) :
        pattern(pattern), interrupt_ms(interrupt_ms) {}
    msg(const char *str) {
        if      (strcmp(str, "emergency_stop")  == 0) pattern = EMERGENCY_STOP;
        else if (strcmp(str, "amr_mode")        == 0) pattern = AMR_MODE;
        else if (strcmp(str, "agv_mode")        == 0) pattern = AGV_MODE;
        else if (strcmp(str, "mission_pause")   == 0) pattern = MISSION_PAUSE;
        else if (strcmp(str, "path_blocked")    == 0) pattern = PATH_BLOCKED;
        else if (strcmp(str, "manual_drive")    == 0) pattern = MANUAL_DRIVE;
        else if (strcmp(str, "charging")        == 0) pattern = CHARGING;
        else if (strcmp(str, "waiting_for_job") == 0) pattern = WAITING_FOR_JOB;
        else if (strcmp(str, "left_winker")     == 0) pattern = LEFT_WINKER;
        else if (strcmp(str, "right_winker")    == 0) pattern = RIGHT_WINKER;
        else if (strcmp(str, "both_winker")     == 0) pattern = BOTH_WINKER;
        else if (strcmp(str, "move_actuator")   == 0) pattern = MOVE_ACTUATOR;
        else if (strcmp(str, "charge_level")    == 0) pattern = CHARGE_LEVEL;
        else if (strcmp(str, "showtime")        == 0) pattern = SHOWTIME;
        else if (strcmp(str, "lockdown")        == 0) pattern = LOCKDOWN;
        else if (*str == '#') setup_cpm_breath(str);
        else pattern = NONE;
        interrupt_ms = 0;
    }
    void setup_cpm_breath(const char *str) {
        pattern = NONE;
        cpm = 0;
        int r{dec2(&str[1])};
        int g{dec2(&str[3])};
        int b{dec2(&str[5])};
        if (r < 0 || g < 0 || b < 0)
            return;
        rgb[0] = r;
        rgb[1] = g;
        rgb[2] = b;
        if (str[7] != ' ')
            return;
        if (strncmp(&str[8], "breath", 6) == 0 && str[14] == ' ') {
            pattern = RGB_BREATH;
            cpm = atoi(&str[15]);
        } else if (strncmp(&str[8], "blink", 5) == 0 && str[13] == ' ') {
            pattern = RGB_BLINK;
            cpm = atoi(&str[14]);
        } else {
            return;
        }
        if (cpm == 0)
            pattern = RGB;
    }
    int dec2(const char *str) const {
        int h{dec1(str[0])};
        int l{dec1(str[1])};
        if (h < 0 || l < 0)
            return -1;
        return h << 4 | l;
    }
    int dec1(char c) const {
        return (c >= '0' && c <= '9') ? c - '0'
             : (c >= 'a' && c <= 'f') ? c - 'a' + 10
             : (c >= 'A' && c <= 'F') ? c - 'A' + 10
                                      : -1;
    }
    uint32_t pattern{NONE}, interrupt_ms{0}, cpm{0};
    uint8_t rgb[3]{0, 0, 0};
    static constexpr uint32_t NONE{0};
    static constexpr uint32_t EMERGENCY_STOP{1};
    static constexpr uint32_t AMR_MODE{2};
    static constexpr uint32_t AGV_MODE{3};
    static constexpr uint32_t MISSION_PAUSE{4};
    static constexpr uint32_t PATH_BLOCKED{5};
    static constexpr uint32_t MANUAL_DRIVE{6};
    static constexpr uint32_t CHARGING{10};
    static constexpr uint32_t WAITING_FOR_JOB{11};
    static constexpr uint32_t LEFT_WINKER{12};
    static constexpr uint32_t RIGHT_WINKER{13};
    static constexpr uint32_t BOTH_WINKER{14};
    static constexpr uint32_t MOVE_ACTUATOR{15};
    static constexpr uint32_t CHARGE_LEVEL{16};
    static constexpr uint32_t SHOWTIME{10000};
    static constexpr uint32_t LOCKDOWN{10001};
    static constexpr uint32_t RGB{20000};
    static constexpr uint32_t RGB_BLINK{20001};
    static constexpr uint32_t RGB_BREATH{20002};
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
extern k_thread thread;
extern k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
