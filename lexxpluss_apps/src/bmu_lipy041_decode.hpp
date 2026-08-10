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

#include <cstdint>

namespace lexxhard::bmu_lipy041 {

struct extreme_pair_u16 {
    uint16_t value;
    uint8_t id;
};

struct extreme_pair_i16 {
    int16_t value;
    uint8_t id;
};

struct msg_0x100 {
    uint8_t fail_status1{0xff};
    uint8_t leader_battery_status{0};
    uint8_t asoc_min{0};
    uint8_t rsoc_min{0};
    uint8_t soh_min{0};
    int16_t max_fet_temp{0};
};

struct msg_0x101 {
    int16_t average_current{0};
    uint16_t max_charging_current{0};
    uint16_t bm_voltage_max{0};
    uint8_t fail_status2{0xff};
};

struct msg_0x103 {
    uint16_t design_capacity{0};
    uint16_t fcc_min{0};
    uint16_t rc_min{0};
    uint8_t fet_status{0};
};

struct msg_0x110 {
    extreme_pair_u16 max_voltage, min_voltage;
};

struct msg_0x111 {
    extreme_pair_i16 max_temp, min_temp;
};

struct msg_0x112 {
    extreme_pair_i16 max_current, min_current;
};

struct msg_0x113 {
    uint8_t fw_ver{0};
    uint8_t data_ver{0};
    uint8_t connected_bm_count{0};
    uint8_t leader_alarm1{0xff};
    uint8_t leader_alarm2{0xff};
    uint8_t fail_status3{0xff};
};

struct msg_0x120 {
    extreme_pair_u16 max_cell_voltage, min_cell_voltage;
};

struct msg_0x130 {
    uint16_t manufacturing{0};
    uint16_t inspection{0};
    uint16_t serial{0};
};

struct msg_0x131 {
    uint32_t accumulated_capacity{0};
};

void decode_0x100(const uint8_t data[8], msg_0x100 &msg);
void decode_0x101(const uint8_t data[8], msg_0x101 &msg);
void decode_0x103(const uint8_t data[8], msg_0x103 &msg);
void decode_0x110(const uint8_t data[8], msg_0x110 &msg);
void decode_0x111(const uint8_t data[8], msg_0x111 &msg);
void decode_0x112(const uint8_t data[8], msg_0x112 &msg);
void decode_0x113(const uint8_t data[8], msg_0x113 &msg);
void decode_0x120(const uint8_t data[8], msg_0x120 &msg);
void decode_0x130(const uint8_t data[8], msg_0x130 &msg);
void decode_0x131(const uint8_t data[8], msg_0x131 &msg);

bool is_ok(const msg_0x100 &f100, const msg_0x101 &f101, const msg_0x113 &f113);
bool is_full_charge(const msg_0x100 &f100);
bool is_chargable(const msg_0x100 &f100, const msg_0x101 &f101);
bool is_charging(const msg_0x101 &f101);

}

// vim: set expandtab shiftwidth=4:
