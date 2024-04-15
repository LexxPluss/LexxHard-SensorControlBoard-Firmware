/*
 * Copyright (c) 2024, LexxPluss Inc.
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

namespace lexxhard::bmu_controller {

#define BMU_CAN_DATA_LENGTH 8

struct msg_bmu {
    struct {
        uint16_t value;
        uint8_t id;
    } max_voltage, min_voltage, max_cell_voltage, min_cell_voltage;
    struct {
        int16_t value;
        uint8_t id;
    } max_temp, min_temp, max_current, min_current;
    int16_t fet_temp, pack_current;
    uint16_t charging_current, pack_voltage, design_capacity, full_charge_capacity, remain_capacity;
    uint16_t manufacturing, inspection, serial;
    uint8_t mod_status1, mod_status2, bmu_status, asoc, rsoc, soh;
    uint8_t bmu_fw_ver, mod_fw_ver, serial_config, parallel_config, bmu_alarm1, bmu_alarm2;
} __attribute__((aligned(4)));

struct msg_rawframe_bmu {
    uint8_t frame[BMU_CAN_DATA_LENGTH];
} __attribute__((aligned(4)));


struct msg_can_bmu {
    uint32_t can_id;
    uint8_t frame[BMU_CAN_DATA_LENGTH];
} __attribute__((aligned(4)));

void init();
void run(void *p1, void *p2, void *p3);
uint32_t get_rsoc();
extern k_thread thread;
extern k_msgq msgq_parsed_bmu, msgq_can_recv_bmu, msgq_rawframe_bmu, msgq_board, msgq_control;
}

// vim: set expandtab shiftwidth=4: