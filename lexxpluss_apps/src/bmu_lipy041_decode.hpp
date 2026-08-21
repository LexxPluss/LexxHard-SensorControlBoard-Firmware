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

// Bits considered abnormal in is_ok(). Named here so board_controller.cpp's
// debug logging can reference the same definition instead of duplicating literals.
inline constexpr uint8_t FAIL_STATUS1_ABNORMAL_MASK{0b10111111};  // excludes bit6 (full-charge info bit, checked separately by is_full_charge())
inline constexpr uint8_t FAIL_STATUS2_ABNORMAL_MASK{0b11111111};  // no reserved bits
inline constexpr uint8_t FAIL_STATUS3_ABNORMAL_MASK{0b11111111};  // self-test diagnostics, no reserved bits
inline constexpr uint8_t LEADER_ALARM1_ABNORMAL_MASK{0b00000111}; // bits 3-7 reserved
inline constexpr uint8_t LEADER_ALARM2_ABNORMAL_MASK{0b00001111}; // bits 4-7 reserved

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

// Bundles all 10 message structs consumed by bmu_controller.cpp's bmu_info shell
// output. Moved here (was previously lexxhard::bmu_controller::msg_bmu) so
// decode_frame_bmu_info() can be declared alongside the other decode functions.
struct msg_bmu {
    msg_0x100 f100;
    msg_0x101 f101;
    msg_0x103 f103;
    msg_0x110 f110;
    msg_0x111 f111;
    msg_0x112 f112;
    msg_0x113 f113;
    msg_0x120 f120;
    msg_0x130 f130;
    msg_0x131 f131;
} __attribute__((aligned(4)));

// All decode_0x1XX functions require dlc == 8 (every LIPY041 List# frame is a fixed
// 8 bytes per the datasheet). On dlc != 8 they leave msg untouched and return false,
// so a stale cached value is kept rather than mixing in hardware-register residue
// from a short frame (STM32 bxcan always copies 8 bytes regardless of dlc).
bool decode_0x100(const uint8_t (&data)[8], uint8_t dlc, msg_0x100 &msg);
bool decode_0x101(const uint8_t (&data)[8], uint8_t dlc, msg_0x101 &msg);
bool decode_0x103(const uint8_t (&data)[8], uint8_t dlc, msg_0x103 &msg);
bool decode_0x110(const uint8_t (&data)[8], uint8_t dlc, msg_0x110 &msg);
bool decode_0x111(const uint8_t (&data)[8], uint8_t dlc, msg_0x111 &msg);
bool decode_0x112(const uint8_t (&data)[8], uint8_t dlc, msg_0x112 &msg);
bool decode_0x113(const uint8_t (&data)[8], uint8_t dlc, msg_0x113 &msg);
bool decode_0x120(const uint8_t (&data)[8], uint8_t dlc, msg_0x120 &msg);
bool decode_0x130(const uint8_t (&data)[8], uint8_t dlc, msg_0x130 &msg);
bool decode_0x131(const uint8_t (&data)[8], uint8_t dlc, msg_0x131 &msg);

// ID -> decoder dispatch, one function per caller. Returns false only when id is
// recognized but dlc is invalid (the caller should log this); an unrecognized id
// returns true (nothing to decode, not an error -- matches the pre-existing
// silently-ignore-unknown-id behavior).
bool decode_frame_bmu_info(uint32_t id, const uint8_t (&data)[8], uint8_t dlc, msg_bmu &msg);
bool decode_frame_power_sequence(uint32_t id, const uint8_t (&data)[8], uint8_t dlc,
                                  msg_0x100 &f100, msg_0x101 &f101, msg_0x113 &f113);

bool is_ok(const msg_0x100 &f100, const msg_0x101 &f101, const msg_0x113 &f113);
bool is_full_charge(const msg_0x100 &f100);
bool is_chargable(const msg_0x100 &f100, const msg_0x101 &f101);
bool is_charging(const msg_0x101 &f101);

// POST-state power-on timeout. Named to record its basis: the LIPY041 datasheet's
// own "BM communication error" detection window is 30-90 seconds; this is not a
// completeness check (is_ok()'s 0xff defaults already make "not yet received"
// abnormal) -- it is purely a give-up backstop for "genuinely broken, or Leader BM
// never elected" cases.
inline constexpr int64_t POST_TIMEOUT_MS{90000};

enum class post_result { wait, standby, off };

// Pure decision for board_controller.cpp's POWER_STATE::POST handling. Does not
// consider should_turn_off() (a separate, orthogonal input handled by the caller).
post_result decide_post_transition(const msg_0x100 &f100, const msg_0x101 &f101, const msg_0x113 &f113,
                                    bool switch_released, int64_t elapsed_ms);

// Per-field diagnostic distinguishing "not yet received" (still at its 0xff startup
// default) from "received but showing an abnormal bit", so a stuck POST state can be
// explained rather than just timing out silently.
enum class field_health { ok, not_received, abnormal };

field_health describe_fail_status1(const msg_0x100 &f100);
field_health describe_fail_status2(const msg_0x101 &f101);
field_health describe_leader_alarm1(const msg_0x113 &f113);
field_health describe_leader_alarm2(const msg_0x113 &f113);

}

// vim: set expandtab shiftwidth=4:
