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

#include <zephyr/ztest.h>
#include "bmu_lipy041_decode.hpp"

using namespace lexxhard::bmu_lipy041;

ZTEST_SUITE(bmu_lipy041_decode, NULL, NULL, NULL, NULL, NULL);

// ---- 0x120 ----

ZTEST(bmu_lipy041_decode, test_0x120_normal_max_greater_than_min)
{
    uint8_t data[8] = {0x10, 0x68, 0x01, 0x00, 0x0e, 0xd8, 0x02, 0x00};  // 0x1068=4200, 0x0ed8=3800
    msg_0x120 msg{};
    decode_0x120(data, msg);
    zassert_equal(msg.max_cell_voltage.value, 4200);
    zassert_equal(msg.min_cell_voltage.value, 3800);
}

// LIA1020 stores Byte0-1=Min, Byte4-5=Max (the exact opposite layout). This guards
// against accidentally reintroducing the LIA1020 byte order for LIPY041.
ZTEST(bmu_lipy041_decode, test_0x120_regression_not_lia1020_layout)
{
    uint8_t data[8] = {0x10, 0x68, 0x01, 0x00, 0x0e, 0xd8, 0x02, 0x00};
    msg_0x120 msg{};
    decode_0x120(data, msg);
    zassert_not_equal(msg.max_cell_voltage.value, 3800);
}

ZTEST(bmu_lipy041_decode, test_0x120_module_id_decoded)
{
    uint8_t data[8] = {0x10, 0x68, 0x05, 0x00, 0x0e, 0xd0, 0x07, 0x00};
    msg_0x120 msg{};
    decode_0x120(data, msg);
    zassert_equal(msg.max_cell_voltage.id, 0x05);
    zassert_equal(msg.min_cell_voltage.id, 0x07);
}

ZTEST(bmu_lipy041_decode, test_0x120_boundary_zero)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    msg_0x120 msg{};
    decode_0x120(data, msg);
    zassert_equal(msg.max_cell_voltage.value, 0);
    zassert_equal(msg.min_cell_voltage.value, 0);
}

ZTEST(bmu_lipy041_decode, test_0x120_boundary_max)
{
    uint8_t data[8] = {0xff, 0xff, 0, 0, 0xff, 0xff, 0, 0};
    msg_0x120 msg{};
    decode_0x120(data, msg);
    zassert_equal(msg.max_cell_voltage.value, 0xFFFF);
    zassert_equal(msg.min_cell_voltage.value, 0xFFFF);
}

// ---- 0x131 (new frame, 32-bit) ----

ZTEST(bmu_lipy041_decode, test_0x131_thirty_two_bit_big_endian)
{
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0, 0, 0, 0};
    msg_0x131 msg{};
    decode_0x131(data, msg);
    zassert_equal(msg.accumulated_capacity, 0x01020304u);
}

ZTEST(bmu_lipy041_decode, test_0x131_zero_value)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    msg_0x131 msg{};
    decode_0x131(data, msg);
    zassert_equal(msg.accumulated_capacity, 0u);
}

ZTEST(bmu_lipy041_decode, test_0x131_max_value)
{
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0};
    msg_0x131 msg{};
    decode_0x131(data, msg);
    zassert_equal(msg.accumulated_capacity, 0xFFFFFFFFu);
}

// ---- 0x100 ----

ZTEST(bmu_lipy041_decode, test_0x100_fail_status1)
{
    uint8_t data[8] = {0x42, 0x01, 50, 60, 70, 0x00, 0x64, 0};
    msg_0x100 msg{};
    decode_0x100(data, msg);
    zassert_equal(msg.fail_status1, 0x42);
}

ZTEST(bmu_lipy041_decode, test_0x100_leader_battery_status)
{
    uint8_t data[8] = {0x00, 0x01, 50, 60, 70, 0x00, 0x64, 0};
    msg_0x100 msg{};
    decode_0x100(data, msg);
    zassert_equal(msg.leader_battery_status, 0x01);
}

ZTEST(bmu_lipy041_decode, test_0x100_asoc_rsoc_soh_min)
{
    uint8_t data[8] = {0x00, 0x01, 50, 60, 70, 0x00, 0x64, 0};
    msg_0x100 msg{};
    decode_0x100(data, msg);
    zassert_equal(msg.asoc_min, 50);
    zassert_equal(msg.rsoc_min, 60);
    zassert_equal(msg.soh_min, 70);
}

ZTEST(bmu_lipy041_decode, test_0x100_max_fet_temp_signed_big_endian)
{
    uint8_t data[8] = {0x00, 0x01, 50, 60, 70, 0x00, 0x64, 0};  // 0x0064 = 100 (10.0C)
    msg_0x100 msg{};
    decode_0x100(data, msg);
    zassert_equal(msg.max_fet_temp, 100);
}

ZTEST(bmu_lipy041_decode, test_0x100_max_fet_temp_negative)
{
    uint8_t data[8] = {0x00, 0x01, 50, 60, 70, 0xff, 0x9c, 0};  // 0xff9c = -100
    msg_0x100 msg{};
    decode_0x100(data, msg);
    zassert_equal(msg.max_fet_temp, -100);
}

// ---- 0x101 ----

ZTEST(bmu_lipy041_decode, test_0x101_average_current_positive)
{
    uint8_t data[8] = {0x00, 0x64, 0x00, 0xc8, 0x30, 0x39, 0x00, 0};  // avg=100, maxchg=200, vol=0x3039
    msg_0x101 msg{};
    decode_0x101(data, msg);
    zassert_equal(msg.average_current, 100);
}

ZTEST(bmu_lipy041_decode, test_0x101_average_current_negative)
{
    uint8_t data[8] = {0xff, 0x9c, 0x00, 0xc8, 0x30, 0x39, 0x00, 0};  // avg=-100
    msg_0x101 msg{};
    decode_0x101(data, msg);
    zassert_equal(msg.average_current, -100);
}

ZTEST(bmu_lipy041_decode, test_0x101_max_charging_current)
{
    uint8_t data[8] = {0x00, 0x64, 0x00, 0xc8, 0x30, 0x39, 0x00, 0};
    msg_0x101 msg{};
    decode_0x101(data, msg);
    zassert_equal(msg.max_charging_current, 200);
}

ZTEST(bmu_lipy041_decode, test_0x101_bm_voltage_max)
{
    uint8_t data[8] = {0x00, 0x64, 0x00, 0xc8, 0x30, 0x39, 0x00, 0};
    msg_0x101 msg{};
    decode_0x101(data, msg);
    zassert_equal(msg.bm_voltage_max, 0x3039);
}

ZTEST(bmu_lipy041_decode, test_0x101_fail_status2)
{
    uint8_t data[8] = {0x00, 0x64, 0x00, 0xc8, 0x30, 0x39, 0x81, 0};
    msg_0x101 msg{};
    decode_0x101(data, msg);
    zassert_equal(msg.fail_status2, 0x81);
}

// ---- 0x103 ----

ZTEST(bmu_lipy041_decode, test_0x103_design_fcc_rc_capacity)
{
    uint8_t data[8] = {0x27, 0x10, 0x1f, 0x40, 0x13, 0x88, 0x02, 0};  // 0x2710=10000,0x1f40=8000,0x1388=5000
    msg_0x103 msg{};
    decode_0x103(data, msg);
    zassert_equal(msg.design_capacity, 10000);
    zassert_equal(msg.fcc_min, 8000);
    zassert_equal(msg.rc_min, 5000);
}

ZTEST(bmu_lipy041_decode, test_0x103_fet_status_byte6)
{
    uint8_t data[8] = {0x27, 0x10, 0x1f, 0x40, 0x13, 0x88, 0x03, 0};
    msg_0x103 msg{};
    decode_0x103(data, msg);
    zassert_equal(msg.fet_status, 0x03);
}

// ---- 0x110/0x111/0x112 (unchanged from LIA1020) ----

ZTEST(bmu_lipy041_decode, test_0x110_max_min_voltage)
{
    uint8_t data[8] = {0x10, 0x68, 0x01, 0, 0x0e, 0xd8, 0x02, 0};
    msg_0x110 msg{};
    decode_0x110(data, msg);
    zassert_equal(msg.max_voltage.value, 4200);
    zassert_equal(msg.max_voltage.id, 1);
    zassert_equal(msg.min_voltage.value, 3800);
    zassert_equal(msg.min_voltage.id, 2);
}

ZTEST(bmu_lipy041_decode, test_0x111_max_min_temp_positive)
{
    uint8_t data[8] = {0x00, 0x19, 0x03, 0, 0x00, 0x0a, 0x04, 0};  // max=25, id=3; min=10, id=4
    msg_0x111 msg{};
    decode_0x111(data, msg);
    zassert_equal(msg.max_temp.value, 25);
    zassert_equal(msg.max_temp.id, 3);
    zassert_equal(msg.min_temp.value, 10);
    zassert_equal(msg.min_temp.id, 4);
}

ZTEST(bmu_lipy041_decode, test_0x111_max_min_temp_negative)
{
    uint8_t data[8] = {0xff, 0xd8, 0x05, 0, 0xff, 0xc9, 0x06, 0};  // max=-40, id=5; min=-55, id=6
    msg_0x111 msg{};
    decode_0x111(data, msg);
    zassert_equal(msg.max_temp.value, -40);
    zassert_equal(msg.max_temp.id, 5);
    zassert_equal(msg.min_temp.value, -55);
    zassert_equal(msg.min_temp.id, 6);
}

ZTEST(bmu_lipy041_decode, test_0x111_boundary_int16_min_max)
{
    uint8_t data[8] = {0x7f, 0xff, 0x07, 0, 0x80, 0x00, 0x08, 0};  // max=INT16_MAX(32767), id=7; min=INT16_MIN(-32768), id=8
    msg_0x111 msg{};
    decode_0x111(data, msg);
    zassert_equal(msg.max_temp.value, 32767);
    zassert_equal(msg.max_temp.id, 7);
    zassert_equal(msg.min_temp.value, -32768);
    zassert_equal(msg.min_temp.id, 8);
}

ZTEST(bmu_lipy041_decode, test_0x111_boundary_zero_and_negative_one)
{
    uint8_t data[8] = {0x00, 0x00, 0x09, 0, 0xff, 0xff, 0x0a, 0};  // max=0, id=9; min=-1, id=10
    msg_0x111 msg{};
    decode_0x111(data, msg);
    zassert_equal(msg.max_temp.value, 0);
    zassert_equal(msg.max_temp.id, 9);
    zassert_equal(msg.min_temp.value, -1);
    zassert_equal(msg.min_temp.id, 10);
}

ZTEST(bmu_lipy041_decode, test_0x112_max_min_current_signed)
{
    uint8_t data[8] = {0xff, 0x9c, 0x01, 0, 0x00, 0x64, 0x02, 0};
    msg_0x112 msg{};
    decode_0x112(data, msg);
    zassert_equal(msg.max_current.value, -100);
    zassert_equal(msg.min_current.value, 100);
}

// ---- 0x113 ----

ZTEST(bmu_lipy041_decode, test_0x113_fw_data_version)
{
    uint8_t data[8] = {0x01, 0x02, 0xaa, 0x05, 0x00, 0x00, 0x00, 0};
    msg_0x113 msg{};
    decode_0x113(data, msg);
    zassert_equal(msg.fw_ver, 0x01);
    zassert_equal(msg.data_ver, 0x02);
}

ZTEST(bmu_lipy041_decode, test_0x113_connected_bm_num_at_byte3_not_byte2)
{
    uint8_t data[8] = {0x01, 0x02, 0xaa, 0x05, 0x00, 0x00, 0x00, 0};  // byte2=0xaa is Reserved
    msg_0x113 msg{};
    decode_0x113(data, msg);
    zassert_equal(msg.connected_bm_count, 0x05);
}

ZTEST(bmu_lipy041_decode, test_0x113_leader_alarm1_and_2)
{
    uint8_t data[8] = {0x01, 0x02, 0x00, 0x05, 0x03, 0x0f, 0x00, 0};
    msg_0x113 msg{};
    decode_0x113(data, msg);
    zassert_equal(msg.leader_alarm1, 0x03);
    zassert_equal(msg.leader_alarm2, 0x0f);
}

ZTEST(bmu_lipy041_decode, test_0x113_fail_status3)
{
    uint8_t data[8] = {0x01, 0x02, 0x00, 0x05, 0x00, 0x00, 0x80, 0};
    msg_0x113 msg{};
    decode_0x113(data, msg);
    zassert_equal(msg.fail_status3, 0x80);
}

// ---- 0x130 (unchanged from LIA1020) ----

ZTEST(bmu_lipy041_decode, test_0x130_mfg_insp_serial)
{
    uint8_t data[8] = {0x26, 0x08, 0x00, 0x01, 0x12, 0x34, 0x00, 0};
    msg_0x130 msg{};
    decode_0x130(data, msg);
    zassert_equal(msg.manufacturing, 0x2608);
    zassert_equal(msg.inspection, 0x0001);
    zassert_equal(msg.serial, 0x1234);
}

// ---- is_ok() / is_full_charge() / is_chargable() / is_charging() ----

ZTEST(bmu_lipy041_decode, test_is_ok_all_clear_is_ok)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0;
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0;
    f113.leader_alarm2 = 0;
    zassert_true(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_full_charge_bit_alone_still_ok)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0b01000000;  // bit6 only: informational, excluded from mask
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0;
    f113.leader_alarm2 = 0;
    zassert_true(is_ok(f100, f101, f113));
}

// is_ok() uses AND: any single field with an abnormal bit makes it false.
// The original LIA1020 implementation (board_controller.cpp:902-905) used OR, which is
// only false when all four fields are abnormal simultaneously -- inconsistent with
// !is_ok() being used as the power-shutdown trigger at 7 call sites. Ported as AND.
ZTEST(bmu_lipy041_decode, test_is_ok_fail_status1_abnormal_bit_detected)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0b00000001;  // bit0: over current
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0;
    f113.leader_alarm2 = 0;
    zassert_false(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_fail_status2_abnormal_bit_detected)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0;
    f101.fail_status2 = 0b00000001;
    f113.leader_alarm1 = 0;
    f113.leader_alarm2 = 0;
    zassert_false(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_leader_alarm1_abnormal_bit_detected)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0;
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0b00000001;
    f113.leader_alarm2 = 0;
    zassert_false(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_leader_alarm1_reserved_bit_ignored)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0;
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0b00001000;  // bit3: reserved, masked out
    f113.leader_alarm2 = 0;
    zassert_true(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_leader_alarm2_abnormal_bit_detected)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0;
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0;
    f113.leader_alarm2 = 0b00000001;
    zassert_false(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_leader_alarm2_reserved_bit_ignored)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    f100.fail_status1 = 0;
    f101.fail_status2 = 0;
    f113.leader_alarm1 = 0;
    f113.leader_alarm2 = 0b00010000;  // bit4: reserved, masked out
    zassert_true(is_ok(f100, f101, f113));
}

// Fields default to 0xff (treat as abnormal until a real frame arrives). With the old
// OR implementation, one field receiving a healthy value while the other three still
// held their 0xff startup default could make is_ok() spuriously true; AND requires all
// fields to be confirmed healthy first.
ZTEST(bmu_lipy041_decode, test_is_ok_startup_all_default_before_any_frame_received)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    msg_0x113 f113{};
    zassert_false(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_ok_startup_only_one_field_received_still_not_ok)
{
    msg_0x100 f100{};
    f100.fail_status1 = 0;  // only 0x100 received and healthy
    msg_0x101 f101{};       // 0x101 not yet received, still default 0xff
    msg_0x113 f113{};       // 0x113 not yet received, still default 0xff
    zassert_false(is_ok(f100, f101, f113));
}

ZTEST(bmu_lipy041_decode, test_is_full_charge_bit6_set)
{
    msg_0x100 f100{};
    f100.fail_status1 = 0b01000000;
    zassert_true(is_full_charge(f100));
}

ZTEST(bmu_lipy041_decode, test_is_full_charge_other_bits_do_not_trigger)
{
    msg_0x100 f100{};
    f100.fail_status1 = 0b10111111;
    zassert_false(is_full_charge(f100));
}

ZTEST(bmu_lipy041_decode, test_is_chargable_true_when_not_full_and_below_threshold)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    f100.fail_status1 = 0;
    f100.rsoc_min = 94;
    zassert_true(is_chargable(f100, f101));
}

ZTEST(bmu_lipy041_decode, test_is_chargable_false_when_full_charge)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    f100.fail_status1 = 0b01000000;
    f100.rsoc_min = 50;
    zassert_false(is_chargable(f100, f101));
}

ZTEST(bmu_lipy041_decode, test_is_chargable_false_when_rsoc_at_threshold)
{
    msg_0x100 f100{};
    msg_0x101 f101{};
    f100.fail_status1 = 0;
    f100.rsoc_min = 95;
    zassert_false(is_chargable(f100, f101));
}

ZTEST(bmu_lipy041_decode, test_is_charging_true_when_positive)
{
    msg_0x101 f101{};
    f101.average_current = 1;
    zassert_true(is_charging(f101));
}

ZTEST(bmu_lipy041_decode, test_is_charging_false_when_zero_or_negative)
{
    msg_0x101 f101{};
    f101.average_current = 0;
    zassert_false(is_charging(f101));
    f101.average_current = -1;
    zassert_false(is_charging(f101));
}
