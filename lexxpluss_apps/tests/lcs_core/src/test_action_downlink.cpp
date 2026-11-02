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

#include <array>
#include <cstddef>
#include <cstdint>

#include <zephyr/ztest.h>

#include "lcs_core_test_support.hpp"
#include "lcs_crc16.hpp"
#include "lcs_frame.hpp"

using namespace lcs_test;
namespace lcs = lexxhard::lcs;

ZTEST_SUITE(lcs_action_downlink, NULL, NULL, NULL, NULL, NULL);

using Frame8 = std::array<uint8_t, 8>;

struct Golden {
    uint8_t target;
    uint8_t cmd;
    uint16_t param;
    Frame8 frame;
};

/* Fixed vectors: every other implementation of the ACTION downlink must reproduce these bytes. */
static constexpr std::array<Golden, 9> kTargetCmdGoldens{{
    {0x00, 0x48, 0x0000, {0x4B, 0x08, 0x00, 0x48, 0x00, 0x00, 0x11, 0x83}},
    {0x01, 0x48, 0x0000, {0x4B, 0x08, 0x01, 0x48, 0x00, 0x00, 0x67, 0x37}},
    {0xFF, 0x48, 0x0000, {0x4B, 0x08, 0xFF, 0x48, 0x00, 0x00, 0x5A, 0x20}},
    {0x00, 0x50, 0x0000, {0x4B, 0x08, 0x00, 0x50, 0x00, 0x00, 0xFB, 0x41}},
    {0x01, 0x50, 0x0000, {0x4B, 0x08, 0x01, 0x50, 0x00, 0x00, 0x8D, 0xF5}},
    {0xFF, 0x50, 0x0000, {0x4B, 0x08, 0xFF, 0x50, 0x00, 0x00, 0xB0, 0xE2}},
    {0x00, 0x68, 0x0000, {0x4B, 0x08, 0x00, 0x68, 0x00, 0x00, 0x97, 0x45}},
    {0x01, 0x68, 0x0000, {0x4B, 0x08, 0x01, 0x68, 0x00, 0x00, 0xE1, 0xF1}},
    {0xFF, 0x68, 0x0000, {0x4B, 0x08, 0xFF, 0x68, 0x00, 0x00, 0xDC, 0xE6}},
}};

static constexpr std::array<Golden, 5> kParamGoldens{{
    {0x00, 0x48, 0x0000, {0x4B, 0x08, 0x00, 0x48, 0x00, 0x00, 0x11, 0x83}},
    {0x00, 0x48, 0x00FF, {0x4B, 0x08, 0x00, 0x48, 0x00, 0xFF, 0x0F, 0x73}},
    {0x00, 0x48, 0xFF00, {0x4B, 0x08, 0x00, 0x48, 0xFF, 0x00, 0x12, 0x7C}},
    {0x00, 0x48, 0xFFFF, {0x4B, 0x08, 0x00, 0x48, 0xFF, 0xFF, 0x0C, 0x8C}},
    {0x00, 0x48, 0x1234, {0x4B, 0x08, 0x00, 0x48, 0x12, 0x34, 0x02, 0x45}},
}};

static constexpr Frame8 kCrcFrame{0x4B, 0x08, 0x00, 0x48, 0x12, 0x34, 0x02, 0x45};

static Bytes to_bytes(const Frame8 &array)
{
    Bytes bytes;
    for (const uint8_t byte : array) {
        bytes.push(byte);
    }
    return bytes;
}

static void check_golden(const Golden &golden)
{
    const Frame8 actual{lcs::encode_downlink(golden.target, golden.cmd, golden.param)};
    zassert_equal(actual, golden.frame);
    zassert_equal(actual[4], static_cast<uint8_t>(golden.param >> 8));
    zassert_equal(actual[5], static_cast<uint8_t>(golden.param & 0xFFU));
    const uint16_t stored{static_cast<uint16_t>((actual[6] << 8) | actual[7])};
    zassert_equal(stored, ref_crc16(to_bytes(actual), 6));
}

static_assert(lcs::encode_downlink(0x00, 0x48, 0x0000) ==
                  std::array<uint8_t, 8>{0x4B, 0x08, 0x00, 0x48, 0x00, 0x00, 0x11, 0x83},
              "the encoder must stay a constant expression with the golden bytes");

ZTEST(lcs_action_downlink, test_sc04_01_golden_target_cmd)
{
    for (const Golden &golden : kTargetCmdGoldens) {
        check_golden(golden);
    }
}

ZTEST(lcs_action_downlink, test_sc04_02_param_big_endian)
{
    for (const Golden &golden : kParamGoldens) {
        check_golden(golden);
    }
}

/* The CRC bytes of the encoded frame must change when any single bit of target, cmd or param changes. */
static void expect_crc_changes(const uint8_t target, const uint8_t cmd, const uint16_t param, const unsigned bit)
{
    const Frame8 frame{lcs::encode_downlink(target, cmd, param)};
    zassert_equal(frame[2], target, "target reaches byte 2");
    zassert_equal(frame[3], cmd, "cmd reaches byte 3");
    zassert_equal(frame[4], static_cast<uint8_t>(param >> 8), "param high reaches byte 4");
    zassert_equal(frame[5], static_cast<uint8_t>(param & 0xFFU), "param low reaches byte 5");
    const uint16_t stored{static_cast<uint16_t>((frame[6] << 8) | frame[7])};
    zassert_not_equal(stored, 0x0245, "a flip of input bit %u went unnoticed by the CRC", bit);
}

ZTEST(lcs_action_downlink, test_sc04_03_crc_covers_target_cmd_and_param)
{
    zassert_equal(lcs::encode_downlink(0x00, 0x48, 0x1234), kCrcFrame);
    for (unsigned bit{0}; bit < 8; ++bit) {
        expect_crc_changes(static_cast<uint8_t>(0x00U ^ (1U << bit)), 0x48, 0x1234, bit);
        expect_crc_changes(0x00, static_cast<uint8_t>(0x48U ^ (1U << bit)), 0x1234, bit);
    }
    for (unsigned bit{0}; bit < 16; ++bit) {
        expect_crc_changes(0x00, 0x48, static_cast<uint16_t>(0x1234U ^ (1U << bit)), bit);
    }
}

ZTEST(lcs_action_downlink, test_sc04_04_stateless_constexpr)
{
    const Frame8 first{lcs::encode_downlink(0x00, 0x48, 0x0000)};
    const Frame8 second{lcs::encode_downlink(0x00, 0x48, 0x0000)};
    zassert_equal(first, second);
    zassert_equal(first, kTargetCmdGoldens[0].frame);
}
