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

using namespace lcs_test;
namespace lcs = lexxhard::lcs;

ZTEST_SUITE(lcs_core_crc16, NULL, NULL, NULL, NULL, NULL);

/* Bit-wise reference step: polynomial 0x1021, no reflection. */
static uint16_t ref_step_bitwise(const uint16_t reg, const uint8_t byte, const uint32_t poly = 0x1021U)
{
    uint32_t r{static_cast<uint32_t>(reg) ^ (static_cast<uint32_t>(byte) << 8)};
    for (int bit{0}; bit < 8; ++bit) {
        r = (r & 0x8000U) != 0U ? ((r << 1) ^ poly) & 0xFFFFU : (r << 1) & 0xFFFFU;
    }
    return static_cast<uint16_t>(r);
}

/* Deliberately wrong step (polynomial 0x1022): the negative control of the exhaustive comparison. */
static uint16_t wrong_step(const uint16_t reg, const uint8_t byte)
{
    return ref_step_bitwise(reg, byte, 0x1022U);
}

struct StepComparison {
    uint32_t compared{0};
    bool mismatch{false};
    uint32_t first_reg{0};
    uint32_t first_byte{0};
};

/* Compares the step under test with the bit-wise step for every (register, byte); stops at the first mismatch. */
template <typename Step>
static StepComparison compare_all(const Step &step)
{
    StepComparison result{};
    for (uint32_t reg{0}; reg < 0x10000U; ++reg) {
        for (uint32_t byte{0}; byte < 0x100U; ++byte) {
            const auto r{static_cast<uint16_t>(reg)};
            const auto b{static_cast<uint8_t>(byte)};
            ++result.compared;
            if (step(r, b) != ref_step_bitwise(r, b)) {
                result.mismatch = true;
                result.first_reg = reg;
                result.first_byte = byte;
                return result;
            }
        }
    }
    return result;
}

ZTEST(lcs_core_crc16, test_lcs_core_crc2_step_matches_bitwise_exhaustive)
{
    const auto step_under_test = [](const uint16_t reg, const uint8_t byte) { return lcs::crc16_step(reg, byte); };
    const StepComparison result{compare_all(step_under_test)};
    zassert_false(result.mismatch, "step differs from bit-wise at register 0x%04x byte 0x%02x", result.first_reg,
                  result.first_byte);
    zassert_equal(result.compared, 0x1000000U, "compared %u pairs", result.compared);

    /* Negative control: the comparison must detect a wrong step. */
    const StepComparison control{compare_all(wrong_step)};
    zassert_true(control.mismatch, "the comparison did not detect a wrong step");
}

ZTEST(lcs_core_crc16, test_lcs_core_crc3_fold_matches_reference)
{
    for (size_t len{0}; len <= 12; ++len) {
        Bytes bytes{};
        for (size_t i{0}; i < len; ++i) {
            /* Deterministic bytes; 0x00, 0xFF and 0x4B appear at fixed positions. */
            static constexpr std::array<uint8_t, 12> kPattern{0x4B, 0x00, 0xFF, 0x12, 0x80, 0x4B,
                                                              0xFF, 0x00, 0xA5, 0x5A, 0x01, 0x7F};
            bytes.push(kPattern[i]);
        }
        uint16_t folded{0xFFFF};
        for (size_t i{0}; i < len; ++i) {
            folded = ref_step_bitwise(folded, bytes.data[i]);
        }
        zassert_equal(lcs::crc16(bytes.data.data(), len), folded, "bit-wise fold, length %u",
                      static_cast<unsigned>(len));
        zassert_equal(lcs::crc16(bytes.data.data(), len), ref_crc16(bytes, len), "table reference, length %u",
                      static_cast<unsigned>(len));
    }
}
