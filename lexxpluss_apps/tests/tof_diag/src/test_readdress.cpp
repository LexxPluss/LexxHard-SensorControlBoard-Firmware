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

// Host-side tests for the pure VL53L4CX readdress helper. The fake bus
// records every call so the tests can assert not only the outcome but the
// exact traffic: which register, which value, which address, and -- just as
// important -- which calls never happened.

#include <errno.h>

#include <zephyr/ztest.h>

#include "tof_diag_readdress.hpp"

namespace {

namespace readdress = lexxhard::tof_diag_readdress;

struct fake_ops final : readdress::i2c_ops {
    // scripted behaviour
    int probe_rc{-EIO};             // default: nothing ACKs on the new address
    int wr8_rc{0};
    int rd_rc{0};
    uint8_t rd_data[2]{readdress::kModelId, readdress::kModuleType};
    // recorded traffic
    int probe_calls{0};
    uint8_t probe_addr{0};
    int wr8_calls{0};
    uint8_t wr8_addr{0};
    uint16_t wr8_reg{0};
    uint8_t wr8_value{0};
    int rd_calls{0};
    uint8_t rd_addr{0};
    uint16_t rd_reg{0};
    size_t rd_len{0};

    int probe(uint8_t addr7) override
    {
        ++probe_calls;
        probe_addr = addr7;
        return probe_rc;
    }
    int wr8(uint8_t addr7, uint16_t reg, uint8_t value) override
    {
        ++wr8_calls;
        wr8_addr = addr7;
        wr8_reg = reg;
        wr8_value = value;
        return wr8_rc;
    }
    int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) override
    {
        ++rd_calls;
        rd_addr = addr7;
        rd_reg = reg;
        rd_len = len;
        if (rd_rc == 0) {
            for (size_t i{0}; i < len && i < sizeof rd_data; ++i)
                buf[i] = rd_data[i];
        }
        return rd_rc;
    }
};

}  // namespace

ZTEST_SUITE(tof_diag_readdress, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_diag_readdress, test_same_address_rejected_without_bus_traffic)
{
    fake_ops ops;
    auto const r{readdress::readdress_l4(ops, 0x29, 0x29)};
    zassert_equal(r.rc, -EINVAL);
    zassert_true(r.failed_at == readdress::stage::validate);
    zassert_equal(ops.probe_calls, 0);
    zassert_equal(ops.wr8_calls, 0);
    zassert_equal(ops.rd_calls, 0);
}

ZTEST(tof_diag_readdress, test_occupied_new_address_refused_before_write)
{
    fake_ops ops;
    ops.probe_rc = 0;  // something already ACKs on the new address
    auto const r{readdress::readdress_l4(ops, 0x29, 0x2c)};
    zassert_equal(r.rc, -EADDRINUSE);
    zassert_true(r.failed_at == readdress::stage::collision);
    zassert_equal(ops.probe_addr, 0x2c);
    zassert_equal(ops.wr8_calls, 0, "the address write must never happen on a collision");
    zassert_equal(ops.rd_calls, 0);
}

ZTEST(tof_diag_readdress, test_success_writes_exact_register_and_value)
{
    fake_ops ops;
    auto const r{readdress::readdress_l4(ops, 0x29, 0x2c)};
    zassert_equal(r.rc, 0);
    zassert_true(r.failed_at == readdress::stage::done);
    zassert_equal(ops.probe_calls, 1);
    zassert_equal(ops.probe_addr, 0x2c);
    zassert_equal(ops.wr8_calls, 1);
    zassert_equal(ops.wr8_addr, 0x29, "the write goes to the OLD address");
    zassert_equal(ops.wr8_reg, 0x0001, "I2C_SLAVE__DEVICE_ADDRESS");
    zassert_equal(ops.wr8_value, 0x2c, "the register takes the 7-bit address directly");
    zassert_equal(ops.rd_calls, 1);
    zassert_equal(ops.rd_addr, 0x2c, "the readback must use the NEW address only");
    zassert_equal(ops.rd_reg, 0x010f);
    zassert_equal(ops.rd_len, 2u);
    zassert_equal(r.model_id, 0xeb);
    zassert_equal(r.module_type, 0xaa);
}

ZTEST(tof_diag_readdress, test_write_failure_propagates_and_skips_read)
{
    fake_ops ops;
    ops.wr8_rc = -EIO;
    auto const r{readdress::readdress_l4(ops, 0x29, 0x2c)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::stage::write);
    zassert_equal(ops.rd_calls, 0);
}

ZTEST(tof_diag_readdress, test_read_failure_on_new_address_propagates)
{
    fake_ops ops;
    ops.rd_rc = -EIO;
    auto const r{readdress::readdress_l4(ops, 0x29, 0x2c)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::stage::read);
}

ZTEST(tof_diag_readdress, test_id_mismatch_returns_enodev)
{
    fake_ops ops;
    ops.rd_data[0] = 0xd0;  // a lone L7 answered these at the L4 id index
    ops.rd_data[1] = 0xcd;
    auto const r{readdress::readdress_l4(ops, 0x29, 0x2c)};
    zassert_equal(r.rc, -ENODEV, "a MISMATCH must not report success");
    zassert_true(r.failed_at == readdress::stage::verify);
    zassert_equal(r.model_id, 0xd0);
    zassert_equal(r.module_type, 0xcd);
}
