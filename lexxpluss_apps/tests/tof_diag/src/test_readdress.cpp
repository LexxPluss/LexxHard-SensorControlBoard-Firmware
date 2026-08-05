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

// --- VL53L7CX staged readdress ---

namespace {

struct l7_fake final : readdress::i2c_ops {
    static constexpr int kMax{8};
    // scripted returns, consumed per call in order
    int probe_script[kMax]{-EIO, -EIO, -EIO, -EIO, -EIO, -EIO, -EIO, -EIO};
    int wr8_script[kMax]{};
    int rd_rc{0};
    uint8_t rd_data[2]{readdress::kL7DeviceId, readdress::kL7Revision};
    // recorded traffic
    int probe_n{0};
    uint8_t probe_addrs[kMax]{};
    int wr8_n{0};
    uint8_t wr8_addrs[kMax]{};
    uint16_t wr8_regs[kMax]{};
    uint8_t wr8_vals[kMax]{};
    int rd_n{0};
    uint8_t rd_addr{0};
    uint16_t rd_reg{0};
    size_t rd_len{0};

    int probe(uint8_t addr7) override
    {
        probe_addrs[probe_n] = addr7;
        return probe_script[probe_n++];
    }
    int wr8(uint8_t addr7, uint16_t reg, uint8_t value) override
    {
        wr8_addrs[wr8_n] = addr7;
        wr8_regs[wr8_n] = reg;
        wr8_vals[wr8_n] = value;
        return wr8_script[wr8_n++];
    }
    int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) override
    {
        ++rd_n;
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

ZTEST(tof_diag_readdress, test_l7_same_address_rejected_without_bus_traffic)
{
    l7_fake ops;
    auto const r{readdress::readdress_l7(ops, 0x29, 0x29)};
    zassert_equal(r.rc, -EINVAL);
    zassert_true(r.failed_at == readdress::l7_stage::validate);
    zassert_equal(ops.probe_n, 0);
    zassert_equal(ops.wr8_n, 0);
    zassert_equal(ops.rd_n, 0);
}

ZTEST(tof_diag_readdress, test_l7_occupied_new_address_refused_before_write)
{
    l7_fake ops;
    ops.probe_script[0] = 0;
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -EADDRINUSE);
    zassert_true(r.failed_at == readdress::l7_stage::collision);
    zassert_equal(ops.probe_addrs[0], 0x2a);
    zassert_equal(ops.wr8_n, 0);
}

ZTEST(tof_diag_readdress, test_l7_success_exact_traffic)
{
    l7_fake ops;
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, 0);
    zassert_true(r.failed_at == readdress::l7_stage::done);
    zassert_false(r.postmortem);
    zassert_false(r.write_ack_lost);
    zassert_equal(ops.wr8_n, 3);
    zassert_equal(ops.wr8_addrs[0], 0x29, "page select goes to the OLD address");
    zassert_equal(ops.wr8_regs[0], 0x7fff);
    zassert_equal(ops.wr8_vals[0], 0x00);
    zassert_equal(ops.wr8_addrs[1], 0x29, "address write goes to the OLD address");
    zassert_equal(ops.wr8_regs[1], 0x0004);
    zassert_equal(ops.wr8_vals[1], 0x2a, "7-bit address written directly");
    zassert_equal(ops.rd_addr, 0x2a, "id verify on the NEW address, before page restore");
    zassert_equal(ops.rd_reg, 0x0000);
    zassert_equal(ops.rd_len, 2u);
    zassert_equal(ops.wr8_addrs[2], 0x2a, "page-2 restore on the NEW address");
    zassert_equal(ops.wr8_regs[2], 0x7fff);
    zassert_equal(ops.wr8_vals[2], 0x02);
    zassert_equal(r.device_id, 0xf0);
    zassert_equal(r.revision, 0x02);
}

ZTEST(tof_diag_readdress, test_l7_page_select_failure_runs_postmortem)
{
    l7_fake ops;
    ops.wr8_script[0] = -EIO;
    ops.probe_script[1] = 0;  // old still ACKs
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::l7_stage::page_select);
    zassert_true(r.postmortem);
    zassert_equal(ops.probe_n, 3, "collision probe + post-mortem old + new");
    zassert_equal(ops.probe_addrs[1], 0x29);
    zassert_equal(ops.probe_addrs[2], 0x2a);
    zassert_equal(r.old_probe_rc, 0);
    zassert_not_equal(r.new_probe_rc, 0);
    zassert_equal(ops.wr8_n, 1, "nothing written after the failure");
    zassert_equal(ops.rd_n, 0);
}

ZTEST(tof_diag_readdress, test_l7_addr_write_failure_old_acks_stops)
{
    l7_fake ops;
    ops.wr8_script[1] = -EIO;
    ops.probe_script[1] = 0;   // old ACK: write did not take effect
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::l7_stage::addr_write);
    zassert_true(r.postmortem);
    zassert_false(r.write_ack_lost);
    zassert_equal(ops.rd_n, 0, "must not continue to verify");
    zassert_equal(ops.wr8_n, 2);
}

ZTEST(tof_diag_readdress, test_l7_addr_write_ack_lost_recovers_on_new_address)
{
    l7_fake ops;
    ops.wr8_script[1] = -EIO;  // the write's ACK is lost...
    ops.probe_script[2] = 0;   // ...but the device answers on the NEW address
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, 0, "verified move counts as success");
    zassert_true(r.failed_at == readdress::l7_stage::done);
    zassert_true(r.write_ack_lost);
    zassert_true(r.postmortem, "the evidence of the lost ACK is preserved");
    zassert_equal(ops.rd_addr, 0x2a);
    zassert_equal(ops.wr8_addrs[2], 0x2a, "page-2 restore still runs on the new address");
    zassert_equal(r.device_id, 0xf0);
}

ZTEST(tof_diag_readdress, test_l7_addr_write_failure_both_silent_stops)
{
    l7_fake ops;
    ops.wr8_script[1] = -EIO;  // both post-mortem probes stay -EIO
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::l7_stage::addr_write);
    zassert_true(r.postmortem);
    zassert_not_equal(r.old_probe_rc, 0);
    zassert_not_equal(r.new_probe_rc, 0);
    zassert_equal(ops.rd_n, 0, "must not continue when the device is silent everywhere");
}

ZTEST(tof_diag_readdress, test_l7_id_mismatch_returns_enodev_and_restores_page)
{
    l7_fake ops;
    ops.rd_data[0] = 0x00;
    ops.rd_data[1] = 0x00;
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -ENODEV, "a MISMATCH must not report success");
    zassert_true(r.failed_at == readdress::l7_stage::verify);
    zassert_equal(r.device_id, 0x00);
    zassert_equal(ops.wr8_n, 3, "page-2 restore still attempted on mismatch");
    zassert_equal(ops.wr8_regs[2], 0x7fff);
    zassert_equal(ops.wr8_vals[2], 0x02);
    zassert_equal(r.restore_rc, 0);
}

ZTEST(tof_diag_readdress, test_l7_verify_read_failure_runs_postmortem)
{
    l7_fake ops;
    ops.rd_rc = -EIO;
    ops.probe_script[2] = 0;  // device still on the new address
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::l7_stage::verify);
    zassert_true(r.postmortem);
    zassert_equal(r.new_probe_rc, 0);
}

ZTEST(tof_diag_readdress, test_l7_page_restore_failure_reported)
{
    l7_fake ops;
    ops.wr8_script[2] = -EIO;
    auto const r{readdress::readdress_l7(ops, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress::l7_stage::page_restore);
    zassert_true(r.postmortem);
    zassert_equal(r.device_id, 0xf0, "the id itself had verified before the restore failed");
}
