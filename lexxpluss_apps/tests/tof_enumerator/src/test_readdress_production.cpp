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

// Exact-traffic tests for the PRODUCTION readdress helper. These exist
// because the enumerator's own fakes cannot prove properties internal to
// this operation -- above all that a transport error on the collision probe
// leaves the address-write count at zero, the exact hole a pre-probing
// adapter over the diag-era helpers would have reopened.

#include <errno.h>

#include <zephyr/ztest.h>

#include "tof_readdress.hpp"

namespace {

namespace rd = lexxhard::tof_readdress;
using lexxhard::tof_enum::id_bytes;
using lexxhard::tof_enum::model;
using lexxhard::tof_enum::probe_result;
using lexxhard::tof_enum::probe_state;
using lexxhard::tof_enum::readdress_stage;

struct fake_bus final : rd::i2c_ops {
    static constexpr int kMax{8};
    // scripted behaviour
    probe_result probe_reply{probe_state::nack, 0};
    int wr8_script[kMax]{};
    int rd_rc{0};
    uint8_t rd_data[2]{0, 0};
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

    probe_result probe(uint8_t addr7) override
    {
        probe_addrs[probe_n++] = addr7;
        return probe_reply;
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

fake_bus l7_bus()
{
    fake_bus b;
    b.rd_data[0] = rd::kL7DeviceId;
    b.rd_data[1] = rd::kL7Revision;
    return b;
}

fake_bus l4_bus()
{
    fake_bus b;
    b.rd_data[0] = rd::kL4ModelId;
    b.rd_data[1] = rd::kL4ModuleType;
    return b;
}

}  // namespace

ZTEST_SUITE(tof_readdress_production, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_readdress_production, test_same_address_rejected_without_bus_traffic)
{
    fake_bus bus{l4_bus()};
    auto const r{rd::readdress(model::l4cx, bus, 0x29, 0x29)};
    zassert_equal(r.rc, -EINVAL);
    zassert_true(r.failed_at == readdress_stage::validate);
    zassert_equal(bus.probe_n, 0);
    zassert_equal(bus.wr8_n, 0);
    zassert_equal(bus.rd_n, 0);
}

ZTEST(tof_readdress_production, test_target_ack_refused_with_zero_writes)
{
    fake_bus bus{l4_bus()};
    bus.probe_reply = {probe_state::ack, 0};
    auto const r{rd::readdress(model::l4cx, bus, 0x29, 0x2c)};
    zassert_equal(r.rc, -EADDRINUSE);
    zassert_true(r.failed_at == readdress_stage::collision);
    zassert_equal(bus.probe_addrs[0], 0x2c);
    zassert_equal(bus.wr8_n, 0);
    zassert_equal(bus.rd_n, 0);
}

// The property that forbids reusing the diag-era helpers: a transport error
// on the collision probe proves nothing about the target, so the address
// write must provably never happen.
ZTEST(tof_readdress_production, test_transport_error_on_collision_probe_forbids_the_write)
{
    static constexpr model kModels[]{model::l4cx, model::l7cx};
    for (auto const m : kModels) {
        fake_bus bus = (m == model::l4cx) ? l4_bus() : l7_bus();
        bus.probe_reply = {probe_state::transport_error, -ETIMEDOUT};
        auto const r{rd::readdress(m, bus, 0x29, 0x2a)};
        zassert_equal(r.rc, -ETIMEDOUT, "probe rc must pass through");
        zassert_true(r.failed_at == readdress_stage::collision);
        zassert_equal(bus.wr8_n, 0, "address-write count must be zero");
        zassert_equal(bus.rd_n, 0);
    }
}

ZTEST(tof_readdress_production, test_l4_success_exact_traffic)
{
    fake_bus bus{l4_bus()};
    id_bytes seen{};
    auto const r{rd::readdress(model::l4cx, bus, 0x29, 0x2c, &seen)};
    zassert_equal(r.rc, 0);
    zassert_true(r.failed_at == readdress_stage::none);
    zassert_equal(bus.probe_n, 1);
    zassert_equal(bus.wr8_n, 1, "L4 has no page registers");
    zassert_equal(bus.wr8_addrs[0], 0x29, "write goes to the OLD address");
    zassert_equal(bus.wr8_regs[0], rd::kL4AddrReg);
    zassert_equal(bus.wr8_vals[0], 0x2c, "7-bit value written directly");
    zassert_equal(bus.rd_n, 1);
    zassert_equal(bus.rd_addr, 0x2c, "verify on the NEW address only");
    zassert_equal(bus.rd_reg, rd::kL4IdReg);
    zassert_equal(bus.rd_len, 2u);
    zassert_equal(seen.first, rd::kL4ModelId);
    zassert_equal(seen.second, rd::kL4ModuleType);
}

ZTEST(tof_readdress_production, test_l7_success_exact_traffic)
{
    fake_bus bus{l7_bus()};
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a)};
    zassert_equal(r.rc, 0);
    zassert_true(r.failed_at == readdress_stage::none);
    zassert_equal(bus.wr8_n, 3);
    zassert_equal(bus.wr8_addrs[0], 0x29);
    zassert_equal(bus.wr8_regs[0], rd::kL7PageReg);
    zassert_equal(bus.wr8_vals[0], 0x00, "page 0 first");
    zassert_equal(bus.wr8_addrs[1], 0x29);
    zassert_equal(bus.wr8_regs[1], rd::kL7AddrReg);
    zassert_equal(bus.wr8_vals[1], 0x2a);
    zassert_equal(bus.rd_addr, 0x2a, "id verified before the page restore");
    zassert_equal(bus.rd_reg, rd::kL7IdReg);
    zassert_equal(bus.wr8_addrs[2], 0x2a, "page-2 restore on the NEW address");
    zassert_equal(bus.wr8_regs[2], rd::kL7PageReg);
    zassert_equal(bus.wr8_vals[2], 0x02);
}

ZTEST(tof_readdress_production, test_l7_id_mismatch_still_restores_page)
{
    fake_bus bus{l7_bus()};
    bus.rd_data[0] = 0x00;
    bus.rd_data[1] = 0x00;
    id_bytes seen{};
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a, &seen)};
    zassert_equal(r.rc, -ENODEV, "a mismatch is never success");
    zassert_true(r.failed_at == readdress_stage::verify);
    zassert_equal(bus.wr8_n, 3, "page-2 restore still attempted");
    zassert_equal(bus.wr8_regs[2], rd::kL7PageReg);
    zassert_equal(bus.wr8_vals[2], 0x02);
    zassert_equal(seen.first, 0x00, "the mismatching bytes are reported");
}

ZTEST(tof_readdress_production, test_l4_id_mismatch_returns_enodev)
{
    fake_bus bus{l4_bus()};
    bus.rd_data[0] = 0xd0;
    bus.rd_data[1] = 0xcd;
    auto const r{rd::readdress(model::l4cx, bus, 0x29, 0x2c)};
    zassert_equal(r.rc, -ENODEV);
    zassert_true(r.failed_at == readdress_stage::verify);
    zassert_equal(bus.wr8_n, 1, "no page registers on the L4");
}

ZTEST(tof_readdress_production, test_l7_page_select_failure_stops_before_the_address_write)
{
    fake_bus bus{l7_bus()};
    bus.wr8_script[0] = -EIO;
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::page_select);
    zassert_equal(bus.wr8_n, 1, "address write must not follow a failed page select");
    zassert_equal(bus.rd_n, 0);
}

ZTEST(tof_readdress_production, test_addr_write_failure_stops_before_the_read)
{
    fake_bus bus{l4_bus()};
    bus.wr8_script[0] = -EIO;
    auto const r{rd::readdress(model::l4cx, bus, 0x29, 0x2c)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::addr_write);
    zassert_equal(bus.rd_n, 0);
}

ZTEST(tof_readdress_production, test_verify_read_failure_propagates)
{
    fake_bus bus{l4_bus()};
    bus.rd_rc = -EIO;
    auto const r{rd::readdress(model::l4cx, bus, 0x29, 0x2c)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::verify);
}

ZTEST(tof_readdress_production, test_l7_page_restore_failure_reported_after_good_id)
{
    fake_bus bus{l7_bus()};
    bus.wr8_script[2] = -EIO;
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::page_restore);
}

// A failed page-2 restore outranks the id verdict: a device stuck on page 0
// poisons the next fresh run, and `seen` keeps the mismatch evidence anyway.
ZTEST(tof_readdress_production, test_l7_mismatch_with_failed_restore_reports_the_restore)
{
    fake_bus bus{l7_bus()};
    bus.rd_data[0] = 0x00;
    bus.rd_data[1] = 0x00;
    bus.wr8_script[2] = -EIO;  // the restore write fails
    id_bytes seen{};
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a, &seen)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::page_restore);
    zassert_equal(seen.first, 0x00, "mismatch evidence still delivered via seen");
}

ZTEST(tof_readdress_production, test_l7_addr_write_failure_after_page_select_stops)
{
    fake_bus bus{l7_bus()};
    bus.wr8_script[1] = -EIO;  // page select succeeded, address write fails
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::addr_write);
    zassert_equal(bus.wr8_n, 2, "no restore write after a failed address write");
    zassert_equal(bus.rd_n, 0, "no id read after a failed address write");
}

// Policy under test: a failed verify READ freezes without attempting the
// page restore -- the device's page state is unknown and further writes to
// an unresponsive address prove nothing (the L4 test alone would not cover
// the L7 branch).
ZTEST(tof_readdress_production, test_l7_verify_read_failure_freezes_without_restore)
{
    fake_bus bus{l7_bus()};
    bus.rd_rc = -EIO;
    auto const r{rd::readdress(model::l7cx, bus, 0x29, 0x2a)};
    zassert_equal(r.rc, -EIO);
    zassert_true(r.failed_at == readdress_stage::verify);
    zassert_equal(bus.wr8_n, 2, "no page-2 restore after a failed verify read");
}
