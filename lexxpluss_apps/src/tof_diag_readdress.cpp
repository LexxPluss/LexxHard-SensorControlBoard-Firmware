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

#include <errno.h>

#include "tof_diag_readdress.hpp"

namespace lexxhard::tof_diag_readdress {

result readdress_l4(i2c_ops &ops, uint8_t old_addr, uint8_t new_addr)
{
    result r{0, stage::done, 0, 0};
    if (old_addr == new_addr) {
        // A same-address "move" would report success without proving that
        // anything moved.
        r.rc = -EINVAL;
        r.failed_at = stage::validate;
        return r;
    }
    if (ops.probe(new_addr) == 0) {
        r.rc = -EADDRINUSE;
        r.failed_at = stage::collision;
        return r;
    }
    if (int const rc{ops.wr8(old_addr, kAddrReg, new_addr)}; rc != 0) {
        r.rc = rc;
        r.failed_at = stage::write;
        return r;
    }
    uint8_t buf[2]{};
    if (int const rc{ops.rd(new_addr, kModelIdReg, buf, sizeof buf)}; rc != 0) {
        r.rc = rc;
        r.failed_at = stage::read;
        return r;
    }
    r.model_id = buf[0];
    r.module_type = buf[1];
    if (buf[0] != kModelId || buf[1] != kModuleType) {
        r.rc = -ENODEV;
        r.failed_at = stage::verify;
        return r;
    }
    return r;
}

namespace {

void l7_postmortem(i2c_ops &ops, l7_result &r, uint8_t old_addr, uint8_t new_addr)
{
    r.postmortem = true;
    r.old_probe_rc = ops.probe(old_addr);
    r.new_probe_rc = ops.probe(new_addr);
}

}  // namespace

l7_result readdress_l7(i2c_ops &ops, uint8_t old_addr, uint8_t new_addr)
{
    l7_result r{0, l7_stage::done, 0, 0, false, 0, 0, false, 0};
    if (old_addr == new_addr) {
        r.rc = -EINVAL;
        r.failed_at = l7_stage::validate;
        return r;
    }
    if (ops.probe(new_addr) == 0) {
        r.rc = -EADDRINUSE;
        r.failed_at = l7_stage::collision;
        return r;
    }
    if (int const rc{ops.wr8(old_addr, kL7PageReg, 0x00)}; rc != 0) {
        r.rc = rc;
        r.failed_at = l7_stage::page_select;
        l7_postmortem(ops, r, old_addr, new_addr);
        return r;
    }
    if (int const rc{ops.wr8(old_addr, kL7AddrReg, new_addr)}; rc != 0) {
        r.rc = rc;
        r.failed_at = l7_stage::addr_write;
        l7_postmortem(ops, r, old_addr, new_addr);
        // Decision table: old silent + new answering means the write very
        // likely landed and only its ACK was lost -- continue on the new
        // address and let verify decide. Every other combination stops here.
        if (!(r.old_probe_rc != 0 && r.new_probe_rc == 0))
            return r;
        r.write_ack_lost = true;
    }
    uint8_t buf[2]{};
    if (int const rc{ops.rd(new_addr, kL7IdReg, buf, sizeof buf)}; rc != 0) {
        r.rc = rc;
        r.failed_at = l7_stage::verify;
        l7_postmortem(ops, r, old_addr, new_addr);
        return r;
    }
    r.device_id = buf[0];
    r.revision = buf[1];
    if (buf[0] != kL7DeviceId || buf[1] != kL7Revision) {
        // The device answered with the wrong identity. Still restore page 2
        // -- never leave a responding device on page 0 -- then report.
        r.restore_rc = ops.wr8(new_addr, kL7PageReg, 0x02);
        r.rc = -ENODEV;
        r.failed_at = l7_stage::verify;
        return r;
    }
    if (int const rc{ops.wr8(new_addr, kL7PageReg, 0x02)}; rc != 0) {
        r.rc = rc;
        r.failed_at = l7_stage::page_restore;
        l7_postmortem(ops, r, old_addr, new_addr);
        return r;
    }
    r.rc = 0;
    r.failed_at = l7_stage::done;
    return r;
}

}  // namespace lexxhard::tof_diag_readdress
