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

#include "tof_readdress.hpp"

namespace lexxhard::tof_readdress {

using tof_enum::model;
using tof_enum::probe_state;
using tof_enum::readdress_result;
using tof_enum::readdress_stage;

readdress_result readdress(model m, i2c_ops &ops, uint8_t old7, uint8_t new7,
                           tof_enum::id_bytes *seen)
{
    if (old7 == new7)
        return {-EINVAL, readdress_stage::validate};

    // Tri-state collision check: only a clean NACK proves the target free.
    // An ACK is a collision; a transport error proves nothing, and writing
    // an address on top of "nothing proven" is exactly the silent-merge
    // failure this helper exists to prevent -- so both refuse before any
    // write happens.
    auto const target{ops.probe(new7)};
    if (target.state == probe_state::ack)
        return {-EADDRINUSE, readdress_stage::collision};
    if (target.state == probe_state::transport_error)
        return {target.rc, readdress_stage::collision};

    uint16_t id_reg{};
    uint8_t want_first{}, want_second{};
    if (m == model::l7cx) {
        if (int const rc{ops.wr8(old7, kL7PageReg, 0x00)}; rc != 0)
            return {rc, readdress_stage::page_select};
        if (int const rc{ops.wr8(old7, kL7AddrReg, new7)}; rc != 0)
            return {rc, readdress_stage::addr_write};
        id_reg = kL7IdReg;
        want_first = kL7DeviceId;
        want_second = kL7Revision;
    } else {
        if (int const rc{ops.wr8(old7, kL4AddrReg, new7)}; rc != 0)
            return {rc, readdress_stage::addr_write};
        id_reg = kL4IdReg;
        want_first = kL4ModelId;
        want_second = kL4ModuleType;
    }

    uint8_t buf[2]{};
    if (int const rc{ops.rd(new7, id_reg, buf, sizeof buf)}; rc != 0)
        return {rc, readdress_stage::verify};
    if (seen != nullptr) {
        seen->first = buf[0];
        seen->second = buf[1];
    }
    bool const id_ok{buf[0] == want_first && buf[1] == want_second};

    if (m == model::l7cx) {
        // Restore page 2 whether or not the id matched: never leave a
        // responding device on page 0. A restore FAILURE outranks the id
        // verdict either way -- a device stuck on page 0 poisons the next
        // fresh run, and the caller must know; the mismatch evidence is not
        // lost because `seen` already carries the id bytes.
        int const restore_rc{ops.wr8(new7, kL7PageReg, 0x02)};
        if (restore_rc != 0)
            return {restore_rc, readdress_stage::page_restore};
        if (!id_ok)
            return {-ENODEV, readdress_stage::verify};
    } else if (!id_ok) {
        return {-ENODEV, readdress_stage::verify};
    }

    return {0, readdress_stage::none};
}

}  // namespace lexxhard::tof_readdress
