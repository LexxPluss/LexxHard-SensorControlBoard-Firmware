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

// Pure logic for VL53L4CX readdressing, host-testable through injected I2C
// operations. The shell wrapper with the real bus lives in tof_diag.cpp.
//
// The VL53L4CX shares the VL53L1 core: VL53L1_SetDeviceAddress() writes the
// 7-bit address to I2C_SLAVE__DEVICE_ADDRESS (0x0001) -- the ST API takes an
// 8-bit address and halves it before the write (vl53l1_api.c; the register
// map declares a 7-bit field, msb=6 lsb=0). There is no register-page
// mechanism on this part; the L7 sequence (page 0 + register 0x0004) must
// not be reused, the two parts differ in both respects.
//
// Ordering matters for safety on a chain where every part ships at the same
// default address: the new address is probed *before* the write, because
// writing onto an occupied address silently merges two devices -- the id
// readback would still MATCH and nothing downstream would ever notice.

#include <stdint.h>
#include <stddef.h>

namespace lexxhard::tof_diag_readdress {

inline constexpr uint16_t kAddrReg{0x0001};     // I2C_SLAVE__DEVICE_ADDRESS
inline constexpr uint16_t kModelIdReg{0x010f};  // model id, then module type
inline constexpr uint8_t kModelId{0xeb};
inline constexpr uint8_t kModuleType{0xaa};

struct i2c_ops {
    // All return 0 on ACK/success, a negative errno otherwise.
    virtual int probe(uint8_t addr7) = 0;
    virtual int wr8(uint8_t addr7, uint16_t reg, uint8_t value) = 0;
    virtual int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) = 0;
    virtual ~i2c_ops() = default;
};

enum class stage : uint8_t {
    validate,   // argument rejection, no bus traffic at all
    collision,  // new address already ACKs, write not attempted
    write,      // address write on the old address failed
    read,       // model id read on the new address failed
    verify,     // id bytes read but do not identify a VL53L4CX
    done,
};

struct result {
    int rc;              // 0 on success, negative errno on failure
    stage failed_at;     // stage::done when rc == 0
    uint8_t model_id;    // valid once the read stage succeeded
    uint8_t module_type;
};

result readdress_l4(i2c_ops &ops, uint8_t old_addr, uint8_t new_addr);

// --- VL53L7CX (L5CX family) ---
//
// The ULD sequence is: select register page 0, write the 7-bit address to
// 0x0004, continue on the new address. The id registers (0x0000/0x0001) are
// also on page 0, so the move is verified there before restoring page 2.
//
// A transient failure on DS20001 (2026-08-05) showed why every stage must be
// individually observable and why a failure must be followed by probing BOTH
// addresses immediately: LPn low resets the dynamic address to the default,
// so any recovery that touches LPn destroys the evidence of how far the
// write got. The post-mortem probes run before control returns to the
// operator, with LPn untouched.

inline constexpr uint16_t kL7PageReg{0x7fff};
inline constexpr uint16_t kL7AddrReg{0x0004};
inline constexpr uint16_t kL7IdReg{0x0000};
inline constexpr uint8_t kL7DeviceId{0xf0};
inline constexpr uint8_t kL7Revision{0x02};

enum class l7_stage : uint8_t {
    validate,      // argument rejection, no bus traffic
    collision,     // new address already ACKs, nothing written
    page_select,   // wr8(old, 0x7fff, 0x00) failed
    addr_write,    // wr8(old, 0x0004, new7) failed
    verify,        // id read on the new address failed or mismatched
    page_restore,  // wr8(new, 0x7fff, 0x02) failed
    done,
};

struct l7_result {
    int rc;              // 0 on success, negative errno on failure
    l7_stage failed_at;  // l7_stage::done when rc == 0
    uint8_t device_id;   // valid once the verify read succeeded
    uint8_t revision;
    // Filled whenever an I2C stage failed: both addresses probed at once,
    // LPn untouched. 0 = ACK. Feeds the operator decision table.
    bool postmortem;
    int old_probe_rc;
    int new_probe_rc;
    // The address write itself reported failure, but the post-mortem showed
    // the device answering on the new address only, and verify + restore
    // then succeeded there: the write landed and only its ACK was lost.
    bool write_ack_lost;
    // The id-mismatch path still restores page 2 (never leave a responding
    // device on page 0); this records that restore's own rc.
    int restore_rc;
};

l7_result readdress_l7(i2c_ops &ops, uint8_t old_addr, uint8_t new_addr);

}  // namespace lexxhard::tof_diag_readdress
