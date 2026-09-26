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

// Production guarded readdress for the ToF chain (AMRSW-2322, Phase 2).
//
// This is NOT the diagnostic helper. The diag-era helpers (tof_diag_readdress
// on the diag branch) take an int-returning probe and treat any non-zero rc
// as "target free", which on the stock STM32 driver conflates a clean NACK
// with a timeout or bus error -- acceptable for an operator at a shell,
// unacceptable for unattended enumeration. This helper takes the tri-state
// probe natively and guarantees, testably:
//
//   - the target is treated as free ONLY on a clean NACK
//   - a transport error on the collision probe forbids the move: the
//     address-write count is provably zero
//   - the move is verified by an id read on the NEW address only, and the
//     id bytes must identify the expected model
//
// Register facts (same sources as the diag helpers): the VL53L4CX shares
// the VL53L1 core, whose address lives at I2C_SLAVE__DEVICE_ADDRESS
// (0x0001), 7-bit value, no register paging (vendored vl53l1_api.c). The
// VL53L7CX takes the 7-bit address at 0x0004 behind register page 0; its id
// registers (0x0000/0x0001) are also on page 0, so the move is verified
// before restoring page 2. Both register maps were exercised on the real
// boards on DS20001 (2026-08-05).
//
// Unlike the diagnostic helper there is no post-mortem and no lost-ACK
// recovery here: production enumeration treats every failure as a freeze
// and leaves diagnosis to the diag firmware. Deterministic and minimal.

#include <stdint.h>
#include <stddef.h>

#include "tof_enumerator.hpp"

namespace lexxhard::tof_readdress {

// L7 (VL53L5CX family)
inline constexpr uint16_t kL7PageReg{0x7fff};
inline constexpr uint16_t kL7AddrReg{0x0004};
inline constexpr uint16_t kL7IdReg{0x0000};
inline constexpr uint8_t kL7DeviceId{0xf0};
inline constexpr uint8_t kL7Revision{0x02};
// L4 (VL53L1 core)
inline constexpr uint16_t kL4AddrReg{0x0001};
inline constexpr uint16_t kL4IdReg{0x010f};
inline constexpr uint8_t kL4ModelId{0xeb};
inline constexpr uint8_t kL4ModuleType{0xaa};

struct i2c_ops {
    virtual tof_enum::probe_result probe(uint8_t addr7) = 0;
    // 0 on success, negative errno otherwise.
    virtual int wr8(uint8_t addr7, uint16_t reg, uint8_t value) = 0;
    virtual int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) = 0;
    virtual ~i2c_ops() = default;
};

// Moves the device answering at old7 to new7 and verifies it there.
// Stage semantics are tof_enum::readdress_stage; rc detail:
//   validate      -EINVAL      old == new (no bus traffic at all)
//   collision     -EADDRINUSE  target ACKed
//   collision     probe rc     target probe hit a transport error
//                              (address-write count is zero in both cases)
//   verify        -ENODEV      id bytes do not identify the expected model
//   other stages  errno        passed through from the failing operation
// On success (and on the -ENODEV path) `seen` carries the id bytes read.
tof_enum::readdress_result readdress(tof_enum::model m, i2c_ops &ops,
                                     uint8_t old7, uint8_t new7,
                                     tof_enum::id_bytes *seen = nullptr);

// Raw identity read with the same failure policy as the guarded move: after
// a transport failure nothing further is written. L7 (paged): a failed page
// select returns, a failed id read returns WITHOUT attempting the restore,
// the page-2 restore runs only after a successful read, and a failed
// restore fails the whole read (a device stuck on page 0 is not usable).
// L4: a single unpaged read. Returns 0 with `out` filled, else the errno of
// the failing operation; no id-match judgement here -- that belongs to the
// caller (the enumeration state machine).
int read_id(tof_enum::model m, i2c_ops &ops, uint8_t addr7, tof_enum::id_bytes &out);

}  // namespace lexxhard::tof_readdress
