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

// stdint.h rather than cstdint: the native_sim test build links Zephyr's
// minimal libc++, which ships no <c*> wrapper headers.
#include <stdint.h>

// Pure logic, no Zephyr dependencies: the wire discipline the crossed-pin
// diagnostic depends on (open-drain only, bit order, START/STOP framing,
// clock stretching, stuck-bus detection) is verified on native_sim in
// tests/tof_diag before the firmware ever runs on a robot.

namespace lexxhard::tof_diag {

// Open-drain line operations. The interface deliberately has no "drive high":
// a line is either actively pulled low or released to the external pull-up,
// so driving push-pull high against a transmitting slave is unrepresentable
// by construction.
struct pin_ops {
    virtual void sda_drive_low() = 0;
    virtual void sda_release() = 0;
    virtual bool sda_read() = 0;
    virtual void scl_drive_low() = 0;
    virtual void scl_release() = 0;
    virtual bool scl_read() = 0;
    virtual void delay_half_bit() = 0;
    virtual ~pin_ops() = default;
};

enum class probe_result : uint8_t {
    ACK,           // address byte acknowledged
    NACK,          // clean not-acknowledge on the ninth clock
    BUS_BUSY,      // SDA low before START: bus not idle
    SCL_STUCK_LOW, // SCL never returned high within the stretch budget
};

const char *to_string(probe_result result);

class master {
public:
    explicit master(pin_ops &ops) : ops(ops) {}
    // START, addr7 << 1 (write direction), sample ACK, STOP.
    probe_result probe(uint8_t addr7);
    // Nine clock pulses with SDA released, then STOP: the standard recovery
    // for a slave left mid-transfer holding SDA low.
    void bus_clear();
private:
    // Half-bit periods to wait on a stretched SCL before declaring it stuck.
    static constexpr int stretch_budget{64};
    bool wait_scl_high();
    bool clock_bit_out(bool bit);
    int clock_bit_in();
    void stop();
    pin_ops &ops;
};

// Classified counters for repeated probe/transfer attempts. The counting
// rules are pure so they are host-testable; the errno values come from
// whichever i2c path the caller used (hardware driver or bit-bang glue).
struct stress_stats {
    uint32_t attempts{0};
    uint32_t ok{0};
    uint32_t nack_or_io{0};    // -EIO: NACK on the STM32 driver, or a generic bus error
    uint32_t timeout{0};       // -ETIMEDOUT
    uint32_t busy{0};          // -EBUSY
    uint32_t other_error{0};
    uint32_t data_mismatch{0}; // successful reads whose payload differs from the reference

    void count(int err);
    bool all_ok() const { return attempts == ok && data_mismatch == 0; }
};

}
