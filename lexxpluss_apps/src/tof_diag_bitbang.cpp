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

#include "tof_diag_bitbang.hpp"

namespace lexxhard::tof_diag {

const char *to_string(probe_result result)
{
    switch (result) {
    case probe_result::ACK:           return "ACK";
    case probe_result::NACK:          return "NACK";
    case probe_result::BUS_BUSY:      return "BUS_BUSY";
    case probe_result::SCL_STUCK_LOW: return "SCL_STUCK_LOW";
    }
    return "?";
}

bool master::wait_scl_high()
{
    for (int i{0}; i < stretch_budget; ++i) {
        if (ops.scl_read())
            return true;
        ops.delay_half_bit();
    }
    return false;
}

bool master::clock_bit_out(bool bit)
{
    // SDA changes only while SCL is held low; the level must be stable
    // before SCL is released and for the whole high phase.
    if (bit)
        ops.sda_release();
    else
        ops.sda_drive_low();
    ops.delay_half_bit();
    ops.scl_release();
    if (!wait_scl_high())
        return false;
    ops.delay_half_bit();
    ops.scl_drive_low();
    return true;
}

int master::clock_bit_in()
{
    ops.sda_release();
    ops.delay_half_bit();
    ops.scl_release();
    if (!wait_scl_high())
        return -1;
    int const bit{ops.sda_read() ? 1 : 0};
    ops.delay_half_bit();
    ops.scl_drive_low();
    return bit;
}

void master::stop()
{
    // STOP: SDA rises while SCL is high. Entered with SCL low.
    ops.sda_drive_low();
    ops.delay_half_bit();
    ops.scl_release();
    (void)wait_scl_high();
    ops.delay_half_bit();
    ops.sda_release();
    ops.delay_half_bit();
}

probe_result master::probe(uint8_t addr7)
{
    ops.scl_release();
    ops.sda_release();
    ops.delay_half_bit();
    // A stuck SCL is reported before BUS_BUSY because it is the more
    // specific fault: with SCL clamped, SDA state proves nothing.
    if (!ops.scl_read())
        return probe_result::SCL_STUCK_LOW;
    if (!ops.sda_read())
        return probe_result::BUS_BUSY;
    // START: SDA falls while SCL is high.
    ops.sda_drive_low();
    ops.delay_half_bit();
    ops.scl_drive_low();
    uint8_t const byte(addr7 << 1); // write direction
    for (int i{7}; i >= 0; --i) {
        if (!clock_bit_out((byte >> i) & 1)) {
            ops.sda_release();
            return probe_result::SCL_STUCK_LOW;
        }
    }
    int const ack{clock_bit_in()};
    if (ack < 0) {
        ops.sda_release();
        return probe_result::SCL_STUCK_LOW;
    }
    stop();
    return ack == 0 ? probe_result::ACK : probe_result::NACK;
}

void master::bus_clear()
{
    ops.sda_release();
    for (int i{0}; i < 9; ++i) {
        ops.scl_drive_low();
        ops.delay_half_bit();
        ops.scl_release();
        (void)wait_scl_high();
        ops.delay_half_bit();
    }
    // STOP in case the slave released SDA during the pulses.
    ops.scl_drive_low();
    ops.sda_drive_low();
    ops.delay_half_bit();
    ops.scl_release();
    (void)wait_scl_high();
    ops.delay_half_bit();
    ops.sda_release();
}

void stress_stats::count(int err)
{
    ++attempts;
    if (err == 0)
        ++ok;
    else if (err == -EIO)
        ++nack_or_io;
    else if (err == -ETIMEDOUT)
        ++timeout;
    else if (err == -EBUSY)
        ++busy;
    else
        ++other_error;
}

}
