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

#include <zephyr/ztest.h>

#include "tof_diag_bitbang.hpp"

using namespace lexxhard::tof_diag;

namespace {

// Simulated open-drain bus with one address-decoding slave. A line is high
// only when neither the master nor the slave pulls it low, so the master's
// wire discipline (never driving high, SDA changes only while SCL is low)
// is exercised against the same physics as the real bus. The slave FSM
// advances on SCL/SDA edges, exactly as a hardware slave samples them.
class sim_bus : public pin_ops {
public:
    explicit sim_bus(uint8_t match_addr7) : match(match_addr7) {}

    void sda_drive_low() override { master_sda_low = true; update(); }
    void sda_release() override { master_sda_low = false; update(); }
    bool sda_read() override { return sda_level(); }
    void scl_drive_low() override { master_scl_low = true; update(); }
    void scl_release() override { master_scl_low = false; update(); }
    bool scl_read() override { return scl_level(); }
    void delay_half_bit() override {
        if (stretch_pending > 0 && --stretch_pending == 0 && !clamped) {
            slave_scl_low = false;
            update();
        }
    }

    // Fault injection. Previous levels are synchronised so the injected
    // state is a precondition, not an edge the FSM would misread as START.
    void hold_sda_low() { slave_sda_low = true; prev_sda = false; }
    void clamp_scl() { slave_scl_low = true; clamped = true; prev_scl = false; }
    void stretch_after_start(int half_bits) { stretch_arm = half_bits; }

    // Observations.
    uint8_t address_byte() const { return shift; }
    bool saw_start{false};
    bool saw_stop{false};
    bool acked{false};
    int scl_rises{0};

private:
    bool scl_level() const { return !(master_scl_low || slave_scl_low); }
    bool sda_level() const { return !(master_sda_low || slave_sda_low); }

    void update() {
        bool const scl{scl_level()};
        bool const sda{sda_level()};
        if (!prev_scl && scl)
            ++scl_rises;
        // START: SDA falls while SCL is high.
        if (prev_scl && scl && prev_sda && !sda) {
            saw_start = true;
            in_frame = true;
            bit_count = 0;
            shift = 0;
            if (stretch_arm > 0 && !clamped) {
                slave_scl_low = true;
                stretch_pending = stretch_arm;
                stretch_arm = 0;
            }
        }
        // STOP: SDA rises while SCL is high.
        if (prev_scl && scl && !prev_sda && sda)
            saw_stop = true;
        // Rising SCL edge: sample one address bit. Bits advance on rising
        // edges only -- the falling edge right after START carries no data.
        if (!prev_scl && scl && in_frame && bit_count < 8) {
            shift = static_cast<uint8_t>((shift << 1) | (sda ? 1 : 0));
            ++bit_count;
        }
        // Falling SCL edge after the eighth bit: drive ACK for the ninth
        // clock; the falling edge after that releases SDA and ends the frame.
        if (prev_scl && !scl && in_frame && bit_count == 8) {
            if (!ack_phase) {
                ack_phase = true;
                if ((shift >> 1) == match) {
                    slave_sda_low = true;
                    acked = true;
                }
            } else {
                slave_sda_low = false;
                ack_phase = false;
                in_frame = false;
            }
        }
        prev_scl = scl;
        prev_sda = sda;
    }

    uint8_t const match;
    bool master_scl_low{false}, master_sda_low{false};
    bool slave_scl_low{false}, slave_sda_low{false};
    bool prev_scl{true}, prev_sda{true};
    bool in_frame{false}, clamped{false}, ack_phase{false};
    int bit_count{0}, stretch_pending{0}, stretch_arm{0};
    uint8_t shift{0};
};

// Adapter presenting a bus with SDA and SCL swapped: what the master sees as
// SCL is physically the slave's SDA and vice versa. This is the crossed-pin
// wiring the whole diagnostic exists to detect.
class crossed_bus : public pin_ops {
public:
    explicit crossed_bus(pin_ops &inner) : inner(inner) {}
    void sda_drive_low() override { inner.scl_drive_low(); }
    void sda_release() override { inner.scl_release(); }
    bool sda_read() override { return inner.scl_read(); }
    void scl_drive_low() override { inner.sda_drive_low(); }
    void scl_release() override { inner.sda_release(); }
    bool scl_read() override { return inner.sda_read(); }
    void delay_half_bit() override { inner.delay_half_bit(); }
private:
    pin_ops &inner;
};

}

ZTEST_SUITE(tof_diag_bitbang, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_diag_bitbang, test_ack_and_wire_format)
{
    sim_bus bus(0x29); // VL53 default 7-bit address
    master m(bus);
    zassert_equal(m.probe(0x29), probe_result::ACK);
    // The slave saw a well-formed frame: START, the 8-bit address byte in
    // write direction (0x29 << 1 = 0x52, i.e. the ST-documented 8-bit form),
    // and a STOP.
    zassert_true(bus.saw_start);
    zassert_true(bus.acked);
    zassert_equal(bus.address_byte(), 0x52);
    zassert_true(bus.saw_stop);
}

ZTEST(tof_diag_bitbang, test_nack_still_stops)
{
    sim_bus bus(0x29);
    master m(bus);
    zassert_equal(m.probe(0x2a), probe_result::NACK);
    zassert_false(bus.acked);
    zassert_equal(bus.address_byte(), 0x54);
    // A NACKed probe must still close the frame, or the bus is left busy
    // for every probe that follows (a scan would report one NACK and then
    // nothing but BUS_BUSY).
    zassert_true(bus.saw_stop);
}

ZTEST(tof_diag_bitbang, test_bus_busy_detected_before_start)
{
    sim_bus bus(0x29);
    bus.hold_sda_low();
    master m(bus);
    zassert_equal(m.probe(0x29), probe_result::BUS_BUSY);
    zassert_false(bus.saw_start);
}

ZTEST(tof_diag_bitbang, test_scl_clamped_low_detected)
{
    sim_bus bus(0x29);
    bus.clamp_scl();
    master m(bus);
    zassert_equal(m.probe(0x29), probe_result::SCL_STUCK_LOW);
    zassert_false(bus.saw_start);
}

ZTEST(tof_diag_bitbang, test_clock_stretch_within_budget_still_acks)
{
    sim_bus bus(0x29);
    bus.stretch_after_start(8); // released well inside the 64 half-bit budget
    master m(bus);
    zassert_equal(m.probe(0x29), probe_result::ACK);
}

ZTEST(tof_diag_bitbang, test_clock_stretch_beyond_budget_reports_stuck)
{
    sim_bus bus(0x29);
    bus.stretch_after_start(1000); // never released within the budget
    master m(bus);
    zassert_equal(m.probe(0x29), probe_result::SCL_STUCK_LOW);
}

// The discrimination property the crossed-pin diagnosis rests on: against
// the same slave, the correct orientation ACKs and the swapped orientation
// cannot complete an ACKed address frame. Note that spurious STARTs are
// expected under crossing -- the master's clock toggles what the slave sees
// as SDA while the slave's SCL line sits high -- so `saw_start` proves
// nothing either way. What cannot happen is an ACK: the master changes data
// only while its own SCL is low, so every bit the confused slave samples at
// its SDA-derived clock edges reads as the idle level, and the address
// never assembles.
ZTEST(tof_diag_bitbang, test_crossed_wiring_cannot_ack)
{
    sim_bus bus(0x29);
    crossed_bus crossed(bus);
    master m(crossed);
    probe_result const result{m.probe(0x29)};
    zassert_not_equal(result, probe_result::ACK);
    zassert_false(bus.acked);
}

ZTEST(tof_diag_bitbang, test_bus_clear_pulses_and_stops)
{
    sim_bus bus(0x29);
    master m(bus);
    zassert_true(m.bus_clear());
    zassert_true(bus.scl_rises >= 9);
    zassert_true(bus.saw_stop);
}

// A clamped clock cannot be cleared by clocking; reporting success there
// would send the operator down the wrong path (P1 from review).
ZTEST(tof_diag_bitbang, test_bus_clear_fails_when_scl_clamped)
{
    sim_bus bus(0x29);
    bus.clamp_scl();
    master m(bus);
    zassert_false(m.bus_clear());
}

// Failures are split by duration, not errno: on the STM32 driver a NACK, a
// bus error and the controller timeout all return -EIO, so errno carries no
// diagnostic detail on the hardware path.
ZTEST(tof_diag_bitbang, test_stress_stats_duration_split)
{
    stress_stats stats(10000); // 10 ms threshold
    stats.count(0, 200);
    stats.count(-EIO, 150);     // NACK-like: fails within a bit time
    stats.count(-EIO, 120000);  // timeout-like: fails only after the deadline
    stats.count(-EIO, 10000);   // exactly at the threshold counts as slow
    zassert_equal(stats.attempts, 4u);
    zassert_equal(stats.ok, 1u);
    zassert_equal(stats.failed_fast, 1u);
    zassert_equal(stats.failed_slow, 2u);
    zassert_equal(stats.last_error, -EIO);
    zassert_equal(stats.worst_elapsed_us, 120000u);
    zassert_false(stats.all_ok());

    stress_stats clean;
    clean.count(0, 300);
    clean.count(0, 400);
    zassert_true(clean.all_ok());
    clean.data_mismatch = 1;
    zassert_false(clean.all_ok());
}
