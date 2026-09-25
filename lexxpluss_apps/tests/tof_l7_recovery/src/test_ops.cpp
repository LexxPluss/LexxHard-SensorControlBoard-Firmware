/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The adapter that turns the recovery pass's two calls into I2C traffic, against fake ULD and port
 * symbols. The pure pass is tested next door; what is at stake here is the translation, and two
 * pieces of it decide whether the pass works on hardware at all.
 *
 * THE ERROR MAPPING. The backported STM32 I2C driver keeps patch 0001's classification, so a clean
 * NACK arrives as -ENXIO and a broken transport as -EIO or -ETIMEDOUT. The adapter has to keep them
 * apart: a NACK is a completed probe with nobody at the address, which is what every address gives
 * on a cold boot, while a transport error is a fault that must not be read as an empty chain. Fold
 * them together either way and the pass either calls every cold boot a failure or calls a dead bus
 * an empty one.
 *
 * THE ZEROED CONFIGURATION. stop_ranging only takes its provoke-MCU-stop path when
 * is_auto_stop_enabled is zero, and that path is the one that can end a session started by an
 * instance of this firmware that no longer exists. So the zeroing is load-bearing, not tidiness,
 * and it is pinned here by watching what the ULD actually receives.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_l7_recovery.hpp"
#include "tof_l7_recovery_ops.hpp"
#include "tof_l7_sensor.hpp"

namespace
{

namespace rec = lexxhard::tof_l7_recovery;

int sticky_port_error{0};

struct uld_spy {
    uint16_t alive_address{0xFFFFU};
    uint16_t stop_address{0xFFFFU};
    uint8_t alive_answer{0};
    uint8_t alive_status{VL53L7CX_STATUS_OK};
    uint8_t stop_status{VL53L7CX_STATUS_OK};
    uint8_t alive_saw_auto_stop{0xFFU};
    uint8_t stop_saw_auto_stop{0xFFU};
    /* What the port records DURING the call, which is when a real transfer fails. Scripted here
     * rather than assigned to sticky_port_error by the test, because the adapter clears the port
     * before every ULD call -- and an error the test set beforehand would be wiped by exactly the
     * clear that has to happen. Modelling the order is what makes these cases mean anything. */
    int alive_port_error{0};
    int stop_port_error{0};
    int alive_calls{0};
    int stop_calls{0};
};

uld_spy spy_{};

void reset_spy()
{
    spy_ = uld_spy{};
    sticky_port_error = 0;
}

}  // namespace

extern "C" {

void vl53l7cx_port_clear_error(void) { sticky_port_error = 0; }
int vl53l7cx_port_error(void) { return sticky_port_error; }

uint8_t vl53l7cx_is_alive(VL53L7CX_Configuration *p_dev, uint8_t *p_is_alive)
{
    ++spy_.alive_calls;
    spy_.alive_address = p_dev->platform.address;
    spy_.alive_saw_auto_stop = p_dev->is_auto_stop_enabled;
    sticky_port_error = spy_.alive_port_error;
    if (p_is_alive != nullptr)
        *p_is_alive = spy_.alive_answer;
    return spy_.alive_status;
}

uint8_t vl53l7cx_stop_ranging(VL53L7CX_Configuration *p_dev)
{
    ++spy_.stop_calls;
    spy_.stop_address = p_dev->platform.address;
    spy_.stop_saw_auto_stop = p_dev->is_auto_stop_enabled;
    sticky_port_error = spy_.stop_port_error;
    return spy_.stop_status;
}

}  // extern "C"

namespace
{

lexxhard::tof_l7::sensor scratch_{};

rec::ops ops_under_test()
{
    return rec::uld_ops(&scratch_);
}

}  // namespace

ZTEST_SUITE(tof_l7_recovery_ops, nullptr, nullptr, nullptr, nullptr, nullptr);

/* The cold boot. Nobody is at the address, the driver says so cleanly, and the adapter reports a
 * completed probe rather than a failure -- which is what keeps the pass free on nearly every boot. */
ZTEST(tof_l7_recovery_ops, test_a_clean_nack_is_a_completed_probe_with_nobody_there)
{
    reset_spy();
    spy_.alive_port_error = -ENXIO;
    spy_.alive_answer = 1;   /* even if the ULD would claim one, a NACK means nobody answered */
    bool alive{true};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), 0, "a NACK is an answer, not an error");
    zassert_false(alive);
}

/* A bus that could not carry the question. Reporting this as "nobody there" would be the
 * comfortable reading and would hide a broken chain behind a clean-looking boot. */
ZTEST(tof_l7_recovery_ops, test_a_transport_error_is_passed_through_and_not_flattened_to_absent)
{
    reset_spy();
    spy_.alive_port_error = -EIO;
    bool alive{true};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), -EIO);
    zassert_false(alive);

    reset_spy();
    spy_.alive_port_error = -ETIMEDOUT;
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), -ETIMEDOUT,
                  "a timeout is its own errno and must survive the adapter");
}

/* The survivor: it answers, it identifies as an L7, and the address it was asked at is the 7-bit
 * one shifted up, which is the only form the ULD platform understands. */
ZTEST(tof_l7_recovery_ops, test_a_live_sensor_is_probed_at_the_shifted_address)
{
    reset_spy();
    spy_.alive_answer = 1;
    bool alive{false};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2BU, &alive), 0);
    zassert_true(alive);
    zassert_equal(spy_.alive_address, static_cast<uint16_t>(0x2BU << 1));
}

/* An ACK from something that is not an L7. This pass stops L7 sessions, and it has no business
 * sending a five-second stop sequence to whatever else answered; the census in enumeration is what
 * acts on an identity that does not belong. */
ZTEST(tof_l7_recovery_ops, test_an_ack_from_a_foreign_device_is_not_a_survivor)
{
    reset_spy();
    spy_.alive_status = VL53L7CX_STATUS_ERROR;
    spy_.alive_answer = 1;
    bool alive{true};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), 0);
    zassert_false(alive, "identified as something else, so not something this pass may stop");
}

/* The precondition the whole recovery rests on. A zeroed configuration carries
 * is_auto_stop_enabled == 0, which is what sends stop_ranging down its provoke-MCU-stop path --
 * the path that works on a session this firmware never started. */
ZTEST(tof_l7_recovery_ops, test_the_uld_is_handed_a_zeroed_configuration_every_time)
{
    reset_spy();
    scratch_.uld.is_auto_stop_enabled = 1;
    scratch_.uld.platform.address = 0x99U;

    const rec::ops o{ops_under_test()};
    bool alive{false};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), 0);
    zassert_equal(spy_.alive_saw_auto_stop, 0U, "a stale auto-stop flag would take the wrong path");

    scratch_.uld.is_auto_stop_enabled = 1;
    zassert_equal(o.stop_ranging(o.ctx, 0x2AU), 0);
    zassert_equal(spy_.stop_saw_auto_stop, 0U);
    zassert_equal(spy_.stop_address, static_cast<uint16_t>(0x2AU << 1),
                  "the stop re-addresses rather than trusting the probe that came before it");
}

/* The ULD folds its own give-up into a status byte with no transport error behind it: five seconds
 * of polling that never saw the MCU stop. It is a failure of the stop, and the caller records it. */
ZTEST(tof_l7_recovery_ops, test_a_uld_level_stop_failure_is_reported_without_a_transport_error)
{
    reset_spy();
    spy_.stop_status = VL53L7CX_STATUS_ERROR;

    const rec::ops o{ops_under_test()};
    zassert_equal(o.stop_ranging(o.ctx, 0x2AU), -EIO);
    zassert_equal(sticky_port_error, 0, "nothing was wrong with the bus; the device did not stop");
}

/* A transport error during the stop keeps its own errno rather than being flattened into the
 * ULD's byte, because which one it was is the difference between a busy sensor and a dead bus. */
ZTEST(tof_l7_recovery_ops, test_a_transport_error_during_the_stop_keeps_its_errno)
{
    reset_spy();
    spy_.stop_port_error = -ETIMEDOUT;
    spy_.stop_status = VL53L7CX_STATUS_ERROR;

    const rec::ops o{ops_under_test()};
    zassert_equal(o.stop_ranging(o.ctx, 0x2AU), -ETIMEDOUT);
}

/* No scratch, no bus traffic. The object is over a kilobyte and the caller owns it, so a missing
 * one is a wiring defect that must fail loudly and touch nothing. */
ZTEST(tof_l7_recovery_ops, test_a_missing_scratch_refuses_both_calls_and_issues_nothing)
{
    reset_spy();
    const rec::ops o{rec::uld_ops(nullptr)};
    bool alive{true};

    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), -EINVAL);
    zassert_equal(o.stop_ranging(o.ctx, 0x2AU), -EINVAL);
    zassert_equal(spy_.alive_calls, 0);
    zassert_equal(spy_.stop_calls, 0);
}

/* The two halves together, on the boot this module exists for: both sensors survived, and the pass
 * driving the real adapter stops each of them once at its own address. */
ZTEST(tof_l7_recovery_ops, test_the_pass_and_the_adapter_together_stop_both_survivors)
{
    reset_spy();
    spy_.alive_answer = 1;

    rec::request q{};
    q.addr_7bit[0] = 0x2AU;
    q.addr_7bit[1] = 0x2BU;
    q.count = 2;

    const rec::report r{rec::run(ops_under_test(), q)};

    zassert_equal(r.stopped, 2U);
    zassert_false(r.any_failure);
    zassert_equal(spy_.alive_calls, 2);
    zassert_equal(spy_.stop_calls, 2);
    zassert_equal(spy_.stop_address, static_cast<uint16_t>(0x2BU << 1), "the last stop was source 1");
}
