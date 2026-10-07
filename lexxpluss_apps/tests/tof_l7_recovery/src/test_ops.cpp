/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
#include "vl53l7cx_port.h"

namespace
{

namespace rec = lexxhard::tof_l7_recovery;

int sticky_port_error{0};
uint32_t port_completed_{0};

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
    /* What the port counted DURING the call. Scripted with the error for the same reason: the
     * adapter clears the port before every ULD call, so a count set beforehand would be wiped. */
    uint32_t alive_completed{0};
    uint32_t stop_completed{0};
    int alive_calls{0};
    int stop_calls{0};
};

uld_spy spy_{};

void reset_spy()
{
    spy_ = uld_spy{};
    sticky_port_error = 0;
    port_completed_ = 0;
}

}  // namespace

extern "C" {

/* The stubs record the context they were given, so a build that went back to one process-global
 * record would fail here rather than quietly passing every behavioural test below. */
const vl53l7cx_port_context *last_port_ctx_{nullptr};

void vl53l7cx_port_clear_error(vl53l7cx_port_context *port)
{
    last_port_ctx_ = port;
    sticky_port_error = 0;
    port_completed_ = 0;
}
int vl53l7cx_port_error(const vl53l7cx_port_context *port)
{
    last_port_ctx_ = port;
    return sticky_port_error;
}
uint32_t vl53l7cx_port_completed_transfers(const vl53l7cx_port_context *port)
{
    last_port_ctx_ = port;
    return port_completed_;
}
int sticky_argument_fault_{0};
int vl53l7cx_port_argument_fault(void) { return sticky_argument_fault_; }
void vl53l7cx_port_clear_argument_fault(void) { sticky_argument_fault_ = 0; }

uint8_t vl53l7cx_is_alive(VL53L7CX_Configuration *p_dev, uint8_t *p_is_alive)
{
    ++spy_.alive_calls;
    spy_.alive_address = p_dev->platform.address;
    spy_.alive_saw_auto_stop = p_dev->is_auto_stop_enabled;
    sticky_port_error = spy_.alive_port_error;
    port_completed_ = spy_.alive_completed;
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
    port_completed_ = spy_.stop_completed;
    return spy_.stop_status;
}

}  // extern "C"

namespace
{

lexxhard::tof_l7::sensor scratch_{};

/* A non-null stand-in. This suite never reaches a driver, but the probe refuses a context that
 * names no controller, so the pointer has to exist and be distinguishable. */
const device test_bus_{};
vl53l7cx_port_context ctx_{};

rec::ops ops_under_test()
{
    /* Set here rather than once at startup: several tests reset `scratch_` wholesale, and binding
     * is the single place that installs the context on it. */
    ctx_.bus = &test_bus_;
    return rec::uld_ops(&scratch_, &ctx_);
}

}  // namespace

ZTEST_SUITE(tof_l7_recovery_ops, nullptr, nullptr, nullptr, nullptr, nullptr);

/* The cold boot. Nobody is at the address, the driver says so cleanly, and the adapter reports a
 * completed probe rather than a failure -- which is what keeps the pass free on nearly every boot. */
ZTEST(tof_l7_recovery_ops, test_a_clean_nack_is_a_completed_probe_with_nobody_there)
{
    reset_spy();
    spy_.alive_port_error = -ENXIO;
    spy_.alive_answer = 0;
    spy_.alive_completed = 0;   /* nothing completed a transaction: the cold boot */
    bool alive{true};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), 0, "a NACK is an answer, not an error");
    zassert_false(alive);
}

/* THE CASE THE CASE ABOVE USED TO HIDE, and the reason its `alive_answer` changed from 1 to 0.
 *
 * It used to script a NACK together with an answer and assert absent, on the reading that a NACK
 * means nobody answered. Those two facts cannot both be true, and the combination is reachable:
 * the port keeps only the FIRST error, and vl53l7cx_is_alive() issues all four transfers with no
 * early return, so a survivor whose page-select write NACKs while its ID reads return 0xF0 and 0x02
 * produces exactly this. Reporting it absent lost the sensor and recorded no failure -- and since
 * this pass's census is evidence, a false absent argues for the wrong conclusion about warm resets.
 */
ZTEST(tof_l7_recovery_ops, test_a_nack_with_a_completed_transfer_is_a_failed_probe)
{
    /* TWO SHAPES OF THE SAME PARTIAL EXCHANGE, and the second is why this is asked of the transport
     * rather than of the identity. The ULD's alive flag is set only when BOTH identity bytes match,
     * so a device that answers the device-id read with 0xF0 and NACKs the revision-id read leaves
     * it at ZERO -- and a classification keyed on that flag files the survivor as an empty address
     * all over again. Only the port can say that something completed. */
    const uint8_t identity_flag[]{1U, 0U};

    for (const uint8_t answered : identity_flag) {
        reset_spy();
        scratch_ = lexxhard::tof_l7::sensor{};
        spy_.alive_port_error = -ENXIO;
        spy_.alive_answer = answered;
        spy_.alive_completed = 2U;   /* the page select and one id read got through */
        bool alive{true};

        const rec::ops o{ops_under_test()};
        const int rc{o.is_alive(o.ctx, 0x2AU, &alive)};

        zassert_equal(rc, -EIO,
                      "a partial exchange with alive flag %u was reported %d", answered, rc);
        zassert_false(alive, "the out-parameter stays false on every refusal");
    }
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
 * acts on an identity that does not belong.
 *
 * THE SHAPE OF THIS CASE WAS WRONG, AND THAT WAS THE DEFECT. It used to drive a non-OK ULD status
 * with the answer flag set, which is not how vl53l7cx_is_alive() reports a foreign device: it
 * compares the device and revision ids, sets its out-parameter to zero when they do not match, and
 * returns OK. A wrong identity therefore looks like THIS. */
ZTEST(tof_l7_recovery_ops, test_an_ack_from_a_foreign_device_is_not_a_survivor)
{
    reset_spy();
    spy_.alive_status = VL53L7CX_STATUS_OK;
    spy_.alive_answer = 0;
    bool alive{true};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), 0);
    zassert_false(alive, "identified as something else, so not something this pass may stop");
}

/* AND A NON-OK STATUS IS A FAILED PROBE, which is the case the one above used to occupy.
 *
 * vl53l7cx_is_alive()'s status can only become non-OK from one of its four platform calls, and
 * every one of those sets the port's sticky errno -- which this adapter checks first. So a non-OK
 * status with no transport error means the port said every transfer succeeded and the ULD still
 * refused. That is an anomaly, and reporting it as an empty address would file it as the most
 * ordinary observation a cold boot makes. */
ZTEST(tof_l7_recovery_ops, test_a_uld_failure_without_a_transport_error_is_a_failed_probe)
{
    reset_spy();
    spy_.alive_status = VL53L7CX_STATUS_ERROR;
    spy_.alive_answer = 0;
    spy_.alive_port_error = 0;
    bool alive{true};

    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), -EIO,
                  "the transport was fine and the ULD refused: that is not an empty address");
    zassert_false(alive, "and nothing may be concluded about what is there");
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
    const rec::ops o{rec::uld_ops(nullptr, &ctx_)};
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

/* ---- the scratch must not be a sensor somebody is using ---- */

/* WHY BOTH ENTRY POINTS REFUSE RATHER THAN COPE. addressed() zeroes the whole ULD configuration,
 * the firmware pointer included, and rewrites the address. Done to a sensor that is open, running or
 * carrying a stop_unconfirmed debt, that leaves `current` saying one thing and the configuration
 * describing a different device -- and the next stop() or read_once() would talk to that device
 * through an empty configuration. The header always required an unopened scratch; only the null
 * pointer was checked.
 *
 * Asserted on the TRAFFIC as well as the return value: a refusal that still issued a transfer would
 * have already touched the wrong address by the time the caller saw the error. */
ZTEST(tof_l7_recovery_ops, test_a_scratch_that_is_in_use_is_refused_and_no_traffic_is_issued)
{
    const lexxhard::tof_l7::lifecycle in_use[]{
        lexxhard::tof_l7::lifecycle::opened,
        lexxhard::tof_l7::lifecycle::configured,
        lexxhard::tof_l7::lifecycle::running,
        lexxhard::tof_l7::lifecycle::stop_unconfirmed,
    };

    for (const auto state : in_use) {
        reset_spy();
        scratch_ = lexxhard::tof_l7::sensor{};
        scratch_.current = state;
        /* Something recognisable in the configuration, so a wipe is visible. */
        scratch_.uld.platform.address = 0x52U;
        scratch_.uld.platform.firmware = reinterpret_cast<const uint8_t *>(&spy_);

        bool alive{true};
        const rec::ops o{ops_under_test()};

        zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), -EINVAL,
                      "probe accepted a scratch in state %d", static_cast<int>(state));
        zassert_false(alive, "and it must not claim the address is alive");
        zassert_equal(o.stop_ranging(o.ctx, 0x2AU), -EINVAL,
                      "stop accepted a scratch in state %d", static_cast<int>(state));

        zassert_equal(spy_.alive_calls, 0, "the ULD was asked anyway");
        zassert_equal(spy_.stop_calls, 0, "the ULD was asked anyway");
        zassert_equal(scratch_.uld.platform.address, 0x52U,
                      "the configuration was wiped before the refusal");
        zassert_not_null(scratch_.uld.platform.firmware, "the firmware pointer was wiped");
        zassert_equal(scratch_.current, state, "and the lifecycle was not touched");
    }

    scratch_ = lexxhard::tof_l7::sensor{};
}

/* And the ordinary case still works, so the check is not just refusing everything. `empty` is the
 * one state the pass accepts, and tof_l7::close() is how production gets a used sensor back to it;
 * the state is set directly here because this suite deliberately does not link tof_l7_sensor.cpp --
 * it fakes two ULD entry points, and linking the sensor would drag in the rest of the ULD to fake
 * as well. What is under test is the adapter's precondition, not close(). */
ZTEST(tof_l7_recovery_ops, test_an_empty_scratch_is_accepted)
{
    reset_spy();
    scratch_ = lexxhard::tof_l7::sensor{};
    zassert_equal(scratch_.current, lexxhard::tof_l7::lifecycle::empty, "");

    bool alive{false};
    spy_.alive_answer = 1;
    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x2AU, &alive), 0, "");
    zassert_true(alive, "");
    zassert_equal(spy_.alive_calls, 1, "");
    zassert_equal(spy_.alive_address, 0x2AU << 1, "and it was addressed for this probe");

    scratch_ = lexxhard::tof_l7::sensor{};
}

/* ---- the probe talks through the context it was bound with -------------------------------- */

/* The recovery pass must use ITS OWN context: a second bus means a second scratch sensor with a
 * second context, and the two must not share an errno or a completed-transfer count. Nothing else
 * in this file would notice a return to one process-global record. */
ZTEST(tof_l7_recovery_ops, test_the_probe_uses_the_context_it_was_bound_with)
{
    reset_spy();
    scratch_ = lexxhard::tof_l7::sensor{};
    last_port_ctx_ = nullptr;

    bool alive{false};
    const rec::ops o{ops_under_test()};
    zassert_ok(o.is_alive(o.ctx, 0x29U, &alive));

    zassert_equal(last_port_ctx_, &ctx_, "the port was asked about the bound context");
    zassert_equal(scratch_.uld.platform.port, &ctx_,
                  "and the zeroing inside the probe did not drop it");
}

/* A scratch sensor whose context names no controller is refused like any other bad argument, and
 * nothing is driven. Without the check the probe would reach the port, be told -ENODEV, and that
 * reads exactly like a cold address -- the conclusion this pass exists not to reach by accident. */
ZTEST(tof_l7_recovery_ops, test_a_context_without_a_bus_refuses_the_probe)
{
    reset_spy();
    scratch_ = lexxhard::tof_l7::sensor{};
    const rec::ops o{ops_under_test()};
    ctx_.bus = nullptr;

    bool alive{true};
    zassert_equal(o.is_alive(o.ctx, 0x29U, &alive), -EINVAL);
    zassert_equal(spy_.alive_calls, 0, "the ULD was not asked");

    ctx_.bus = &test_bus_;
}

/* BINDING MUST NOT TOUCH A SENSOR IT IS NOT ALLOWED TO USE. Installing the context first and
 * leaning on the call-time lifecycle refusal would already have repointed a live session's bus by
 * the time the probe was turned away: refused call, re-bussed sensor. */
ZTEST(tof_l7_recovery_ops, test_binding_refuses_a_live_sensor_without_modifying_it)
{
    reset_spy();
    scratch_ = lexxhard::tof_l7::sensor{};

    vl53l7cx_port_context original{};
    original.bus = &test_bus_;
    scratch_.uld.platform.port = &original;
    scratch_.current = lexxhard::tof_l7::lifecycle::running;

    vl53l7cx_port_context intruder{};
    intruder.bus = &test_bus_;
    const rec::ops o{rec::uld_ops(&scratch_, &intruder)};

    zassert_equal(scratch_.uld.platform.port, &original,
                  "the live session still points at the context it was opened with");

    bool alive{true};
    zassert_equal(o.is_alive(o.ctx, 0x29U, &alive), -EINVAL, "and the probe is refused");
    zassert_equal(spy_.alive_calls, 0, "with nothing issued");
    zassert_equal(scratch_.uld.platform.port, &original, "still unmodified after the refusal");

    scratch_ = lexxhard::tof_l7::sensor{};
}

/* An argument fault outranks the cold-boot classification: "nobody there" is the worst answer to
 * give when the reason nothing was heard is that this build called the port wrongly. */
ZTEST(tof_l7_recovery_ops, test_an_argument_fault_is_not_reported_as_an_empty_address)
{
    reset_spy();
    scratch_ = lexxhard::tof_l7::sensor{};
    spy_.alive_port_error = -ENXIO;
    spy_.alive_completed = 0U;
    sticky_argument_fault_ = -EINVAL;

    bool alive{true};
    const rec::ops o{ops_under_test()};
    zassert_equal(o.is_alive(o.ctx, 0x29U, &alive), -EINVAL,
                  "not 0, which is how this pass reports a cold address");

    sticky_argument_fault_ = 0;
}

/* THE STOP PATH OBEYS THE SAME ORDER. A stop that returns success is what the enumeration above
 * acts on, so an image-wide fault must not be reportable as a completed stop -- and this path is
 * the easiest of the three to leave out, because it already has a transport errno and a status
 * byte of its own to report. */
ZTEST(tof_l7_recovery_ops, test_an_argument_fault_is_not_reported_as_a_completed_stop)
{
    reset_spy();
    scratch_ = lexxhard::tof_l7::sensor{};
    spy_.stop_status = VL53L7CX_STATUS_OK;   /* the device would have said it stopped */
    sticky_argument_fault_ = -EINVAL;

    const rec::ops o{ops_under_test()};
    zassert_equal(o.stop_ranging(o.ctx, 0x29U), -EINVAL,
                  "not 0, which the enumeration would read as a stopped survivor");

    sticky_argument_fault_ = 0;
}
