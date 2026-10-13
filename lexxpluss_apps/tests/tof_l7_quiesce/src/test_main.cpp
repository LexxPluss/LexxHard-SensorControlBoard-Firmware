/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * What a quiesce told us, and that the record of it survives.
 *
 * The run this exists for printed `-5` and nothing else, so the failure could not be placed in a
 * layer. These cases pin the two things that fixes: the ordering that makes each layer mean what it
 * is named, and that the record is complete and not rewritable by whatever runs next.
 */

#include <zephyr/ztest.h>

#include <errno.h>
#include <string.h>

#include "tof_l7_quiesce.hpp"

using namespace lexxhard;

namespace {

tof_l7_quiesce::inputs clean()
{
    return tof_l7_quiesce::inputs{
        .argument_fault = 0, .port_errno = 0, .completed = 12U, .uld_status = tof_l7_quiesce::kUldOk};
}

void before(void *)
{
    tof_l7_quiesce::forget_all();
}

}  // namespace

ZTEST_SUITE(tof_l7_quiesce, NULL, NULL, before, NULL, NULL);

ZTEST(tof_l7_quiesce, test_everything_succeeded_is_the_only_ok)
{
    const tof_l7_quiesce::verdict v{tof_l7_quiesce::classify(clean())};
    zassert_equal(v.rc, 0);
    zassert_equal(v.which, tof_l7_quiesce::layer::ok);
}

ZTEST(tof_l7_quiesce, test_an_argument_fault_outranks_everything_below_it)
{
    /* Including a transport errno and a status byte that would otherwise be read: this build handed
     * the port something it cannot process, so nothing the bus did is worth interpreting. */
    tof_l7_quiesce::inputs in{clean()};
    in.argument_fault = -EINVAL;
    in.port_errno = -EIO;
    in.uld_status = tof_l7_quiesce::kUldTimeout;

    const tof_l7_quiesce::verdict v{tof_l7_quiesce::classify(in)};
    zassert_equal(v.rc, -EINVAL);
    zassert_equal(v.which, tof_l7_quiesce::layer::image_fault);
}

ZTEST(tof_l7_quiesce, test_a_transport_error_is_not_read_as_a_handshake_timeout)
{
    /* THE CASE REVIEW ASKED FOR BY NAME. A status byte of exactly the timeout code, with the port
     * reporting an errno, must NOT be classified as "the handshake did not complete": the exchange
     * did not finish, so the device's answer was never fully heard. On today's tree the port cannot
     * actually return 1, and that is precisely why this must not be the thing the conclusion rests
     * on -- see the header. */
    tof_l7_quiesce::inputs in{clean()};
    in.port_errno = -EIO;
    in.uld_status = tof_l7_quiesce::kUldTimeout;

    const tof_l7_quiesce::verdict v{tof_l7_quiesce::classify(in)};
    zassert_equal(v.which, tof_l7_quiesce::layer::transport, "a timeout-shaped status with a "
                                                             "transport error is a transport fault");
    zassert_equal(v.rc, -EIO);
}

ZTEST(tof_l7_quiesce, test_a_transport_error_outranks_an_ok_status_too)
{
    /* The other direction of the same rule: the ULD can accumulate OK while the port recorded a
     * failure on a call whose status was discarded. "The device said fine" is not an answer when
     * the question did not arrive. */
    tof_l7_quiesce::inputs in{clean()};
    in.port_errno = -ENXIO;
    in.uld_status = tof_l7_quiesce::kUldOk;

    const tof_l7_quiesce::verdict v{tof_l7_quiesce::classify(in)};
    zassert_equal(v.which, tof_l7_quiesce::layer::transport);
    zassert_equal(v.rc, -ENXIO);
}

ZTEST(tof_l7_quiesce, test_the_timeout_code_means_the_handshake_only_with_a_clean_port)
{
    tof_l7_quiesce::inputs in{clean()};
    in.uld_status = tof_l7_quiesce::kUldTimeout;

    const tof_l7_quiesce::verdict v{tof_l7_quiesce::classify(in)};
    zassert_equal(v.which, tof_l7_quiesce::layer::handshake);
    zassert_equal(v.rc, -EIO);
}

ZTEST(tof_l7_quiesce, test_any_other_status_keeps_its_own_byte_and_is_not_a_timeout)
{
    /* 255 is what the Zephyr port returns for a failed transfer and 127 for a bad argument, so
     * these reach here only when the port itself reported nothing -- an anomaly worth its own name
     * rather than one folded into the timeout. */
    static const uint8_t others[]{2U, 4U, 3U, 126U, 127U, 128U, 255U};
    for (const uint8_t status : others) {
        tof_l7_quiesce::inputs in{clean()};
        in.uld_status = status;

        const tof_l7_quiesce::verdict v{tof_l7_quiesce::classify(in)};
        zassert_equal(v.which, tof_l7_quiesce::layer::device_status, "status 0x%02x", status);
        zassert_equal(v.rc, -EIO, "status 0x%02x", status);
    }
}

ZTEST(tof_l7_quiesce, test_the_returned_errno_is_the_one_the_adapter_always_returned)
{
    /* NAMING THE FAILURE MUST NOT CHANGE WHICH FAILURES ARE FAILURES. The adapter's contract before
     * this module existed was: the argument fault, else the transport errno, else 0 for an OK
     * status and -EIO for anything else. Pinned here so the diagnostic round cannot quietly become
     * a behaviour round. */
    tof_l7_quiesce::inputs in{clean()};
    zassert_equal(tof_l7_quiesce::classify(in).rc, 0);

    in.uld_status = 3U;
    zassert_equal(tof_l7_quiesce::classify(in).rc, -EIO);

    in = clean();
    in.port_errno = -ETIMEDOUT;
    zassert_equal(tof_l7_quiesce::classify(in).rc, -ETIMEDOUT);

    in = clean();
    in.argument_fault = -EFAULT;
    in.port_errno = -ETIMEDOUT;
    zassert_equal(tof_l7_quiesce::classify(in).rc, -EFAULT);
}

ZTEST(tof_l7_quiesce, test_a_span_longer_than_the_old_wrap_is_reported_in_full)
{
    /* THE DEFECT, IN NUMBERS. The first version timed the call with `k_cycle_get_32()`, which wraps
     * every 2^32 / 216e6 = 19.884 s at 216 MHz; review measured a 30-second call recorded as
     * 10.116 s -- 30 s minus one wrap. The arithmetic is checked here across that boundary; that
     * the adapter reads a 64-bit source at all is checked by the call-site gate, because it is a
     * property of a call site. */
    zassert_equal(tof_l7_quiesce::span_us(0, 30'000'000), 30'000'000U,
                  "thirty seconds is thirty seconds, not thirty minus a wrap");
    zassert_equal(tof_l7_quiesce::span_us(0, 19'884'000), 19'884'000U, "at the old wrap point");
    zassert_equal(tof_l7_quiesce::span_us(0, 19'884'001), 19'884'001U, "just past it");

    /* The length a quiesce reaches when every polled read spends the I2C driver's 500 ms timeout:
     * 500 polls times 510 ms. Twelve wraps of the old counter. */
    zassert_equal(tof_l7_quiesce::span_us(0, 255'000'000), 255'000'000U);

    /* And it does not matter where on the clock the call started. */
    zassert_equal(tof_l7_quiesce::span_us(987'654'321, 987'654'321 + 255'000'000), 255'000'000U);
}

ZTEST(tof_l7_quiesce, test_an_impossible_span_saturates_rather_than_looking_plausible)
{
    /* Unreachable in this image -- the deadman cold-reboots after ten minutes and UINT32_MAX us is
     * 71 -- and the point is the DIRECTION of the failure. A wrap turns a long call into a short
     * believable number, which is how the first version hid; a clamp turns it into an obviously
     * maximal one. */
    zassert_equal(tof_l7_quiesce::span_us(0, 5'000'000'000LL), UINT32_MAX);
    zassert_equal(tof_l7_quiesce::span_us(0, 1LL << 40), UINT32_MAX);
}

ZTEST(tof_l7_quiesce, test_a_span_measured_backwards_is_zero_not_enormous)
{
    zassert_equal(tof_l7_quiesce::span_us(100, 100), 0U);
    zassert_equal(tof_l7_quiesce::span_us(500, 100), 0U, "unsigned subtraction would say 4294966.9 s");
}

ZTEST(tof_l7_quiesce, test_nothing_recorded_is_its_own_answer_not_a_clean_one)
{
    /* Zeroes would read as "quiesced, everything fine, no time taken". A run that never quiesced
     * has to be distinguishable from one that quiesced cleanly. */
    zassert_false(tof_l7_quiesce::last().taken);
    zassert_equal(tof_l7_quiesce::last().sequence, 0U);
}

ZTEST(tof_l7_quiesce, test_a_successful_quiesce_is_recorded_too)
{
    /* Review's requirement, and the half an "only on failure" record would miss: a successful
     * quiesce followed by a later failure is a different story from two failures, and only the
     * record tells them apart. */
    tof_l7_quiesce::remember(clean(), 1234U);

    const tof_l7_quiesce::record &q{tof_l7_quiesce::last()};
    zassert_true(q.taken);
    zassert_equal(q.sequence, 1U);
    zassert_equal(q.which, tof_l7_quiesce::layer::ok);
    zassert_equal(q.rc, 0);
    zassert_equal(q.elapsed_us, 1234U);
    zassert_equal(q.raw.completed, 12U);
}

ZTEST(tof_l7_quiesce, test_every_raw_field_is_kept_not_just_the_verdict)
{
    tof_l7_quiesce::inputs in{clean()};
    in.port_errno = -EIO;
    in.completed = 9U;
    in.uld_status = 255U;
    tof_l7_quiesce::remember(in, 5'004'321U);

    const tof_l7_quiesce::record &q{tof_l7_quiesce::last()};
    zassert_equal(q.raw.argument_fault, 0);
    zassert_equal(q.raw.port_errno, -EIO);
    zassert_equal(q.raw.completed, 9U);
    zassert_equal(q.raw.uld_status, 255U, "the raw byte is the evidence, not just the layer");
    zassert_equal(q.elapsed_us, 5'004'321U);
    zassert_equal(q.which, tof_l7_quiesce::layer::transport);
}

ZTEST(tof_l7_quiesce, test_each_quiesce_is_a_new_record_and_says_which_one_it_is)
{
    /* Two prints of the same record and two records that happen to look alike are different
     * observations, and a reader cannot tell them apart without the sequence number. */
    tof_l7_quiesce::remember(clean(), 10U);
    const uint32_t first{tof_l7_quiesce::last().sequence};

    tof_l7_quiesce::inputs second{clean()};
    second.uld_status = tof_l7_quiesce::kUldTimeout;
    tof_l7_quiesce::remember(second, 5'000'000U);

    zassert_equal(tof_l7_quiesce::last().sequence, first + 1U);
    zassert_equal(tof_l7_quiesce::last().which, tof_l7_quiesce::layer::handshake);
    zassert_equal(tof_l7_quiesce::last().elapsed_us, 5'000'000U);
}

ZTEST(tof_l7_quiesce, test_reading_the_record_does_not_consume_it)
{
    /* The shell prints it on every `status`, so a read that cleared it would make the second print
     * of the same fault read as "none recorded". */
    tof_l7_quiesce::inputs in{clean()};
    in.uld_status = tof_l7_quiesce::kUldTimeout;
    tof_l7_quiesce::remember(in, 4'999'000U);

    for (int i = 0; i < 3; ++i) {
        const tof_l7_quiesce::record &q{tof_l7_quiesce::last()};
        zassert_true(q.taken, "read %d", i);
        zassert_equal(q.sequence, 1U, "read %d", i);
        zassert_equal(q.which, tof_l7_quiesce::layer::handshake, "read %d", i);
    }
}

/* ---- the bring-up step records ------------------------------------------------------------- */

#include "tof_l7_step_record.hpp"

namespace {

tof_l7::operation_status a_failure()
{
    tof_l7::operation_status s{};
    s.failed_stage = tof_l7::stage::initialise;
    s.port_errno = -EIO;
    s.argument_fault = -EINVAL;
    s.uld_status = 0x2AU;
    s.sample_present = false;
    return s;
}

void before_steps(void *)
{
    tof_l7_steps::forget_all();
}

}  // namespace

ZTEST_SUITE(tof_l7_steps, NULL, NULL, before_steps, NULL, NULL);

ZTEST(tof_l7_steps, test_nothing_attempted_is_its_own_answer)
{
    /* A step skipped because an earlier one refused must not read as a step that ran and reported
     * zeroes -- which is the whole reason `attempted` is a field. */
    for (uint8_t i = 0; i < static_cast<uint8_t>(tof_l7_steps::which::count); ++i) {
        const tof_l7_steps::record &r{tof_l7_steps::last(static_cast<tof_l7_steps::which>(i))};
        zassert_false(r.attempted, "step %u", i);
        zassert_equal(r.rc, 0, "and its zeroes are not an observation");
    }
}

ZTEST(tof_l7_steps, test_every_field_comes_straight_from_the_operation_status)
{
    /* Reused, not re-adjudicated: this module adds no verdict of its own, so each field must arrive
     * unchanged. */
    tof_l7_steps::remember(tof_l7_steps::which::open, -EIO, a_failure());

    const tof_l7_steps::record &r{tof_l7_steps::last(tof_l7_steps::which::open)};
    zassert_true(r.attempted);
    zassert_equal(r.rc, -EIO);
    zassert_equal(r.failed_stage, tof_l7::stage::initialise);
    zassert_equal(r.port_errno, -EIO);
    zassert_equal(r.argument_fault, -EINVAL);
    zassert_equal(r.uld_status, 0x2AU);
    zassert_false(r.sample_present);
}

ZTEST(tof_l7_steps, test_a_successful_step_is_recorded_too)
{
    tof_l7::operation_status ok{};
    ok.sample_present = true;
    tof_l7_steps::remember(tof_l7_steps::which::read, 0, ok);

    const tof_l7_steps::record &r{tof_l7_steps::last(tof_l7_steps::which::read)};
    zassert_true(r.attempted);
    zassert_equal(r.rc, 0);
    zassert_equal(r.failed_stage, tof_l7::stage::none);
    zassert_true(r.sample_present, "whether anything came is the sensor layer's answer, not a guess");
}

ZTEST(tof_l7_steps, test_one_step_does_not_disturb_another)
{
    /* Separate slots, so the step that failed is still readable after the ones around it ran. */
    tof_l7_steps::remember(tof_l7_steps::which::close, 0, tof_l7::operation_status{});
    tof_l7_steps::remember(tof_l7_steps::which::open, -ENODEV, a_failure());
    tof_l7_steps::remember(tof_l7_steps::which::configure, 0, tof_l7::operation_status{});

    zassert_equal(tof_l7_steps::last(tof_l7_steps::which::open).rc, -ENODEV);
    zassert_equal(tof_l7_steps::last(tof_l7_steps::which::open).uld_status, 0x2AU);
    zassert_equal(tof_l7_steps::last(tof_l7_steps::which::close).rc, 0);
    zassert_equal(tof_l7_steps::last(tof_l7_steps::which::configure).rc, 0);
    zassert_false(tof_l7_steps::last(tof_l7_steps::which::start).attempted, "and start never ran");
}

ZTEST(tof_l7_steps, test_a_step_outside_the_range_answers_never_attempted)
{
    /* Rather than reading past the array: a reader must not be handed a neighbouring step's
     * observation. */
    const tof_l7_steps::record &r{tof_l7_steps::last(tof_l7_steps::which::count)};
    zassert_false(r.attempted);
    zassert_equal(r.rc, 0);
}

ZTEST(tof_l7_steps, test_forgetting_resets_every_slot)
{
    tof_l7_steps::remember(tof_l7_steps::which::start, -ETIMEDOUT, a_failure());
    tof_l7_steps::forget_all();

    for (uint8_t i = 0; i < static_cast<uint8_t>(tof_l7_steps::which::count); ++i)
        zassert_false(tof_l7_steps::last(static_cast<tof_l7_steps::which>(i)).attempted, "step %u", i);
}

ZTEST(tof_l7_steps, test_every_step_has_a_name_and_the_bound_is_not_one)
{
    static const char *const expected[]{"close", "open", "configure", "start", "read"};
    for (uint8_t i = 0; i < static_cast<uint8_t>(tof_l7_steps::which::count); ++i)
        zassert_equal(strcmp(tof_l7_steps::name(static_cast<tof_l7_steps::which>(i)), expected[i]), 0,
                      "step %u is named %s", i, tof_l7_steps::name(static_cast<tof_l7_steps::which>(i)));
    zassert_not_equal(strcmp(tof_l7_steps::name(tof_l7_steps::which::count), "close"), 0,
                      "the array bound is not a step");
}
