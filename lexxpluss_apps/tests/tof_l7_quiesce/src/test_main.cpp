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
