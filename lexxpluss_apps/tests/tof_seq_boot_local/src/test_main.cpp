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

/*
 * tof_seq_boot_local: the diagnostic sequence source.
 *
 * The property that matters most here is a DEFECT, asserted so that it is recorded rather than
 * assumed: a fresh source starts at one again, so numbers repeat across a restart. Every reading of
 * a stream driven by this source has to know that, and a test is the only place the claim cannot
 * drift away from the code.
 */

#include <zephyr/ztest.h>

#include "tof_package_wire_n.hpp"
#include "tof_seq_boot_local.hpp"

namespace boot_local = lexxhard::tof_seq_boot_local;
namespace wire = lexxhard::tof_package_wire_n;

ZTEST_SUITE(tof_seq_boot_local, NULL, NULL, NULL, NULL, NULL);

ZTEST(tof_seq_boot_local, test_it_starts_at_one_and_never_issues_the_reserved_number)
{
    boot_local::source s{};
    uint32_t n{0};

    zassert_equal(s.issue(&n), 0, "first issue");
    zassert_equal(n, boot_local::kFirstNumber, "the first number is one");
    zassert_not_equal(n, wire::kReservedSequence,
                      "zero is the wire's placeholder and must never be issued");
    zassert_equal(s.issued(), 1U, "counted");
}

ZTEST(tof_seq_boot_local, test_numbers_do_not_repeat_within_one_boot)
{
    boot_local::source s{};
    uint32_t previous{0};

    for (unsigned i{0}; i < 1000U; ++i) {
        uint32_t n{0};
        zassert_equal(s.issue(&n), 0, "issue %u", i);
        zassert_true(n > previous, "number %u (%u) did not advance past %u", i, n, previous);
        previous = n;
    }
    zassert_equal(s.issued(), 1000U, "counted");
}

ZTEST(tof_seq_boot_local, test_a_restart_repeats_the_numbers_it_already_issued)
{
    /* THE DEFECT, PINNED. This is what makes the source a diagnostic and not an implementation of
     * the requirement: the same number comes back after a reset, which is the repeat candidate N's
     * identity cannot tolerate. If this test ever starts failing, something has given the source
     * persistence and the comment about what it may be used to prove is out of date. */
    uint32_t first_run[4]{};
    {
        boot_local::source s{};
        for (unsigned i{0}; i < 4U; ++i)
            zassert_equal(s.issue(&first_run[i]), 0, "run one, %u", i);
    }

    boot_local::source after_reset{};
    for (unsigned i{0}; i < 4U; ++i) {
        uint32_t n{0};
        zassert_equal(after_reset.issue(&n), 0, "run two, %u", i);
        zassert_equal(n, first_run[i], "a restart must repeat -- that is the known limitation");
    }
}

ZTEST(tof_seq_boot_local, test_exhaustion_refuses_rather_than_wrapping)
{
    /* Started near the top through the test seam. A wrap would hand number one to a second
     * acquisition inside a SINGLE boot, which no restart could explain. */
    boot_local::source s{UINT32_MAX - 1U};

    uint32_t n{0};
    zassert_equal(s.issue(&n), 0, "the second-to-last number");
    zassert_equal(n, UINT32_MAX - 1U, "value");
    zassert_false(s.spent(), "not spent yet");

    zassert_equal(s.issue(&n), 0, "the last number is usable");
    zassert_equal(n, UINT32_MAX, "the top of the space is issued, not skipped");
    zassert_true(s.spent(), "and the source retires");

    const uint32_t untouched{0xDEADBEEFUL};
    n = untouched;
    zassert_equal(s.issue(&n), -ENOSPC, "exhausted");
    zassert_equal(n, untouched, "a refusal must leave the caller's value alone");
    zassert_equal(s.issued(), 2U, "a refused issue is not an issue");
}

ZTEST(tof_seq_boot_local, test_the_seam_cannot_be_used_to_reach_the_reserved_number)
{
    boot_local::source s{0};
    zassert_true(s.spent(), "a source started at zero is born spent");

    uint32_t n{7};
    zassert_equal(s.issue(&n), -ENOSPC, "and issues nothing");
    zassert_equal(n, 7U, "leaving the caller's value alone");
}
