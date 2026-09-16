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

// The bench multi-frame read loop, driven by a scripted ops table. The loop is the whole point
// of the command -- `tof cliff read` stops at the first fresh frame, so every L4 sample this
// project has recorded is frame one after a restart -- and the ways it can be wrong are all
// silent: a dropped frame, a miscounted attempt, or a session that keeps polling a sensor that
// already failed.

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "tof_cliff_stream_loop.hpp"

namespace
{

namespace st = lexxhard::tof_cliff_stream;
namespace acq = lexxhard::tof_acq;

// One scripted reply per read call.
struct scripted_read {
    int rc{0};
    bool fresh{false};
    bool sample_present{false};
    uint8_t stream_count{0};
};

constexpr size_t kMaxScripted{8};

struct script {
    scripted_read replies[kMaxScripted]{};
    size_t reply_count{0};
    size_t next{0};
    int read_calls{0};
    int sleep_calls{0};
};

script g_script;
uint8_t g_seen_stream_counts[kMaxScripted];
size_t g_seen_count;

int fake_read(void *dev, void *, struct tof_cliff_sample *out, acq::op_status *st_out)
{
    auto &s{*static_cast<script *>(dev)};
    ++s.read_calls;
    memset(out, 0, sizeof(*out));
    memset(st_out, 0, sizeof(*st_out));
    if (s.next >= s.reply_count)
        return -EIO;
    const scripted_read r{s.replies[s.next++]};
    out->fresh = r.fresh;
    out->stream_count = r.stream_count;
    out->target_count = 0;
    out->entry_count = 1;
    st_out->sample_present = r.sample_present;
    return r.rc;
}

void collect(void *, unsigned, const struct tof_cliff_sample &sample)
{
    if (g_seen_count < kMaxScripted)
        g_seen_stream_counts[g_seen_count++] = sample.stream_count;
}

void counting_sleep(unsigned)
{
    ++g_script.sleep_calls;
}

acq::source_ops ops_with_fake_read()
{
    acq::source_ops ops{};
    ops.read_cliff_sample = fake_read;
    return ops;
}

st::progress run(const scripted_read *replies, size_t count, unsigned want, unsigned max_attempts,
                 unsigned gap_ms = 0)
{
    g_script = script{};
    for (size_t i{0}; i < count && i < kMaxScripted; ++i)
        g_script.replies[i] = replies[i];
    g_script.reply_count = count;
    g_seen_count = 0;
    acq::op_status status{};
    const acq::source_ops ops{ops_with_fake_read()};
    const st::params p{want, gap_ms, max_attempts};
    return st::run_loop(ops, &g_script, nullptr, status, p, collect, nullptr, counting_sleep);
}

}  // namespace

// The reason the command exists: several fresh frames come back from ONE session, in order.
ZTEST(tof_cliff_stream_loop, test_collects_every_fresh_frame_in_order)
{
    const scripted_read sc[]{ { 0, true, true, 10 }, { 0, true, true, 11 }, { 0, true, true, 12 } };
    const auto pr{ run(sc, 3, 3, 10) };

    zassert_equal(3, pr.frames_collected);
    zassert_equal(3, g_script.read_calls, "one read per frame, no restart between them");
    zassert_equal(3u, g_seen_count);
    zassert_equal(10, g_seen_stream_counts[0]);
    zassert_equal(11, g_seen_stream_counts[1]);
    zassert_equal(12, g_seen_stream_counts[2]);
}

// A not-ready check spends an attempt but is not a frame. Getting this wrong would report a
// sensor as having produced frames it never produced.
ZTEST(tof_cliff_stream_loop, test_a_not_ready_check_spends_an_attempt_but_is_not_a_frame)
{
    const scripted_read sc[]{ { 0, false, false, 0 }, { 0, false, false, 0 }, { 0, true, true, 7 } };
    const auto pr{ run(sc, 3, 1, 10) };

    zassert_equal(1, pr.frames_collected);
    zassert_equal(3, pr.attempts_used, "the two empty checks count as attempts");
    zassert_equal(1u, g_seen_count);
    zassert_equal(7, g_seen_stream_counts[0]);
}

// The loop stops as soon as it has what was asked for, rather than spending the whole budget.
ZTEST(tof_cliff_stream_loop, test_stops_at_the_requested_frame_count)
{
    const scripted_read sc[]{ { 0, true, true, 1 }, { 0, true, true, 2 }, { 0, true, true, 3 } };
    const auto pr{ run(sc, 3, 2, 10) };

    zassert_equal(2, pr.frames_collected);
    zassert_equal(2, g_script.read_calls, "the third reply was never requested");
}

// A failed read ends the session. Polling a sensor that already failed a fetch produces noise,
// not data.
ZTEST(tof_cliff_stream_loop, test_a_read_error_ends_the_session)
{
    const scripted_read sc[]{ { 0, true, true, 1 }, { -EIO, false, false, 0 }, { 0, true, true, 3 } };
    const auto pr{ run(sc, 3, 5, 10) };

    zassert_equal(1, pr.frames_collected);
    zassert_equal(-EIO, pr.last_read_rc);
    zassert_equal(2, g_script.read_calls, "nothing was read after the failure");
}

// THE ONE THAT MATTERS. read_once reports a failed re-arm as an error while leaving the fetched
// sample intact and sample_present set. Checking the return code before the sample would throw
// away a frame the sensor really produced -- and it is the last frame of the session, the one
// that explains why it ended.
ZTEST(tof_cliff_stream_loop, test_a_sample_that_arrives_with_a_failed_rearm_is_kept)
{
    const scripted_read sc[]{ { 0, true, true, 4 }, { -EIO, true, true, 5 } };
    const auto pr{ run(sc, 2, 5, 10) };

    zassert_equal(2, pr.frames_collected, "the re-arm failure must not discard its own sample");
    zassert_equal(-EIO, pr.last_read_rc);
    zassert_equal(2u, g_seen_count);
    zassert_equal(5, g_seen_stream_counts[1]);
}

// An error with no sample behind it is just an error.
ZTEST(tof_cliff_stream_loop, test_an_error_without_a_sample_yields_no_frame)
{
    const scripted_read sc[]{ { -EIO, false, false, 0 } };
    const auto pr{ run(sc, 1, 5, 10) };

    zassert_equal(0, pr.frames_collected);
    zassert_equal(-EIO, pr.last_read_rc);
}

// fresh without sample_present is not a frame: the guard is the status, not the flag alone.
ZTEST(tof_cliff_stream_loop, test_fresh_without_sample_present_is_not_a_frame)
{
    const scripted_read sc[]{ { 0, true, false, 9 } };
    const auto pr{ run(sc, 1, 5, 10) };

    zassert_equal(0, pr.frames_collected);
    zassert_equal(0u, g_seen_count);
}

// The attempt budget bounds the session even when the sensor never becomes ready. Without it the
// loop would hold the chain lock indefinitely.
ZTEST(tof_cliff_stream_loop, test_the_attempt_budget_bounds_a_sensor_that_never_readies)
{
    const scripted_read sc[]{ { 0, false, false, 0 },
                              { 0, false, false, 0 },
                              { 0, false, false, 0 },
                              { 0, false, false, 0 } };
    const auto pr{ run(sc, 4, 2, 3) };

    zassert_equal(0, pr.frames_collected);
    zassert_equal(3, pr.attempts_used);
    zassert_equal(3, g_script.read_calls);
}

// The gap separates reads; it is never spent after the last one. A pause there is dead time
// holding the chain lock, and on a bus this command shares that is not free.
ZTEST(tof_cliff_stream_loop, test_the_gap_separates_reads_and_is_never_spent_after_the_last)
{
    const scripted_read one[]{ { 0, true, true, 1 } };
    run(one, 1, 1, 5, 20);
    zassert_equal(0, g_script.sleep_calls, "a single requested frame has nothing to wait for");

    const scripted_read two[]{ { 0, true, true, 1 }, { 0, true, true, 2 } };
    run(two, 2, 2, 5, 20);
    zassert_equal(1, g_script.sleep_calls, "one gap BETWEEN the two frames, none after the second");

    const scripted_read retry[]{ { 0, false, false, 0 }, { 0, true, true, 2 } };
    run(retry, 2, 1, 5, 20);
    zassert_equal(1, g_script.sleep_calls, "one pause between the empty check and the retry");

    const scripted_read nogap[]{ { 0, true, true, 1 }, { 0, true, true, 2 } };
    run(nogap, 2, 2, 5, 0);
    zassert_equal(0, g_script.sleep_calls, "gap_ms=0 means no pause at all");
}

ZTEST_SUITE(tof_cliff_stream_loop, NULL, NULL, NULL, NULL, NULL);
