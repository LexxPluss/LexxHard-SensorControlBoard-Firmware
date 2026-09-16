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

/* The read loop of the bench-only multi-frame probe, with no chain lock, no stage and no shell
 * in it, so it can be driven by the faked source_ops the acquisition suite already provides.
 *
 * WHY IT EXISTS AT ALL. probe_position() stops at the first fresh frame and then stops the
 * sensor, so every L4 sample recorded by this project -- on every machine, across every run --
 * has been the first frame after a restart, and every one of them reported no target. That is
 * not yet evidence that the sensors cannot range: the existing evidence has only ever observed
 * frame one, so a first-frame effect cannot be ruled out either. Nothing here decides which it
 * is; it removes the reason we cannot tell.
 *
 * It hands each frame straight to a sink instead of accumulating an array. That is not a style
 * choice: the shell thread has well under a kilobyte of stack headroom on this board, and a
 * buffer of frames on that stack overflows it silently, because assertions are not compiled in.
 */

#include <cstdint>

#include "tof_acquisition.hpp"
#include "tof_cliff_sample.h"

namespace lexxhard
{
namespace tof_cliff_stream
{

struct params {
    unsigned want_frames{1};
    unsigned gap_ms{0};
    unsigned max_attempts{1};
};

struct progress {
    unsigned frames_collected{0};
    unsigned attempts_used{0};
    int last_read_rc{0};
};

/* Called once per fresh frame, in order. The sample is a reference to the loop's single live
 * frame and must be consumed (printed, hashed, counted) before returning; it is overwritten by
 * the next iteration. */
using frame_sink = void (*)(void *ctx, unsigned index, const struct tof_cliff_sample &sample);

/* `sleep_ms` is injected so a test does not have to wait out a gap, and so this header carries
 * no Zephyr dependency of its own. A null sleep means no pause between reads. */
using sleep_fn = void (*)(unsigned ms);

inline progress run_loop(const tof_acq::source_ops &ops, void *dev, void *scratch,
                         tof_acq::op_status &status, const params &p, frame_sink sink, void *ctx,
                         sleep_fn sleep_ms)
{
    progress out{};
    struct tof_cliff_sample sample{};

    for (unsigned n{0}; n < p.max_attempts && out.frames_collected < p.want_frames; ++n) {
        out.attempts_used = n + 1;
        out.last_read_rc = ops.read_cliff_sample(dev, scratch, &sample, &status);

        /* The sample is checked BEFORE the return code, because read_once reports a failed
         * re-arm as an error while leaving the fetched sample intact and sample_present set.
         * Testing the return code first would throw away a frame the sensor really produced --
         * and it would be the last frame of the session, the one that explains why it ended. */
        if (status.sample_present && sample.fresh) {
            if (sink != nullptr)
                sink(ctx, out.frames_collected, sample);
            ++out.frames_collected;
        }

        /* A failed read ends the session. A sensor that could not be re-armed will not produce
         * another frame, and one that failed a fetch is not one to keep polling. */
        if (out.last_read_rc != 0)
            break;

        if (p.gap_ms != 0 && sleep_ms != nullptr && out.frames_collected < p.want_frames &&
            n + 1 < p.max_attempts)
            sleep_ms(p.gap_ms);
    }
    return out;
}

}  // namespace tof_cliff_stream
}  // namespace lexxhard
