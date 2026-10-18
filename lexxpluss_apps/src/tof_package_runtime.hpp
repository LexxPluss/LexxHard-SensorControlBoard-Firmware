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

/*
 * THE UNATTENDED PACKAGE ToF RUNTIME: one thread that brings the sensor up at boot, keeps it
 * measuring, and puts every acquisition on can2 -- with nobody typing anything.
 *
 * It is the bench's `bind` + `takeover` + `stream` without the shell, and deliberately not a copy
 * of them: the device is tof_package_device, the publication rule is tof_package_stream, the
 * transport is tof_package_can, and WHICH of those to do next is tof_package_auto's policy. What
 * is left here, and all that is left here, is the Zephyr part: a thread, a clock, two build-time
 * numbers turned into a configuration, and the loop that asks the policy and acts on the answer.
 *
 * ENABLED BY THE BUILD AND BY NOTHING ELSE. ENABLE_TOF_PACKAGE_AUTO is what starts this thread,
 * and it cannot be combined with ENABLE_TOF_PACKAGE_BENCH: one owner per image, which is what lets
 * the device layer's objects stay unlocked.
 *
 * WHAT IT WILL NOT DO:
 *
 *   - It will not transmit on identifiers somebody else is using. The identifiers for this stream
 *     are UNREGISTERED candidates, so the thread listens on both before it sends anything and
 *     refuses the whole run if either carries traffic. A refusal here sends nothing at all, not
 *     even a diagnostic: a diagnostic would go out on the colliding identifier.
 *   - It will not keep claiming a sensor that will not come up. Every claim is counted against a
 *     budget the policy holds, and when it is gone the thread keeps publishing diagnostics and
 *     stops touching the device.
 *   - It will not provision the blob. ensure_verified() is asked and reported; an unprovisioned
 *     board is a refusal to acquire, not a cue to write 86 KB into storage_partition unattended.
 *   - It will not pretend its sequence numbers are durable. The source is boot-local: numbers
 *     restart at 1 after any reset, so a consumer that outlives a board reset sees them repeat.
 *     That is a property of this build, stated here, and the reason the stream is still an
 *     observation of the path rather than evidence of an identity.
 */

#if defined(ENABLE_TOF_PACKAGE_AUTO)

#include <zephyr/kernel.h>

namespace lexxhard::tof_package_runtime {

/* The thread object main.cpp's RUN macro fills in. */
extern k_thread thread;

void run(void *p1, void *p2, void *p3);

}  // namespace lexxhard::tof_package_runtime

#endif  // ENABLE_TOF_PACKAGE_AUTO
