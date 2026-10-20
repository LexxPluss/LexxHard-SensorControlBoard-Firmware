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

/* Boot-driven Package ToF thread using the shared device, stream and CAN layers.
 * ENABLE_TOF_PACKAGE_AUTO starts it; the bench build cannot own the same sensor.
 *
 * Before sending, listen on both configured IDs. Observed traffic refuses the
 * whole run, including diagnostics; silence does not prove an ID is unallocated.
 * Claims are budgeted. Exhaustion stops new claims while diagnostics, existing
 * ranging reads and owed stops continue. Verify the provisioned blob; never
 * provision it unattended. Boot-local numbering restarts at 1 after reset,
 * so this image does not establish persistent acquisition identity.
 */

#if defined(ENABLE_TOF_PACKAGE_AUTO)

#include <zephyr/kernel.h>

namespace lexxhard::tof_package_runtime {

/* The thread object main.cpp's RUN macro fills in. */
extern k_thread thread;

void run(void *p1, void *p2, void *p3);

}  // namespace lexxhard::tof_package_runtime

#endif  // ENABLE_TOF_PACKAGE_AUTO
