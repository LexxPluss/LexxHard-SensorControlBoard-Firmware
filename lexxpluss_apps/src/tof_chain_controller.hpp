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

// Zephyr glue for the ToF chain (AMRSW-2322 Phase 2): the real-bus
// implementation of tof_enum::chain_ops plus the manual `tof enum`
// commissioning shell command. Compiled only under ENABLE_TOF_CHAIN; see
// tof_chain_controller.cpp for the ownership rules (i2c2, the tug encoder,
// the chain mutex).
//
// HARDWARE VERIFICATION PENDING: the tri-state probe rests on the driver
// patch's return codes, whose three-way check on the robot (empty address
// -> nack, clamped SCL -> transport_error, live device -> ack) has not
// been executed yet.

#ifdef ENABLE_TOF_CHAIN

#include <zephyr/kernel.h>

namespace lexxhard::tof_chain_controller {

void init();

// The chain control lines and the bus are a single shared resource. The
// shell command holds this mutex for the whole enumeration; the future
// acquisition thread (Phase 3) must take it for its whole session too, so
// manual commissioning and automatic acquisition can never drive the chain
// concurrently.
k_mutex &chain_lock();

}  // namespace lexxhard::tof_chain_controller

#endif  // ENABLE_TOF_CHAIN
