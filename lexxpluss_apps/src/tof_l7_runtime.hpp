/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Boot-time ownership of the verified VL53L7CX device-firmware payload.
 *
 * The record reader answers whether bytes are acceptable; this layer makes that answer a
 * single-shot subsystem state.  No L7 adapter can obtain a firmware pointer until bootstrap() has
 * verified storage_partition against the accept-list compiled into the signed image.
 */

#pragma once

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include <stddef.h>
#include <stdint.h>

#include "tof_l7_blob_provider.hpp"

namespace lexxhard::tof_l7_runtime {

enum class stage : uint8_t {
    not_started,
    verifying,
    available,
    refused,
};

struct snapshot {
    stage current_stage{stage::not_started};
    tof_l7_blob::report verification{};
    bool firmware_available{false};
    size_t firmware_size{0};
};

/* Verify storage exactly once. A refusal is local to L7: the cliff path and its health channel must
 * still boot, while every L7 operation remains unavailable. */
int bootstrap();

snapshot current();
const char *stage_name(stage s);

/* The only production access to the external device firmware. Both return an empty answer unless a
 * completed bootstrap published stage::available. */
const uint8_t *firmware_data();
size_t firmware_size();

using verifier = tof_l7_blob::report (*)(const tof_l7_blob::accept_list &,
                                         tof_l7_blob::blob_view &);
#ifdef CONFIG_ZTEST
int bootstrap_for_test(verifier fn, const tof_l7_blob::accept_list &accepted);
void reset_for_test();
#endif

}  // namespace lexxhard::tof_l7_runtime

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_L7_ULD
