/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Boot-time ownership of the verified VL53L7CX device-firmware payload.
 *
 * The record reader answers whether bytes are acceptable; this layer makes that answer a
 * single-shot subsystem state.  No L7 adapter can obtain a firmware pointer until bootstrap() has
 * verified storage_partition against the accept-list compiled into the signed image.
 */

#pragma once

#if defined(ENABLE_TOF_L7_ULD)

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
 * still boot, while every L7 operation remains unavailable.
 *
 * Reports what THIS CALL did, which is why it is not the entry point for a second consumer: the
 * caller that loses the race gets -EALREADY. */
int bootstrap();

/* THE ENTRY POINT WHEN MORE THAN ONE SENSOR NEEDS THE BLOB.
 *
 * One payload, one record, one accept-list: the architecture verifies the stored firmware once per
 * image and every L7 downloads the same bytes. So a second sensor must not verify again -- it must
 * ask whether the one verification succeeded.
 *
 * bootstrap() cannot answer that. It returns -EALREADY to whoever loses the CAS, and -EALREADY
 * covers `verifying`, `available` and `refused` alike. A caller reading non-zero as failure would
 * refuse to start on a perfectly verified blob; a caller reading -EALREADY as success would start
 * on a refused one. Both are wrong, and with a single consumer neither could happen.
 *
 * Returns what the caller needs rather than what this call performed:
 *
 *   0         verified and readable now -- firmware_data() is non-null
 *   -EAGAIN   another consumer is still verifying; ask again
 *   other     the refusal errno, the same value bootstrap() would have returned
 *
 * Still exactly one verification: this is bootstrap() plus a meaningful answer for the loser, not a
 * second path into the record. */
int ensure_verified();

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
int ensure_verified_for_test(verifier fn, const tof_l7_blob::accept_list &accepted);
void reset_for_test();
#endif

}  // namespace lexxhard::tof_l7_runtime

#endif  // ENABLE_TOF_L7_ULD
