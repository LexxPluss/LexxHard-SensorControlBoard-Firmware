/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Where the stored blob record actually lives: storage_partition, and the address it is visible at.
 *
 * Two responsibilities, both of them about flash rather than about the record:
 *
 *   - open the partition and read bytes out of it, so tof_l7_blob_record can do its checks with no
 *     knowledge of flash at all (which is what makes that logic host-testable);
 *   - work out where the partition is MAPPED, so a verified payload can be handed to the I2C port as
 *     a pointer instead of copied into 84 KiB of RAM that does not exist.
 *
 * The mapping is the reason this file is separate and the reason it is board-specific. Internal
 * flash on this part is memory-mapped and readable while code executes from it -- only erase and
 * write stall the bus -- so the payload can be streamed straight out of flash into an I2C transfer.
 * That is a property of internal flash, not of Zephyr's flash API: a record on an external device
 * would have to be streamed through read() instead, which is why verify() accepts a null mapping and
 * still returns a verdict.
 */

#pragma once

/* Both flags, where the development images asked only for the chain. Nothing in a chain image
 * without the L7 ULD calls this on THIS branch, and the Kconfig the record needs -- CRC and
 * tinycrypt SHA-256 -- is scoped to L7 builds by overlays/tof_l7.conf, so the wider guard
 * compiled a translation unit whose headers the image had not been configured to provide. It
 * only linked at all because those images enabled tinycrypt globally.
 *
 * If the header-only validation command is ever brought over for a chain image with no ULD --
 * it exists on the development branch and reports the stored record without being able to use
 * it -- then this guard and that Kconfig scope widen together, which is the pairing that was
 * missing. */
#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_L7_ULD)

#include <stddef.h>

#include "tof_l7_blob_record.hpp"

namespace lexxhard::tof_l7_blob {

/* Everything a diagnostic needs in one answer.
 *
 * `stored` is filled on a best-effort basis whenever the header parsed, INCLUDING on refusals: the
 * useful message is "this firmware wants 86,016 bytes with digest ab..., the partition holds 84,992
 * with digest cd...", and a status on its own cannot say that. */
struct report {
    status st{status::unreadable};
    header_info stored{};
    size_t region_size{0};
};

/* Reads and checks the record in storage_partition against what the caller was built for.
 *
 * On status::ok, and only then, `out` carries a pointer into mapped flash and the payload length.
 * Every other status clears it, including a view left by an earlier successful call.
 */
report verify_stored(const accept_list &accepted, blob_view &out);

// The header alone, for a bring-up diagnostic that has no expectation to compare against yet.
report stored_header();

}  // namespace lexxhard::tof_l7_blob

#endif  // ENABLE_TOF_CHAIN && ENABLE_TOF_L7_ULD
