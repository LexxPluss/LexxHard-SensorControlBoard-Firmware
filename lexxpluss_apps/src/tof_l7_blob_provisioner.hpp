/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The one-shot provisioner for the VL53L7CX device-firmware record: DEV ONLY.
 *
 * WHY IT EXISTS. The L7 path refuses to range until storage_partition holds a verified record, and
 * the documented way to put one there is SWD -- which on this SCB asserts NRST, drops the main power
 * rail and takes the robot PC down with it, so it needs hands on the machine. This module lets a
 * signed DEV image do the same job after an ordinary CAN DFU: it carries the uncommitted record,
 * writes it, proves it, and only then writes the four-byte commit marker.
 *
 * WHAT IT MUST NEVER DO is destroy something it cannot account for. So the first act is a read-only
 * scan of the whole partition, and every byte is classified against the one record this image
 * carries. Anything that is not blank, not this record, and not an interrupted attempt at this record
 * is left untouched: zero erases, zero writes.
 *
 * THE HEADER IS THE PROVENANCE. Erase sets every bit to 1 and programming only clears bits, so a
 * partial program of THIS record leaves bytes with (stored & target) == target. That relation says a
 * byte COULD have come from this record, not that it did: 0x4d over a target of 0x4c satisfies it,
 * and so does most of any unrelated data that happens to sit on a mostly-erased sector. So the
 * relation is used only once the complete 64-byte header -- magic, format, length, payload digest,
 * CRC -- is byte for byte the one this image writes. Only then are the payload and marker bytes
 * judged by the bit relation, and bytes past the marker must still be 0xff.
 *
 * WHAT THAT COSTS, stated rather than hidden: two windows cannot be recovered automatically, and in
 * both the next boot sees `foreign` and does nothing.
 *   - The first write, which is the 64-byte header alone, torn by a reset: the header is incomplete.
 *   - The erase that starts a retry of an interrupted attempt, torn by a reset: erase raises bits, the
 *     header stops matching, and nothing can tell that sector from unknown data any more.
 * Without a second persistent record of provenance -- and the partition is one erase sector, so there
 * is nowhere to keep one -- "every power loss recovers" and "unknown data is never erased" cannot both
 * be true. This module keeps the second. Recovery from those two windows is SWD, on site.
 *
 * ORDERING: erase -> write the header alone -> write the payload -> read the whole record back and
 * compare every byte -> check the payload digest -> check the marker is still erased -> write the
 * marker WITHOUT an erase -> read the whole record back through the production reader. No failure
 * anywhere writes the marker; every failure is a distinct outcome so the shell can say which step it
 * was.
 *
 * The flash is injected, so the module is host-tested against a NOR-semantics fake with power loss
 * injected after every operation. Nothing here knows about Zephyr.
 */

#pragma once

#if defined(ENABLE_L7_BLOB_PROVISIONER)

#include <stddef.h>
#include <stdint.h>

#include "tof_l7_blob_record.hpp"

namespace lexxhard::tof_l7_blob_provisioner {

namespace blob = lexxhard::tof_l7_blob;

/* Offsets are relative to the start of the partition. read/write must transfer exactly len bytes
 * or return non-zero; erase must cover exactly [offset, offset + len). write must never erase. */
struct flash_ops {
    int (*read)(void *ctx, size_t offset, void *dst, size_t len){nullptr};
    int (*erase)(void *ctx, size_t offset, size_t len){nullptr};
    int (*write)(void *ctx, size_t offset, const void *src, size_t len){nullptr};
    void *ctx{nullptr};
};

/* What the partition held before this run touched it. */
enum class found : uint8_t {
    not_scanned = 0,
    blank,             // every byte 0xff
    committed_exact,   // this record and its marker, byte for byte; the rest 0xff
    uncommitted_exact, // this record byte for byte, marker and the rest still 0xff
    interrupted_exact, // this header exactly; every later byte between 0xff and this record
    foreign,           // anything else. Never erased, never written.
    unreadable,        // a read failed during the scan
};

/* How the run ended. Only the first two are success; every other value names the step that stopped
 * it, and in every one of them the marker was not written by this run. */
enum class outcome : uint8_t {
    not_run = 0,
    already_provisioned,      // committed record found and verified; no erase or write attempted
    provisioned_and_verified, // written, read back, committed, verified by the production reader
    foreign_or_unknown_data,  // refused: no erase or write attempted
    bad_embedded_record,      // this image's own record fails the production reader: nothing touched
    bad_config,               // region too small, or a missing hook: nothing touched
    scan_failed,              // the partition could not be read
    erase_failed,
    erase_verify_failed,      // erased, but something read back as other than 0xff
    write_failed,
    readback_failed,          // a read failed, or a byte differs from the record, after writing
    digest_mismatch,          // every byte matched, yet the payload does not hash to its header
    marker_not_blank,         // the marker bytes were not 0xff at commit time
    commit_failed,            // the marker write failed, or read back wrong
    final_verify_failed,      // the production reader refused the committed record
};

const char *found_name(found f);
const char *outcome_name(outcome o);

struct config {
    size_t region_size{0};
    /* The uncommitted record: 64-byte header plus payload. The marker is not in it on purpose. */
    const uint8_t *record{nullptr};
    size_t record_len{0};
    /* The signed image's accept-list: the final verification must accept exactly what the L7
     * runtime will accept, so it is the same list. */
    blob::accept_list accepted{};
    /* Where the partition is visible in the address space, for the reader's mapping check. Null in
     * a test that has no mapping; the target always passes the real base. */
    const uint8_t *mapped_base{nullptr};
};

struct report {
    found initial{found::not_scanned};
    outcome result{outcome::not_run};
    /* The production reader's verdict on the partition at the end of the run. */
    blob::status final_status{blob::status::unreadable};
    /* Attempts, counted before each call: a failed erase or write is still counted, because it may
     * have changed flash before it failed. */
    uint32_t erase_attempts{0};
    uint32_t write_attempts{0};
    uint32_t readbacks{0};
    /* True once this run has issued any erase or write, whether or not it succeeded -- a failed
     * operation may still have changed flash. False on already_provisioned and on every refusal. */
    bool flash_operation_attempted{false};
    int last_errno{0};
    /* Where the failing step was, when there is an offset to report. */
    size_t failed_offset{0};

    bool ok() const
    {
        return result == outcome::already_provisioned || result == outcome::provisioned_and_verified;
    }
};

/* Read-only: what the partition holds now, classified against the configured record. Performs no
 * erase and no write, ever; used by the run and by the `inspect` shell command alike. */
found classify(const flash_ops &f, const config &c, int &read_errno);

/* The embedded record, checked by the production reader as if it were already committed. A record
 * this image would itself refuse is not something to write. */
blob::status check_embedded(const config &c);

/* The whole provisioning run. */
report run(const flash_ops &f, const config &c);

}  // namespace lexxhard::tof_l7_blob_provisioner

#endif  // ENABLE_L7_BLOB_PROVISIONER
