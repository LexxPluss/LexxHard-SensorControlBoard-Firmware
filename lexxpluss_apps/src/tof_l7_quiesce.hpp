/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * What a quiesce attempt actually told us, and the one place the answer is kept.
 *
 * WHY THIS EXISTS. On 2026-10-12 a bench run on dasher1 refused at the survivor step with errno -5
 * and that was the entire record. -5 was all the adapter had kept: `hook_quiesce` computed an
 * argument fault, a transport errno, a completed-transfer count and the ULD's own status byte, and
 * then collapsed every one of them into `-EIO`. The failure could not be placed in a layer, so the
 * next step after a five-second stop sequence was a guess.
 *
 * So the information is not new instrumentation. It is the information the adapter already had and
 * threw away, now classified by one rule and kept where the shell can print it.
 *
 * THE ONE RULE: A TRANSPORT ERROR OUTRANKS THE STATUS BYTE.
 *
 * It is tempting to read the ULD's status alone -- `VL53L7CX_STATUS_TIMEOUT_ERROR` is 1, so "1
 * means the MCU stop never confirmed". On the patched tree that reading even happens to be sound:
 * the Zephyr port returns `VL53L7CX_STATUS_ERROR` (255) for a failed transfer and
 * `VL53L7CX_STATUS_INVALID_PARAM` (127) for a bad argument, never 1, and patch
 * 0002-stop-ranging-timeout-is-a-failure removed both places where a DEVICE byte was OR-ed into the
 * status.
 *
 * It is still not how this classifies, because soundness by coincidence is not a rule. The
 * separation rests on two independent properties -- that no port error code collides with 1, and
 * that nothing OR-s a device register into an accumulating status -- and the second has already
 * drifted once: restoring it is exactly what patch 0002 is for. A conclusion that would silently
 * become wrong if either drifted again is a conclusion that has to be re-derived every time anyone
 * reads it. So the transport layer is asked first and answers for itself, and `handshake` is
 * reachable only once the port has said it had no error at all.
 *
 * `completed` is carried for the same reason the probe classification carries it: an errno does not
 * say whether anything got through before it, and "the bus refused everything" and "the bus carried
 * nine transfers and then stopped" are different faults.
 *
 * Pure, and free of Zephyr and ULD headers, so every combination is testable without a bus or a
 * vendor object. The ULD's timeout code arrives as a named constant here and the adapter
 * static_asserts it against the vendor definition -- the value is pinned on both sides rather than
 * trusted on one.
 */

#pragma once

#if defined(ENABLE_TOF_L7_ULD)

#include <stdint.h>

namespace lexxhard::tof_l7_quiesce {

/* `VL53L7CX_STATUS_TIMEOUT_ERROR`. Cross-checked against the vendor header where that header is
 * available; see the static_assert in the adapter. */
constexpr uint8_t kUldTimeout{1U};
/* `VL53L7CX_STATUS_OK`. */
constexpr uint8_t kUldOk{0U};

struct inputs {
    /* Image-wide port argument fault: this build handed the port something it cannot process, so
     * nothing below is worth interpreting. */
    int argument_fault{0};
    /* The port's FIRST transport errno for this call, cleared before the call so it cannot be a
     * leftover from an earlier one. */
    int port_errno{0};
    /* COMPLETED chunk transactions. Not derivable from the errno. */
    uint32_t completed{0};
    /* The ULD's status byte, RAW. Kept as the byte rather than a boolean because which non-zero
     * value it is was the question the -5 could not answer. */
    uint8_t uld_status{kUldOk};
};

/* Which layer the attempt stopped in. Named rather than inferred at each reading site. */
enum class layer : uint8_t {
    /* Every transfer succeeded and the ULD reported success. */
    ok = 0,
    /* The port was handed a bad argument. Not a statement about the device or the bus. */
    image_fault,
    /* The port recorded a transport errno. The status byte is NOT interpreted in this case, however
     * it reads. */
    transport,
    /* The port had no error at all AND the ULD reported its timeout code: the stop sequence was
     * issued and the MCU stop was never confirmed within the ULD's five seconds. */
    handshake,
    /* The port had no error at all and the ULD reported something else. Carried as the raw byte for
     * analysis rather than folded into the one above. */
    device_status,
};

struct verdict {
    /* EXACTLY WHAT THE ADAPTER RETURNED BEFORE THIS MODULE EXISTED: the argument fault, else the
     * transport errno, else 0 for an OK status and -EIO for any other. This module names the
     * failure; it does not change which failures are failures, and a test pins that. */
    int rc{0};
    layer which{layer::ok};
};

verdict classify(const inputs &in);

/* THE SNAPSHOT, AND WHY IT IS A SEPARATE THING FROM THE VERDICT. Review's requirement is that every
 * quiesce -- success as well as failure -- leaves a complete record at the moment it returns, and
 * that nothing a later probe or stop does can rewrite it. A verdict computed on demand would
 * satisfy neither: by the time anyone asked, the port's sticky errno and completed count would
 * belong to whatever ran next. */
struct record {
    /* False until a quiesce has recorded one. Printing "all zero" as though it were an observation
     * is how a run with no quiesce at all reads as a clean one. */
    bool taken{false};
    /* Which quiesce this was, counted since boot. A reader comparing two prints can tell a fresh
     * record from the same one shown twice -- which is the mistake a bare timestamp-free print
     * invites, and the mistake the driver side of this project has already had to fix once. */
    uint32_t sequence{0};
    inputs raw{};
    layer which{layer::ok};
    int rc{0};
    /* How long the call took. The ULD polls for up to five seconds, so the duration is part of the
     * evidence -- but it is NOT a discriminator on its own: a transfer error does not make the ULD
     * return early, so a failing transport can spend the same five seconds. */
    uint32_t elapsed_us{0};
};

/* Called once per quiesce, with the raw observations and the measured duration. It classifies,
 * stamps a sequence number and stores the result. There is exactly one caller, which is what makes
 * "no later probe or stop rewrites it" a property of the code rather than a convention. */
void remember(const inputs &in, uint32_t elapsed_us);

/* The stored record. Reading does not clear it. */
const record &last();

/* Boot-time state, and what a test resets to. */
void forget_all();

}  // namespace lexxhard::tof_l7_quiesce

#endif  // ENABLE_TOF_L7_ULD
