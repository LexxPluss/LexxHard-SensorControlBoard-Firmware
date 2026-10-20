/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * What each bring-up step reported, kept so the next step's failure is not another bare errno.
 *
 * WHY THIS EXISTS. The quiesce record exists because a survivor refusal arrived as `-5` and could
 * not be placed in a layer. The steps after it are in the same shape right now: `hook_open`,
 * `hook_configure`, `hook_start` and `hook_read` each construct a `tof_l7::operation_status`, hand
 * it to the sensor layer, and then return the int and DROP it. So the moment the blockage moves
 * from the quiesce to the 86 KB download or the first frame, the next window learns exactly as
 * little as the last one did.
 *
 * REUSED, NOT RE-ADJUDICATED. Every field here comes out of `operation_status`, which the sensor
 * layer already fills with its own verdict -- the failed stage, the port's errno, the argument
 * fault, the ULD's status byte, whether a sample came. This module adds no second opinion about
 * what any of them mean, and no error classification of its own.
 *
 * AND IT DOES NOT READ THE PORT CONTEXT, which is the part worth stating because it would look like
 * an improvement. `open()` and `configure()` refuse early -- a null argument, a wrong lifecycle
 * state -- and return BEFORE anything clears the port. Its sticky errno and its completed-transfer
 * count therefore still belong to whatever ran last, which in a bring-up is the quiesce and its 511
 * transfers. Reading them here would attribute those to a call that never touched the bus, which is
 * the same misattribution this whole line of work has been removing. There is consequently NO
 * completed-transfer count for these steps, and the record does not invent one.
 *
 * `attempted` IS A FIELD BECAUSE "NOT CALLED" IS AN ANSWER. A step skipped because an earlier one
 * refused must not read as a step that ran and reported zeroes.
 *
 * SEPARATE STORAGE FROM THE QUIESCE RECORD, so that recording a step cannot overwrite the
 * observation the quiesce record exists to keep.
 */

#pragma once

#if defined(ENABLE_TOF_L7_ULD)

#include <stdint.h>

#include "tof_l7_status.hpp"

namespace lexxhard::tof_l7_steps {

/* The hooks the lifecycle drives, in the order a bring-up runs them. `count` is the array bound and
 * is not a step. */
enum class which : uint8_t {
    close = 0,
    open,
    configure,
    start,
    read,
    count,
};

struct record {
    /* False until the hook has run once since the last forget_all(). */
    bool attempted{false};
    /* What the hook returned, unchanged. */
    int rc{0};
    /* Straight from `operation_status`. No field below is computed here. */
    tof_l7::stage failed_stage{tof_l7::stage::none};
    int port_errno{0};
    int argument_fault{0};
    uint8_t uld_status{0};
    bool sample_present{false};
};

/* Called once per hook invocation, with what the hook returned and the status the sensor layer
 * filled. There is exactly one caller per step, which is what makes "no step overwrites another's
 * record" a property of the code rather than a convention. */
void remember(which step, int rc, const tof_l7::operation_status &status);

const record &last(which step);

/* Clear step records only when lifecycle::can_begin_round() accepts a round,
 * before its first step. A refused call preserves the previous failure;
 * an accepted round owns new records even when its first step fails.
 */
void begin_round(bool accepted);

/* HOW MANY ROUNDS HAVE BEGUN SINCE BOOT, so a reader can tell which `up` the records below belong
 * to. With a refused `up` no longer clearing them, the steps on screen are those of the last round
 * that STARTED, which is not necessarily the last command the operator typed. Not reset by
 * forget_all(): it counts bring-ups, not records. */
unsigned rounds_begun();

/* Unconditional. Reachable so a test can start from a known state; the application clears through
 * begin_round(). The quiesce record is a different module and is deliberately not touched. */
void forget_all();

const char *name(which step);

}  // namespace lexxhard::tof_l7_steps

#endif  // ENABLE_TOF_L7_ULD
