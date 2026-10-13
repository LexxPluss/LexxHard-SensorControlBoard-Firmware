/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_l7_step_record.hpp"

#include <stddef.h>

#if defined(ENABLE_TOF_L7_ULD)

namespace lexxhard::tof_l7_steps {

namespace {

record kept[static_cast<size_t>(which::count)]{};
unsigned rounds{0};

bool in_range(which step)
{
    return static_cast<size_t>(step) < static_cast<size_t>(which::count);
}

}  // namespace

void remember(which step, int rc, const tof_l7::operation_status &status)
{
    if (!in_range(step))
        return;
    record &r{kept[static_cast<size_t>(step)]};
    r.attempted = true;
    r.rc = rc;
    r.failed_stage = status.failed_stage;
    r.port_errno = status.port_errno;
    r.argument_fault = status.argument_fault;
    r.uld_status = status.uld_status;
    r.sample_present = status.sample_present;
}

const record &last(which step)
{
    /* An out-of-range step answers with a never-attempted record rather than reading past the
     * array: a reader must not be handed a neighbouring step's observation. */
    static const record nothing{};
    return in_range(step) ? kept[static_cast<size_t>(step)] : nothing;
}

void begin_round(bool accepted)
{
    if (!accepted)
        return;
    forget_all();
    ++rounds;
}

unsigned rounds_begun()
{
    return rounds;
}

void forget_all()
{
    for (size_t i = 0; i < static_cast<size_t>(which::count); ++i)
        kept[i] = record{};
}

const char *name(which step)
{
    switch (step) {
    case which::close:
        return "close";
    case which::open:
        return "open";
    case which::configure:
        return "configure";
    case which::start:
        return "start";
    case which::read:
        return "read";
    case which::count:
        break;
    }
    /* No default, so adding a step is a compile error here rather than an unnamed line. */
    return "unnamed";
}

}  // namespace lexxhard::tof_l7_steps

#endif  // ENABLE_TOF_L7_ULD
