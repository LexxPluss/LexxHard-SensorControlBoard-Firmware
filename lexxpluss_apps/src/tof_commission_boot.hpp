/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The one call that turns the commissioning downlink on, and the one place the bench's release
 * condition is decided.
 *
 * THE RELEASE CONDITION IS A BUILD DECISION, NOT A RUNTIME ONE. Whether this board may re-enumerate
 * its chain on request comes from TOF_AUTO_COMMISSION_PERMIT_ENUMERATION, which only the
 * firmware_auto_commission target defines. It is read ONCE, at boot, into a value nothing can change
 * afterwards -- so what a board will do is fixed by the image it is running and is visible in the
 * build command, rather than being a flag somebody can flip while a machine is moving.
 *
 * DEFAULT IS REFUSE. Without the define the hook answers false: no proof runs, on any request, and
 * the board answers `not_permitted`. A trustworthy stationary condition does not exist yet -- it is
 * an open item against safety -- and a default of `true` would be this file inventing one.
 *
 * WHAT THE PRODUCT AND CLIFF IMAGES GET: nothing. The whole file is behind
 * ENABLE_TOF_AUTO_COMMISSION, which only one target sets, and those images come out byte for byte as
 * they did before any of this existed -- measured, not argued.
 *
 * ORDER. Called after the cliff bootstrap, from main()'s one context, because that bootstrap is what
 * brings can2 up for this subsystem and the entropy driver is up long before main(). The binding
 * then decides what to install from what it finds: a session means a filter and a worker, no entropy
 * means a filter and no worker, and a refused configuration means neither.
 */

#pragma once

#include <stdint.h>

#if defined(ENABLE_TOF_AUTO_COMMISSION)

#include "tof_commission_bind.hpp"

namespace lexxhard::tof_commission_boot {

struct report {
    tof_commission_bind::outcome state{tof_commission_bind::outcome::refused};
    int rc{0};
    /* What was latched at boot, reported so a log or a shell command can say what this image will
     * do rather than what its defaults are. */
    bool permit_enumeration{false};
    bool profile_enabled{false};
    bool can_ready{false};
    bool entropy_ready{false};
    bool filter_installed{false};
    bool worker_started{false};
};

/* Once. A second call returns the first call's report and changes nothing: the release condition is
 * latched and the worker, once created, exists for the life of the process. */
report start();

/* What the last call decided, for diagnostics. */
report last();

} // namespace lexxhard::tof_commission_boot

#endif // ENABLE_TOF_AUTO_COMMISSION
