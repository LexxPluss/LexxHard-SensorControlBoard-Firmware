/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Bringing a surviving L7 back to a known state at boot, over I2C, because nothing else can.
 *
 * THE HARDWARE FACT THIS EXISTS FOR. The two models on the enable chain answer an enable-low
 * differently, and the difference is measured rather than assumed -- see tof_enumerator.hpp. The
 * L4 board's enable is reset-class (XSHUT), so an all-off returns every cliff sensor to its
 * factory state. The L7 board's enable GATES COMMS ONLY and leaves the sensor powered, and
 * neither an XSHUT line nor any power-rail control for an L7 appears anywhere in the devicetree.
 * So an SCB-only reset -- a software reset, a watchdog reset, or the reboot at the end of a DFU --
 * restarts the STM32 and leaves both grid sensors running with whatever state they had.
 *
 * WHAT THAT COSTS TODAY. After any run that opened an L7, the next boot re-enumerates the chain as
 * if from cold and commissioning does not finish: scb_attempts_exhausted, roles 255, 0x216 at zero
 * (2026-09-22, out-l7-diag3/MANIFEST.txt). The only recovery anybody has is a full battery power
 * cycle. An image that needs an operator to pull power before it can see its own sensors is not
 * deliverable, whatever else it does.
 *
 * WHY A SOFTWARE RECOVERY IS POSSIBLE AT ALL. The SCB's copy of the ULD's device object does not
 * survive the SCB's own reset, so the obvious objection is that the driver cannot address a sensor
 * it holds no state for. It can. vl53l7cx_stop_ranging touches p_dev->platform -- on this port an
 * I2C address and nothing else -- plus the single byte is_auto_stop_enabled, and vl53l7cx_is_alive
 * touches only the platform. Neither reads anything a previous boot left in RAM. A zeroed
 * configuration carrying the survivor's address is therefore a legal argument to both, and that is
 * the entire basis of this module.
 *
 * WHY THE PROBE IS NOT OPTIONAL. With is_auto_stop_enabled zero -- which is what a freshly zeroed
 * configuration carries -- stop_ranging takes its "provoke MCU stop" path and polls for up to five
 * seconds before giving up. Two sensors is ten seconds added to every boot, most of which are cold
 * boots with nothing to recover. So a sensor is probed first and stopped only if it answers, and
 * the probe is the thing that keeps this pass free on the boots where it has no work.
 *
 * WHEN IT RUNS. Before the all-off, not after it. The flip-flops that carry the enable chain are
 * powered from the rail the reset does not drop, so a survivor is still enabled and still sitting
 * at its programmed address at the moment the application starts -- the same persistence that
 * causes the problem is what makes the survivor addressable before anything has been touched.
 * After an all-off the L7 is merely silent, which is worse: it still holds its state and can no
 * longer be told anything. That ordering is an assumption about the carrier, stated here so that a
 * pass which reports every position absent on a warm reset is read as evidence against it rather
 * than as a quiet success.
 *
 * WHAT IT DOES NOT ESTABLISH. That stopping a ranging session clears whatever state actually
 * survives. What survives is not known: a downloaded ULD, an active session, a latched address and
 * an unfinished transaction all fit what has been observed, and nothing so far tells them apart.
 * This module acts on the one of those a running SCB can act on, and reports per sensor so that a
 * boot which recovered and a boot which merely did not need to are distinguishable in the log
 * rather than by inference.
 *
 * WHAT IT IS NOT. Not an authority. It never decides whether the chain is usable; enumeration's
 * freeze rules keep that job, and a failure here is reported and passed on rather than acted upon.
 * A recovery pass that could stop a boot would be a second opinion on chain health, and the one
 * thing this project has repeatedly paid for is two places believing different things about the
 * same chain.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace lexxhard::tof_l7_recovery {

/* The product chain carries exactly two grid sensors. The constant is here rather than shared with
 * the chain spec because this pass is over ADDRESSES, not positions: it neither knows nor needs to
 * know where on the chain an address sits. */
inline constexpr size_t kMaxGridSensors{2};

enum class result : uint8_t {
    not_attempted,  // no address in this slot, or the pass refused its arguments
    absent,         // the address did not answer. On a cold boot this is the expected outcome for
                    //   every position, and it is the outcome that makes the pass cost nothing
    stopped,        // it answered and stop_ranging succeeded: a session was ended
    stop_failed,    // it answered and stop_ranging did not succeed
    probe_failed,   // the probe hit a transport error, so neither presence nor absence is known.
                    //   Distinct from absent on purpose: a bus that cannot answer is not a bus
                    //   that answered "nobody home"
};

/* Two calls, each of which the ULD can make from an address alone. Injected rather than called
 * directly so the logic is testable on a host with no bus, which is how every other pure layer in
 * this chain is built. */
struct ops {
    /* Sets *alive. Returns 0 when the probe itself completed, whatever its verdict; non-zero is a
     * transport error and *alive is then meaningless. */
    int (*is_alive)(void *ctx, uint8_t addr_7bit, bool *alive){nullptr};
    int (*stop_ranging)(void *ctx, uint8_t addr_7bit){nullptr};
    void *ctx{nullptr};
};

struct request {
    uint8_t addr_7bit[kMaxGridSensors]{};
    size_t count{0};
};

struct report {
    result at[kMaxGridSensors]{};
    size_t count{0};
    /* How many sensors were actually brought out of a session. Zero on a cold boot, and the number
     * worth logging: it is the only direct evidence that this boot had something to recover. */
    size_t stopped{0};
    /* Any probe or stop that did not do what it was asked. Never blocks the boot; see the header. */
    bool any_failure{false};
};

/* Refuses an ops with either call missing, or an address outside 0x08..0x77, by leaving the
 * affected slots not_attempted. Makes at most one probe and at most one stop per address, in the
 * order given, and never stops an address that did not answer. */
report run(const ops &o, const request &req);

const char *result_name(result r);

}  // namespace lexxhard::tof_l7_recovery
