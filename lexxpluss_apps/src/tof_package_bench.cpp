/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * BENCH ONLY. The real Package lifecycle, bound to a real device on I2C4 and driven from the
 * shell. Compiled only under ENABLE_TOF_PACKAGE_BENCH, which no product configuration sets.
 *
 * WHY IT EXISTS AT ALL. tof_package_l7 has no caller, so --gc-sections drops it and every size
 * measured so far says nothing about what it costs. Something has to call the path before the
 * linked cost is a number rather than a guess, and that something must not be a product entry
 * point: binding it into one would make a bring-up decision that belongs to the Package
 * configuration, which does not exist yet.
 *
 * WHAT IT DELIBERATELY DOES NOT DO, each because it is a decision this file is not entitled to
 * make:
 *
 *   - it does not provision the blob. It calls tof_l7_runtime::ensure_verified() and reports what
 *     it was told. An unprovisioned board is a refusal to read, not a cue to write 86 KB into
 *     storage_partition from a shell command.
 *   - it publishes nothing on CAN. The wire layout is undecided -- the drafted payload does not
 *     fit the existing grid header -- and a bench tool inventing one would freeze it by accident.
 *   - it touches no shutter and no door state. FW-5 is a safety interlock whose entry conditions
 *     are an open requirements question; a tool that moved the shutter would be answering it.
 *   - it starts no thread and no timer. Every command runs on the shell's stack and returns, so
 *     there is nothing THIS MODULE started running between commands. That is a claim about this
 *     module only: the image around it is the ordinary firmware, and the threads main() brings up --
 *     CAN, the shutter, the rest -- are running throughout. An earlier wording read as though the
 *     image were otherwise quiet, which it is not.
 *
 * STACK. Every command runs on the shell thread, and sizeof(VL53L7CX_Configuration) is 2,324 B
 * against a 2,048 B default shell stack: one object overruns it before any call chain is counted.
 * So nothing here puts a configuration, a scratch or a sample on the stack, and the shell stack is
 * sized against a measured figure in overlays/tof_package_bench.conf. See `addressed()` below.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#if defined(ENABLE_TOF_PACKAGE_BENCH)

#if !defined(ENABLE_TOF_L7_ULD)
#error "ENABLE_TOF_PACKAGE_BENCH requires ENABLE_TOF_L7_ULD"
#endif

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

extern "C" {
#include "vl53l7cx_api.h"
}
#include "vl53l7cx_port.h"

#include "tof_l7_probe_verdict.hpp"
#include "tof_l7_runtime.hpp"
#include "tof_l7_sensor.hpp"
#include "tof_package_l7.hpp"

LOG_MODULE_REGISTER(tof_package_bench);

namespace lexxhard::tof_package_bench {
namespace {

/* THE ONE PLACE THIS CONFIGURATION NAMES A BUS. The port takes its controller from the context, so
 * choosing I2C4 is a wiring decision and this is where it is made and visible. J22, PF14/PF15,
 * through the COM16988 differential board. */
vl53l7cx_port_context port_ctx{};

tof_l7::sensor sensor_;
tof_l7::scratch scratch_;
tof_package::lifecycle lifecycle_;
bool bound_{false};

/* THE ADDRESS-ONLY SCRATCH CONFIGURATION, IN .bss AND NOT ON THE STACK.
 *
 * sizeof(VL53L7CX_Configuration) is 2,324 B -- it carries the ULD's temporary buffer -- against a
 * 2,048 B default shell stack. Declaring one as a local, which is what the first version of
 * hook_probe() and hook_quiesce() each did, overruns the shell stack with a single object, before
 * any of the ULD's own frames are counted. The same mistake one layer down already cost this
 * project a boot loop: see overlays/tof_l7.conf.
 *
 * ONE object serves both hooks, because the shell runs one command at a time to completion and
 * neither hook retains anything past its own return. It is zeroed on every use for the reason the
 * chain's recovery scratch is: a probe must ask about the bus, not about a previous session. And it
 * is deliberately NOT sensor_, whose object belongs to a lifecycle -- an address-only call made on
 * it would write over a session's state. */
VL53L7CX_Configuration addressed_scratch_;

/* The last complete grid, kept so `read` can show what arrived rather than only how many did. In
 * .bss for the stack reason above, and because the content has to outlive the hook that filled it. */
tof_l7::sample last_sample_;

VL53L7CX_Configuration *addressed(uint8_t addr_7bit)
{
    memset(&addressed_scratch_, 0, sizeof(addressed_scratch_));
    addressed_scratch_.platform.port = &port_ctx;
    addressed_scratch_.platform.address = static_cast<uint16_t>(addr_7bit) << 1;
    return &addressed_scratch_;
}

/* ---- hooks: thin, and each one maps to exactly one layer below ---- */

int hook_verified()
{
    return tof_l7_runtime::ensure_verified();
}

int hook_probe(void *, uint8_t addr_7bit, bool *present, bool *wrong_identity)
{
    if (present == nullptr || wrong_identity == nullptr)
        return -EINVAL;
    *present = false;
    *wrong_identity = false;

    /* A probe must not leave the sensor open, so it runs on the address-only scratch and uses only
     * the ULD calls that need an address. */
    VL53L7CX_Configuration *const probe_dev{addressed(addr_7bit)};

    vl53l7cx_port_clear_error(&port_ctx);
    uint8_t answered{0};
    const uint8_t uld_status{vl53l7cx_is_alive(probe_dev, &answered)};

    /* The shared rule, not a second copy of it. See tof_l7_probe_verdict. */
    const tof_l7_probe::inputs in{
        .argument_fault = vl53l7cx_port_argument_fault(),
        .port_errno = vl53l7cx_port_error(&port_ctx),
        .completed = vl53l7cx_port_completed_transfers(&port_ctx),
        .uld_ok = uld_status == VL53L7CX_STATUS_OK,
        .identity_matched = answered != 0U,
    };
    const tof_l7_probe::verdict v{tof_l7_probe::classify(in)};

    *present = v.present;
    *wrong_identity = v.answered_without_matching_identity;
    return v.rc;
}

int hook_quiesce(void *, uint8_t addr_7bit)
{
    /* Stops a device that answered before this run began, using calls that need only an address.
     * Deliberately NOT tof_l7::stop(), which requires a lifecycle this sensor object does not have
     * for a session we did not open. */
    VL53L7CX_Configuration *const probe_dev{addressed(addr_7bit)};

    vl53l7cx_port_clear_error(&port_ctx);
    const uint8_t uld_status{vl53l7cx_stop_ranging(probe_dev)};
    if (const int fault{vl53l7cx_port_argument_fault()}; fault != 0)
        return fault;
    if (const int rc{vl53l7cx_port_error(&port_ctx)}; rc != 0)
        return rc;
    return uld_status == VL53L7CX_STATUS_OK ? 0 : -EIO;
}

int hook_close(void *)
{
    /* The real close, not `sensor_ = {}`. It refuses a sensor that is running or owes a stop --
     * exactly the state the assignment would have erased -- and from `empty` it succeeds and does
     * nothing, which is what lets the lifecycle call it before every open. */
    tof_l7::operation_status st{};
    return tof_l7::close(&sensor_, &st);
}

int hook_open(void *, uint8_t addr_7bit)
{
    tof_l7::operation_status st{};
    return tof_l7::open(&sensor_, &port_ctx, addr_7bit, &st);
}

int hook_configure(void *, uint8_t frequency_hz)
{
    tof_l7::operation_status st{};
    return tof_l7::configure(&sensor_, frequency_hz, &st);
}

int hook_start(void *)
{
    tof_l7::operation_status st{};
    return tof_l7::start(&sensor_, &st);
}

int hook_read(void *, bool *fresh)
{
    if (fresh == nullptr)
        return -EINVAL;
    *fresh = false;

    /* Into the .bss sample rather than a local: 260 B of stack this path does not need to spend,
     * and the content has to survive the hook for `read` to show it. read_once() clears it on entry
     * and leaves it non-fresh on every refusal, so a stale grid cannot be reported as a new one. */
    tof_l7::operation_status st{};
    const int rc{tof_l7::read_once(&sensor_, &scratch_, &last_sample_, &st)};
    if (rc != 0)
        return rc;
    *fresh = last_sample_.fresh;
    return 0;
}

int hook_stop(void *)
{
    tof_l7::operation_status st{};
    return tof_l7::stop(&sensor_, &st);
}

void report(const struct shell *sh)
{
    const tof_package::status s{lifecycle_.current()};

    shell_print(sh, "state %s  step %s  errno %d  survivor %s  stop_attempts %u",
                tof_package::state_name(s.current), tof_package::step_name(s.failed_step),
                s.errno_value, s.survivor_found ? "yes" : "no", s.stop_attempts_used);
}

int bind(const struct shell *sh, uint8_t frequency_hz)
{
    /* RE-BINDING OVER A SESSION NOBODY HAS ACCOUNTED FOR IS REFUSED, and refused FIRST, before a
     * single byte is written. The earlier version reset port_ctx, sensor_ and lifecycle_
     * unconditionally, so a second `bind` silently erased `ranging`, an unpaid stop debt, or a
     * blocked-after-the-budget state. None of those is bookkeeping: each is the only record that a
     * device may still be measuring, and the way back from forgetting that on this sensor is a
     * machine power cycle.
     *
     * Two independent guards, because one of them is bookkeeping and the other is not: the
     * lifecycle's own answer, which knows whether anything was ever opened; and tof_l7::close()
     * below, which refuses `running` and `stop_unconfirmed` from the sensor's own state and
     * therefore holds even if the bookkeeping above it were wrong. */
    if (bound_ && !lifecycle_.discardable()) {
        shell_error(sh, "refused: the current lifecycle still owns a session. Nothing was changed.");
        report(sh);
        shell_warn(sh, "stop it first. If `stop` has already spent its budget this tool cannot "
                       "clear it -- report it; a re-bind would only hide it.");
        return -EPERM;
    }

    const struct device *const bus{DEVICE_DT_GET(DT_NODELABEL(i2c4))};

    if (!device_is_ready(bus)) {
        shell_error(sh, "i2c4 is not ready; nothing was bound");
        return -ENODEV;
    }

    /* Closed before the context is replaced rather than assigned over. close() does no transport
     * I/O, so running it against the outgoing context is safe, and it is the second refusal above. */
    {
        tof_l7::operation_status st{};
        if (const int rc{tof_l7::close(&sensor_, &st)}; rc != 0) {
            shell_error(sh, "refused: the sensor will not close (%d). Nothing was changed.", rc);
            return rc;
        }
    }

    port_ctx = vl53l7cx_port_context{};
    port_ctx.bus = bus;
    vl53l7cx_port_clear_error(&port_ctx);
    scratch_ = tof_l7::scratch{};
    last_sample_ = tof_l7::sample{};

    tof_package::hooks h{};
    h.ensure_verified = hook_verified;
    h.probe = hook_probe;
    h.quiesce = hook_quiesce;
    h.close = hook_close;
    h.open = hook_open;
    h.configure = hook_configure;
    h.start = hook_start;
    h.read = hook_read;
    h.stop = hook_stop;
    h.ctx = nullptr;

    tof_package::config c{};
    c.address_7bit = 0x29;
    c.frequency_hz = frequency_hz;
    c.stop_attempts = 3;

    /* RE-INITIALISED IN PLACE, NOT REPLACED. `lifecycle_ = {}` was the other half of the problem
     * above: it discards the object rather than asking it, so the refusal the lifecycle now carries
     * would never have been reached. init() resets its own state when it accepts, and refuses with
     * -EPERM when the object still owns a session -- which is the same question the guard at the top
     * of this function asks, answered by the one object that can answer it. */
    if (const int rc{lifecycle_.init(h, c)}; rc != 0) {
        shell_error(sh, "lifecycle init refused: %d", rc);
        bound_ = false;
        return rc;
    }
    bound_ = true;
    return 0;
}

int cmd_bind(const struct shell *sh, size_t argc, char **argv)
{
    /* The rate is the caller's: the sustainable one for this geometry has not been accepted, and a
     * default here would quietly become that answer. */
    const unsigned long hz{argc > 1 ? strtoul(argv[1], nullptr, 10) : 10UL};

    if (hz < 1UL || hz > 15UL) {
        shell_error(sh, "usage: pkg_tof bind [hz 1-15]");
        return -EINVAL;
    }
    if (const int rc{bind(sh, static_cast<uint8_t>(hz))}; rc != 0)
        return rc;

    shell_print(sh, "bound: i2c4, address 0x29, %lu Hz. Nothing opened yet.", hz);
    return 0;
}

int cmd_up(const struct shell *sh, size_t, char **)
{
    if (!bound_) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    const int rc{lifecycle_.bring_up()};

    report(sh);
    if (rc != 0) {
        shell_error(sh, "bring_up failed: %d", rc);
        /* Named explicitly because it is the case the lifecycle exists for, and a bench operator
         * reading only the errno would miss what it means. */
        /* Named explicitly because these are the cases the lifecycle exists for, and a bench
         * operator reading only the errno would miss what each one means. */
        switch (lifecycle_.current().failed_step) {
        case tof_package::step::probe:
            shell_warn(sh, "the probe did not complete, so presence is UNKNOWN. Nothing was "
                           "opened on purpose: this sensor has no reset line, and opening on top "
                           "of a device that may be ranging needs a machine power cycle to undo.");
            break;
        case tof_package::step::identity:
            shell_warn(sh, "something ANSWERED at 0x29 and is not a VL53L7CX. Nothing was stopped "
                           "and nothing was opened: an L7 stop sequence and an 86 KB firmware "
                           "download do not belong to an unidentified device. Check what is on "
                           "i2c4 before running this again.");
            break;
        case tof_package::step::firmware:
            if (rc == -EINPROGRESS)
                shell_warn(sh, "the blob verification is in progress elsewhere; nothing was "
                               "touched. Run `up` again.");
            else
                shell_warn(sh, "the stored blob is not usable. This tool does not provision it.");
            break;
        default:
            break;
        }
        return rc;
    }
    shell_print(sh, "ranging");
    return 0;
}

/* Between readiness polls. The device publishes a complete grid at the configured rate -- 66 ms
 * apart at 15 Hz, 100 ms at 10 Hz -- so a tight loop spends hundreds of I2C transactions per frame
 * and never yields the shell thread. 2 ms is short enough that the measured interval still reflects
 * the device rather than the sampling, and long enough to leave the bus and the CPU alone. */
constexpr int32_t kPollIntervalMs{2};

void show_frame(const struct shell *sh, unsigned index, int64_t interval_ms,
                const tof_l7::sample &s)
{
    unsigned targets{0};
    uint16_t low{0xffffU};
    uint16_t high{0};

    for (size_t i{0}; i < tof_l7::kZoneCount; ++i) {
        if (s.target_count[i] == 0U)
            continue;
        ++targets;
        if (s.distance_mm[i] < low)
            low = s.distance_mm[i];
        if (s.distance_mm[i] > high)
            high = s.distance_mm[i];
    }

    /* `index` IS THIS TOOL'S OWN COUNT, not a device sequence number -- the L7 result carries none.
     * So it establishes how many complete grids this command accepted and in what order, and it
     * cannot by itself tell a genuinely new frame from a repeat. The interval and the content are
     * what speak to that, which is why both are printed. */
    shell_print(sh, "frame %u  +%lld ms  %u/64 zones  %u..%u mm  %d C", index, interval_ms, targets,
                targets == 0U ? 0U : low, high, static_cast<int>(s.silicon_temperature_degc));
}

void show_grid(const struct shell *sh, const tof_l7::sample &s)
{
    /* The whole last grid, distance and status, because a frame count says nothing about whether
     * the content is a measurement. Status is printed raw: the per-code meaning is ST's, this file
     * has no accepted classification table for the L7, and inventing one here would turn a bench
     * reading into a judgement nobody has agreed. */
    shell_print(sh, "last grid, mm per zone then raw target_status (0 targets prints as -----):");
    for (size_t r{0}; r < 8U; ++r) {
        const size_t o{r * 8U};
        shell_fprintf(sh, SHELL_NORMAL, "  mm ");
        for (size_t c{0}; c < 8U; ++c) {
            if (s.target_count[o + c] == 0U)
                shell_fprintf(sh, SHELL_NORMAL, "----- ");
            else
                shell_fprintf(sh, SHELL_NORMAL, "%5u ", s.distance_mm[o + c]);
        }
        shell_fprintf(sh, SHELL_NORMAL, "  st ");
        for (size_t c{0}; c < 8U; ++c)
            shell_fprintf(sh, SHELL_NORMAL, "%3u ", s.target_status[o + c]);
        shell_fprintf(sh, SHELL_NORMAL, "\n");
    }
}

/* ACCEPTANCE IS A FRESH-FRAME COUNT AGAINST A DEADLINE, not a poll count.
 *
 * The first version of this command took a number of polls and reported how many came back fresh.
 * Two things were wrong with that. 200 tight polls against a 10 Hz device complete in a few
 * milliseconds, so "200 polls, 0 fresh" was the expected outcome of a perfectly healthy sensor and
 * said nothing; and the sample was thrown away, so the command could not distinguish a stream of
 * complete grids from a stream of complete zeroes.
 *
 * So the caller asks for N complete grids within a budget. Reaching N is the pass; the deadline is
 * what turns a silent device into a bounded failure instead of a hang; and each frame prints its
 * interval and its content so the rate and the data are both inspectable. */
int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
    if (!bound_) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    const unsigned long frames{argc > 1 ? strtoul(argv[1], nullptr, 10) : 8UL};
    const unsigned long budget_ms{argc > 2 ? strtoul(argv[2], nullptr, 10) : 5000UL};

    if (frames < 1UL || frames > 64UL || budget_ms < 10UL || budget_ms > 60000UL) {
        shell_error(sh, "usage: pkg_tof read [frames 1-64] [budget_ms 10-60000]");
        return -EINVAL;
    }

    const int64_t started{k_uptime_get()};
    int64_t previous{started};
    unsigned long polls{0};
    unsigned seen{0};

    while (seen < frames) {
        if (k_uptime_get() - started > static_cast<int64_t>(budget_ms)) {
            shell_error(sh, "budget expired: %u of %lu frames in %lu ms over %lu polls", seen,
                        frames, budget_ms, polls);
            report(sh);
            /* Ranging is left alone on purpose. The device may simply be slower than the budget,
             * and stopping it here would destroy the state an operator needs to look at. */
            return -ETIMEDOUT;
        }

        bool fresh{false};
        ++polls;
        if (const int rc{lifecycle_.read_once(&fresh)}; rc != 0) {
            shell_error(sh, "read failed after %u frames and %lu polls: %d", seen, polls, rc);
            report(sh);
            return rc;
        }
        if (!fresh) {
            k_msleep(kPollIntervalMs);
            continue;
        }

        const int64_t now{k_uptime_get()};
        ++seen;
        show_frame(sh, seen, now - previous, last_sample_);
        previous = now;
    }

    shell_print(sh, "%lu frames in %lld ms over %lu polls", frames, k_uptime_get() - started,
                polls);
    show_grid(sh, last_sample_);
    return 0;
}

int cmd_stop(const struct shell *sh, size_t, char **)
{
    if (!bound_) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    const int rc{lifecycle_.stop()};

    report(sh);
    if (rc != 0) {
        shell_error(sh, "stop failed: %d", rc);
        if (lifecycle_.current().current == tof_package::state::stop_owed)
            shell_warn(sh, "the stop was not acknowledged. Retry it: reads and a reopen stay "
                           "refused until one succeeds.");
        else if (lifecycle_.current().current == tof_package::state::blocked)
            shell_error(sh, "the retry budget is spent. The device may still be measuring and "
                            "this tool cannot stop it; report it rather than re-binding.");
        return rc;
    }
    return 0;
}

int cmd_status(const struct shell *sh, size_t, char **)
{
    shell_print(sh, "bound %s", bound_ ? "yes" : "no");
    if (bound_)
        report(sh);

    const tof_l7_runtime::snapshot blob{tof_l7_runtime::current()};

    shell_print(sh, "blob stage %s, firmware %s, %zu bytes",
                tof_l7_runtime::stage_name(blob.current_stage),
                blob.firmware_available ? "available" : "unavailable", blob.firmware_size);
    shell_print(sh, "NOTE: this tool never provisions the blob, publishes no CAN and moves no "
                    "shutter.");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pkg_tof,
    SHELL_CMD(status, NULL, "lifecycle state, failed step and blob verification", cmd_status),
    SHELL_CMD_ARG(bind, NULL, "bind [hz=10] - bind the lifecycle to i2c4 @0x29; opens nothing",
                  cmd_bind, 1, 1),
    SHELL_CMD(up, NULL, "ensure_verified, probe, quiesce a survivor, open, configure, start",
              cmd_up),
    SHELL_CMD_ARG(read, NULL,
                  "read [frames=8] [budget_ms=5000] - wait for N complete grids, print each",
                  cmd_read, 1, 2),
    SHELL_CMD(stop, NULL, "stop ranging; retryable while the debt stands", cmd_stop),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pkg_tof, &sub_pkg_tof, "Package ToF bench driver (BENCH BUILD ONLY)", NULL);

}  // namespace
}  // namespace lexxhard::tof_package_bench

#endif  // ENABLE_TOF_PACKAGE_BENCH
