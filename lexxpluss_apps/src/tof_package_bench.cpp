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
 *     there is nothing running between commands that an operator did not ask for.
 */

#include <errno.h>
#include <stdlib.h>

#if defined(ENABLE_TOF_PACKAGE_BENCH)

#if !defined(ENABLE_TOF_L7_ULD)
#error "ENABLE_TOF_PACKAGE_BENCH requires ENABLE_TOF_L7_ULD"
#endif

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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

/* ---- hooks: thin, and each one maps to exactly one layer below ---- */

int hook_verified()
{
    return tof_l7_runtime::ensure_verified();
}

int hook_probe(void *, uint8_t addr_7bit, bool *present)
{
    if (present == nullptr)
        return -EINVAL;
    *present = false;

    /* A probe must not leave the sensor open, so it runs on a scratch configuration of its own and
     * uses only the ULD calls that need an address. Zeroed every time for the same reason the
     * chain's pass zeroes its scratch: a probe that inherited a previous session's object would be
     * asking about state rather than about the bus. */
    VL53L7CX_Configuration probe_dev{};
    probe_dev.platform.port = &port_ctx;
    probe_dev.platform.address = static_cast<uint16_t>(addr_7bit) << 1;

    vl53l7cx_port_clear_error(&port_ctx);
    uint8_t answered{0};
    const uint8_t uld_status{vl53l7cx_is_alive(&probe_dev, &answered)};

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
    return v.rc;
}

int hook_quiesce(void *, uint8_t addr_7bit)
{
    /* Stops a device that answered before this run began, using calls that need only an address.
     * Deliberately NOT tof_l7::stop(), which requires a lifecycle this sensor object does not have
     * for a session we did not open. */
    VL53L7CX_Configuration probe_dev{};
    probe_dev.platform.port = &port_ctx;
    probe_dev.platform.address = static_cast<uint16_t>(addr_7bit) << 1;

    vl53l7cx_port_clear_error(&port_ctx);
    const uint8_t uld_status{vl53l7cx_stop_ranging(&probe_dev)};
    if (const int fault{vl53l7cx_port_argument_fault()}; fault != 0)
        return fault;
    if (const int rc{vl53l7cx_port_error(&port_ctx)}; rc != 0)
        return rc;
    return uld_status == VL53L7CX_STATUS_OK ? 0 : -EIO;
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

    tof_l7::sample out{};
    tof_l7::operation_status st{};
    const int rc{tof_l7::read_once(&sensor_, &scratch_, &out, &st)};
    if (rc != 0)
        return rc;
    *fresh = out.fresh;
    return 0;
}

int hook_stop(void *)
{
    tof_l7::operation_status st{};
    return tof_l7::stop(&sensor_, &st);
}

int bind(const struct shell *sh, uint8_t frequency_hz)
{
    const struct device *const bus{DEVICE_DT_GET(DT_NODELABEL(i2c4))};

    if (!device_is_ready(bus)) {
        shell_error(sh, "i2c4 is not ready; nothing was bound");
        return -ENODEV;
    }

    port_ctx = vl53l7cx_port_context{};
    port_ctx.bus = bus;
    vl53l7cx_port_clear_error(&port_ctx);
    sensor_ = tof_l7::sensor{};
    scratch_ = tof_l7::scratch{};

    tof_package::hooks h{};
    h.ensure_verified = hook_verified;
    h.probe = hook_probe;
    h.quiesce = hook_quiesce;
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

    lifecycle_ = tof_package::lifecycle{};
    if (const int rc{lifecycle_.init(h, c)}; rc != 0) {
        shell_error(sh, "lifecycle init refused: %d", rc);
        bound_ = false;
        return rc;
    }
    bound_ = true;
    return 0;
}

void report(const struct shell *sh)
{
    const tof_package::status s{lifecycle_.current()};

    shell_print(sh, "state %s  step %s  errno %d  survivor %s  stop_attempts %u",
                tof_package::state_name(s.current), tof_package::step_name(s.failed_step),
                s.errno_value, s.survivor_found ? "yes" : "no", s.stop_attempts_used);
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
        if (lifecycle_.current().failed_step == tof_package::step::probe)
            shell_warn(sh, "the probe did not complete, so presence is UNKNOWN. Nothing was "
                           "opened on purpose: this sensor has no reset line, and opening on top "
                           "of a device that may be ranging needs a machine power cycle to undo.");
        return rc;
    }
    shell_print(sh, "ranging");
    return 0;
}

int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
    if (!bound_) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    const unsigned long polls{argc > 1 ? strtoul(argv[1], nullptr, 10) : 1UL};

    if (polls < 1UL || polls > 200UL) {
        shell_error(sh, "usage: pkg_tof read [polls 1-200]");
        return -EINVAL;
    }

    unsigned fresh_count{0};
    for (unsigned long i{0}; i < polls; ++i) {
        bool fresh{false};
        if (const int rc{lifecycle_.read_once(&fresh)}; rc != 0) {
            shell_error(sh, "read failed after %u fresh frames: %d", fresh_count, rc);
            report(sh);
            return rc;
        }
        if (fresh)
            ++fresh_count;
    }
    shell_print(sh, "%lu polls, %u fresh", polls, fresh_count);
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
    SHELL_CMD_ARG(read, NULL, "read [polls=1] - poll for complete grids", cmd_read, 1, 1),
    SHELL_CMD(stop, NULL, "stop ranging; retryable while the debt stands", cmd_stop),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pkg_tof, &sub_pkg_tof, "Package ToF bench driver (BENCH BUILD ONLY)", NULL);

}  // namespace
}  // namespace lexxhard::tof_package_bench

#endif  // ENABLE_TOF_PACKAGE_BENCH
