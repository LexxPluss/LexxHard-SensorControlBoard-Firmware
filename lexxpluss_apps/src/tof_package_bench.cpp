/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Bench shell commands use tof_package_device as the sole sensor owner; the
 * build excludes automatic runtime. They verify an existing blob and accept
 * explicit CAN IDs, but do not provision, move the shutter or start threads.
 * ULD scratch stays off the measured shell stack.
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#if defined(ENABLE_TOF_PACKAGE_BENCH)

#if !defined(ENABLE_TOF_L7_ULD)
#error "ENABLE_TOF_PACKAGE_BENCH requires ENABLE_TOF_L7_ULD"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "tof_l7_quiesce.hpp"
#include "tof_l7_step_record.hpp"
#include "tof_l7_runtime.hpp"
#include "tof_l7_sensor.hpp"
#include "tof_package_bench_deadman.hpp"
#include "tof_package_can.hpp"
#include "tof_package_device.hpp"
#include "tof_package_l7.hpp"
#include "tof_package_stream.hpp"

LOG_MODULE_REGISTER(tof_package_bench);

namespace lexxhard::tof_package_bench {
namespace {

void report(const struct shell *sh)
{
    const tof_package::status s{tof_package_device::lifecycle().current()};

    /* Report the entry on every run: up stops a survivor; takeover does not. */
    shell_print(sh, "state %s  step %s  errno %d  survivor %s  stop_attempts %u  entry %s",
                tof_package::state_name(s.current), tof_package::step_name(s.failed_step),
                s.errno_value, s.survivor_found ? "yes" : "no", s.stop_attempts_used,
                s.entered_by_takeover ? "takeover" : "up");
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
    /* THE REFUSAL IS PRINTED HERE, not in the device layer. Each of these says something different
     * to an operator, and two of them share an errno, so the device answers which one it was and
     * this command turns that into the sentence the operator needs. */
    tof_package_device::bind_refusal why{tof_package_device::bind_refusal::none};
    if (const int rc{tof_package_device::bind(static_cast<uint8_t>(hz), &why)}; rc != 0) {
        switch (why) {
        case tof_package_device::bind_refusal::session_owned:
            shell_error(sh, "refused: the current lifecycle still owns a session. Nothing was changed.");
            report(sh);
            shell_warn(sh, "stop it first. If `stop` has already spent its budget this tool cannot "
                           "clear it -- report it; a re-bind would only hide it.");
            break;
        case tof_package_device::bind_refusal::bus_not_ready:
            shell_error(sh, "i2c4 is not ready; nothing was bound");
            break;
        case tof_package_device::bind_refusal::close_refused:
            shell_error(sh, "refused: the sensor will not close (%d). Nothing was changed.", rc);
            break;
        case tof_package_device::bind_refusal::lifecycle_refused:
            shell_error(sh, "lifecycle init refused: %d", rc);
            break;
        case tof_package_device::bind_refusal::none:
            shell_error(sh, "bind failed: %d", rc);
            break;
        }
        return rc;
    }

    shell_print(sh, "bound: i2c4, address 0x29, %lu Hz. Nothing opened yet.", hz);
    return 0;
}

/* The body both entries share. `takeover` picks which lifecycle entry runs and nothing else: the
 * clearing rule, the report and every message below have to be the same for both, and a second copy
 * is how they would stop being. */
int run_round(const struct shell *sh, bool takeover)
{
    if (!tof_package_device::bound()) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    /* Device layer owns the round boundary for both shell and runtime callers. */
    const int rc{tof_package_device::begin_round(takeover)};

    report(sh);
    if (rc != 0) {
        shell_error(sh, "%s failed: %d", takeover ? "takeover" : "bring_up", rc);
        if (rc == -EPERM)
            shell_warn(sh, "a round starts only from idle. Nothing was probed, no blob was "
                           "consulted and no hook ran, so whatever put the lifecycle in its "
                           "current state is still recorded. Do not re-bind to clear it.");
        /* Named explicitly because it is the case the lifecycle exists for, and a bench operator
         * reading only the errno would miss what it means. */
        /* Named explicitly because these are the cases the lifecycle exists for, and a bench
         * operator reading only the errno would miss what each one means. */
        switch (tof_package_device::lifecycle().current().failed_step) {
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

int cmd_up(const struct shell *sh, size_t, char **)
{
    return run_round(sh, false);
}

int cmd_takeover(const struct shell *sh, size_t, char **)
{
    /* Warn before takeover: it skips stop confirmation and is neither retry nor cold start. */
    shell_warn(sh, "taking over a device whose state is UNKNOWN: the stop that `up` requires is "
                   "skipped, and nothing here establishes that the sensor lost power or stopped "
                   "ranging. The init that follows is what will have changed it.");
    return run_round(sh, true);
}

/* A 2 ms readiness poll yields the shell thread without masking grid cadence. */
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

    /* index counts accepted grids here; the L7 provides no sequence number. */
    shell_print(sh, "frame %u  +%lld ms  %u/64 zones  %u..%u mm  %d C", index, interval_ms, targets,
                targets == 0U ? 0U : low, high, static_cast<int>(s.silicon_temperature_degc));
}

void show_grid(const struct shell *sh, const tof_l7::sample &s)
{
    /* Print distances and raw ST status; this tool does not classify zones. */
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

/* Count complete grids, not polls; print intervals and content. */
int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
    if (!tof_package_device::bound()) {
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
        if (const int rc{tof_package_device::lifecycle().read_once(&fresh)}; rc != 0) {
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
        show_frame(sh, seen, now - previous, tof_package_device::last_sample());
        previous = now;
    }

    shell_print(sh, "%lu frames in %lld ms over %lu polls", frames, k_uptime_get() - started,
                polls);
    show_grid(sh, tof_package_device::last_sample());
    return 0;
}

int cmd_stop(const struct shell *sh, size_t, char **)
{
    if (!tof_package_device::bound()) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    const int rc{tof_package_device::lifecycle().stop()};

    report(sh);
    if (rc != 0) {
        shell_error(sh, "stop failed: %d", rc);
        /* REPORTED FROM THE RECORDED STATE BY ONE TESTED FUNCTION, not chosen here. This branched
         * inline on the state and got the worst case wrong twice: a survivor refusal was reported
         * as a spent budget, and the attempt that spends the budget was told to retry. */
        const tof_package::stop_report what{tof_package::report_for(tof_package_device::lifecycle().current())};
        if (what.retryable)
            shell_warn(sh, "%s", what.text);
        else
            shell_error(sh, "%s", what.text);
        return rc;
    }
    return 0;
}

const char *quiesce_layer_name(tof_l7_quiesce::layer which)
{
    switch (which) {
    case tof_l7_quiesce::layer::ok:
        return "ok";
    case tof_l7_quiesce::layer::image_fault:
        return "image_fault";
    case tof_l7_quiesce::layer::transport:
        return "transport";
    case tof_l7_quiesce::layer::handshake:
        return "handshake_timeout";
    case tof_l7_quiesce::layer::device_status:
        return "device_status";
    }
    /* No default above, so adding a layer is a compile error here rather than a silent "unknown". */
    return "unnamed";
}

const char *go2_exit_name(tof_l7_quiesce::go2_exit e)
{
    switch (e) {
    case tof_l7_quiesce::go2_exit::not_polled:
        return "not_polled";
    case tof_l7_quiesce::go2_exit::confirmed:
        return "confirmed";
    case tof_l7_quiesce::go2_exit::exhausted:
        return "exhausted";
    }
    /* No default, for the same reason as the layer name above. */
    return "unnamed";
}

/* Print unread registers as `--`, distinct from a valid zero. */
void go2_byte(char *out, size_t len, bool valid, uint8_t value)
{
    if (valid)
        snprintk(out, len, "0x%02x", value);
    else
        snprintk(out, len, "--");
}

int cmd_status(const struct shell *sh, size_t, char **)
{
    shell_print(sh, "bound %s", tof_package_device::bound() ? "yes" : "no");
    if (tof_package_device::bound())
        report(sh);

    /* Distinguish no quiesce attempt from a successful zero-valued observation. */
    if (const tof_l7_quiesce::record &q{tof_l7_quiesce::last()}; q.taken) {
        shell_print(sh, "quiesce #%u: %s  rc %d  uld_status 0x%02x  port_errno %d  completed %u  "
                        "%u us",
                    q.sequence, quiesce_layer_name(q.which), q.rc, q.raw.uld_status,
                    q.raw.port_errno, q.raw.completed, q.elapsed_us);
        /* Report observed bytes after the stop poll, without changing its timing. */
        char first_s[5];
        char last_s[5];
        char after_s[5];
        char status1_s[5];
        go2_byte(first_s, sizeof(first_s), q.go2.first_valid, q.go2.first);
        go2_byte(last_s, sizeof(last_s), q.go2.last_valid, q.go2.last);
        go2_byte(after_s, sizeof(after_s), q.go2.after_loop_valid, q.go2.after_loop);
        go2_byte(status1_s, sizeof(status1_s), q.go2.status1_valid, q.go2.status1);
        shell_print(sh, "  observed GO2 status: exit %s  polls %u (%u read)  first %s  last %s  "
                        "after_loop %s  status1 %s  bit7 %s",
                    go2_exit_name(q.go2.exit), q.go2.polls, q.go2.valid_polls, first_s, last_s,
                    after_s, status1_s, q.go2.saw_mcu_stop ? "seen" : "never seen");
        shell_print(sh, "  an observation of GO2 status, not a verdict about the device: the same "
                        "register's bit 0 ends the MCU boot wait AFTER a firmware download, so no "
                        "value here establishes a power history.");
        if (q.which == tof_l7_quiesce::layer::handshake)
            shell_print(sh, "  every transfer succeeded and the MCU stop was never confirmed. The "
                            "duration alone does not say this -- a failing transport spends the "
                            "same five seconds -- the port errno of 0 does.");
        else if (q.which == tof_l7_quiesce::layer::transport)
            shell_print(sh, "  the port recorded a transport error, so the ULD status byte is not "
                            "interpreted. Look at the bus before the device.");
    } else {
        shell_print(sh, "quiesce: none recorded since boot");
    }

    /* WHOSE STEPS THESE ARE. An `up` the lifecycle refuses clears nothing -- that refusal is what
     * used to destroy the previous failure -- so the records below belong to the last round that
     * BEGAN, which need not be the last command typed. */
    if (const unsigned rounds{tof_l7_steps::rounds_begun()}; rounds == 0U)
        shell_print(sh, "steps: no bring-up has begun since boot");
    else
        shell_print(sh, "steps: from bring-up #%u", rounds);

    /* EVERY STEP, INCLUDING THE ONES THAT DID NOT RUN. Printed in bring-up order so a reader sees
     * where the run stopped without having to know the order. */
    for (uint8_t i = 0; i < static_cast<uint8_t>(tof_l7_steps::which::count); ++i) {
        const tof_l7_steps::which step{static_cast<tof_l7_steps::which>(i)};
        const tof_l7_steps::record &s{tof_l7_steps::last(step)};
        if (!s.attempted) {
            shell_print(sh, "step %-9s not called", tof_l7_steps::name(step));
            continue;
        }
        shell_print(sh, "step %-9s rc %d  stage %s  port_errno %d  argument_fault %d  "
                        "uld_status 0x%02x  sample %s",
                    tof_l7_steps::name(step), s.rc, tof_l7::stage_name(s.failed_stage), s.port_errno,
                    s.argument_fault, s.uld_status, s.sample_present ? "yes" : "no");
    }

    const tof_l7_runtime::snapshot blob{tof_l7_runtime::current()};

    shell_print(sh, "blob stage %s, firmware %s, %zu bytes",
                tof_l7_runtime::stage_name(blob.current_stage),
                blob.firmware_available ? "available" : "unavailable", blob.firmware_size);

    /* THE WAY BACK TO PRODUCTION, PRINTED EVERY TIME. An operator has to be able to see, without
     * remembering anything, whether this image still reverts on its own and how long is left. */
    namespace deadman = tof_package_bench_deadman;
    if (deadman::image_confirmed()) {
        shell_error(sh, "image is CONFIRMED and will NOT revert. Nothing in this image confirms "
                        "one, so this is unexpected -- report it; only a re-flash removes it.");
    } else if (deadman::armed()) {
        shell_print(sh, "UNCONFIRMED: whole-machine cold reboot in %u s reverts this image to "
                        "production (%u of %u extensions used; `pkg_tof hold` adds more)",
                    deadman::remaining_ms() / 1000U, deadman::extensions_used(),
                    deadman::kMaxExtensions);
    } else {
        shell_warn(sh, "UNCONFIRMED but the unattended revert is NOT armed: a wedge that keeps the "
                       "watchdog fed would need a manual power cycle. Expected only before main() "
                       "has finished starting the subsystems.");
    }

    shell_print(sh, "NOTE: this tool never provisions the blob, publishes no CAN and moves no "
                    "shutter, and it cannot confirm this image.");
    return 0;
}

int cmd_hold(const struct shell *sh, size_t argc, char **argv)
{
    namespace deadman = tof_package_bench_deadman;
    const unsigned long minutes{argc > 1 ? strtoul(argv[1], nullptr, 10) : deadman::kWindowMinutes};

    /* MANUAL, BOUNDED, AND NOT TIED TO ACTIVITY. A window that renewed itself whenever a command ran
     * would never fire during the session it exists for -- one where an operator is busy and the
     * machine is in a state nobody intended. */
    const int rc{deadman::extend(static_cast<uint32_t>(minutes))};
    if (rc == -EPERM) {
        shell_error(sh, "all %u extensions are spent. This image reverts in %u s; re-flashing it is "
                        "a decision to take again, not one to extend into.",
                    deadman::kMaxExtensions, deadman::remaining_ms() / 1000U);
        return rc;
    }
    if (rc == -EALREADY) {
        shell_error(sh, "image is confirmed; there is no revert to extend");
        return rc;
    }
    if (rc != 0) {
        shell_error(sh, "usage: pkg_tof hold [minutes 1-%u]", deadman::kWindowMinutes);
        return rc;
    }
    shell_print(sh, "revert re-armed: cold reboot in %lu min (%u of %u extensions used)", minutes,
                deadman::extensions_used(), deadman::kMaxExtensions);
    return 0;
}

/* Require the operator to name boot-local numbering; it repeats after reset. */
constexpr const char *kBootLocalSource{"boot-local"};

/* Long enough to catch a 10 Hz talker twice over and short enough that nobody skips the check.
 * What it can and cannot establish is in tof_package_can.hpp -- silence is not proof of ownership.
 */
constexpr int32_t kListenWindowMs{200};

/* Stream fresh grids on explicit 11-bit IDs after the traffic check. Ending
 * the command stops publication, not ranging; consumers detect later silence. */
int cmd_stream(const struct shell *sh, size_t argc, char **argv)
{
    if (!tof_package_device::bound()) {
        shell_error(sh, "run `pkg_tof bind` first");
        return -EPERM;
    }

    if (argc < 5) {
        shell_error(sh, "usage: pkg_tof stream <data_id> <health_id> %s <grids> [budget_ms]",
                    kBootLocalSource);
        return -EINVAL;
    }

    tof_package_can::identifiers ids{};
    ids.data_id = strtoul(argv[1], nullptr, 0);
    ids.health_id = strtoul(argv[2], nullptr, 0);
    ids.extended = false;

    if (strcmp(argv[3], kBootLocalSource) != 0) {
        shell_error(sh, "the sequence source must be named, and `%s` is the only one here: numbers "
                        "come from RAM and restart at 1 after any reset, so they repeat. That is "
                        "not candidate N's sequence. A durable source needs a commissioned journal, "
                        "which this board does not have.",
                    kBootLocalSource);
        return -EINVAL;
    }

    const unsigned long grids{strtoul(argv[4], nullptr, 10)};
    const unsigned long budget_ms{argc > 5 ? strtoul(argv[5], nullptr, 10) : 10000UL};
    if (grids < 1UL || grids > 2000UL || budget_ms < 10UL || budget_ms > 600000UL) {
        shell_error(sh, "usage: pkg_tof stream <data_id> <health_id> %s <grids 1-2000> "
                        "[budget_ms 10-600000]",
                    kBootLocalSource);
        return -EINVAL;
    }

    if (const int rc{tof_package_can::init()}; rc != 0) {
        shell_error(sh, "can2 is not available: %d", rc);
        return rc;
    }
    if (const int rc{tof_package_can::configure(ids)}; rc != 0) {
        shell_error(sh, "identifiers 0x%x/0x%x refused: zero means unset, the two must differ, and "
                        "both must fit 11 bits",
                    ids.data_id, ids.health_id);
        return rc;
    }

    uint32_t heard{0};
    if (const int rc{tof_package_can::listen_for_traffic(kListenWindowMs, &heard)}; rc != 0) {
        shell_error(sh, "could not listen on 0x%x/0x%x: %d", ids.data_id, ids.health_id, rc);
        return rc;
    }
    if (heard != 0U) {
        shell_error(sh, "%u frames arrived on 0x%x/0x%x in %d ms. Somebody is already using these "
                        "identifiers; nothing was sent.",
                    heard, ids.data_id, ids.health_id, kListenWindowMs);
        return -EADDRINUSE;
    }

    shell_print(sh, "data 0x%x, health 0x%x, 11-bit, source %s", ids.data_id, ids.health_id,
                kBootLocalSource);
    shell_print(sh, "silent for %d ms on both: nobody transmitted while we listened. That is all "
                    "it means -- this command takes whatever pair you give it, so check the two "
                    "numbers above against the registry yourself.",
                kListenWindowMs);
    shell_warn(sh, "sequence numbers are boot-local and repeat after a reset. This stream is an "
                   "observation of the path, not evidence that candidate N's identity holds.");

    /* Keep sequence numbering across stream commands; only a board reset restarts it.
     * Reinitialize the publisher's per-run counters, not the boot's sequence source.
     */
    tof_package_stream::sequence_source source{};
    source.issue = tof_package_device::issue_boot_local;
    if (const int rc{tof_package_device::publisher().init(tof_package_can::sink(), source)}; rc != 0) {
        shell_error(sh, "publisher refused the wiring: %d", rc);
        return rc;
    }

    /* Report never_started once when not ranging; -EPERM alone is ambiguous. */
    if (tof_package_device::lifecycle().current().current != tof_package::state::ranging) {
        tof_package_stream::input nothing{};
        nothing.outcome = tof_package_health::acquisition::never_started;
        tof_package_stream::outcome what{};
        (void)tof_package_device::publisher().publish(nothing, &what);
        shell_error(sh, "state is %s, not ranging: sent one `never started` diagnostic (%s) and no "
                        "measurement. Run `pkg_tof up` or `pkg_tof takeover` first.",
                    tof_package::state_name(tof_package_device::lifecycle().current().current),
                    tof_package_stream::outcome_name(what));
        return -EPERM;
    }

    const int64_t started{k_uptime_get()};
    int64_t previous{started};
    unsigned long polls{0};
    unsigned long published{0};

    while (published < grids) {
        if (k_uptime_get() - started > static_cast<int64_t>(budget_ms)) {
            shell_error(sh, "budget expired: %lu of %lu grids in %lu ms over %lu polls", published,
                        grids, budget_ms, polls);
            break;
        }

        bool fresh{false};
        ++polls;
        const int read_rc{tof_package_device::lifecycle().read_once(&fresh)};

        tof_package_stream::input in{};
        if (read_rc == 0 && fresh) {
            in.outcome = tof_package_health::acquisition::complete_grid;
            in.distance_mm = tof_package_device::last_sample().distance_mm;
            in.status = tof_package_device::last_sample().target_status;
            in.zone_count = tof_package_stream::kZones;
        } else if (read_rc == 0) {
            /* Not ready yet. Nothing is published for a poll that found nothing: a readiness poll is
             * not an acquisition, and sending a diagnostic for each one would bury the stream in
             * frames that report no event. */
            k_msleep(kPollIntervalMs);
            continue;
        } else {
            /* A transport or device failure. The grid in tof_package_device::last_sample() may be perfectly good and is
             * deliberately NOT passed: `failed` is what happened to THIS acquisition, and the
             * publisher's rule is what keeps the held grid off the bus. */
            in.outcome = tof_package_health::acquisition::failed;
        }

        tof_package_stream::outcome what{};
        const int publish_rc{tof_package_device::publisher().publish(in, &what)};
        const int64_t now{k_uptime_get()};

        if (publish_rc != 0) {
            shell_error(sh, "publish refused after %lu grids: %s (%d)", published,
                        tof_package_stream::outcome_name(what), publish_rc);
            break;
        }

        if (what == tof_package_stream::outcome::grid_published) {
            ++published;
            shell_print(sh, "grid %lu  seq %u  +%lld ms", published, tof_package_device::publisher().current().last_sequence,
                        now - previous);
            previous = now;
            continue;
        }

        /* Anything else ends the run and says which rule stopped it. Carrying on would spend
         * sequence numbers and bus time on a fault the operator has not seen yet. */
        shell_error(sh, "stopped after %lu grids: %s (read rc %d)", published,
                    tof_package_stream::outcome_name(what), read_rc);
        break;
    }

    const tof_package_stream::counters c{tof_package_device::publisher().current()};
    shell_print(sh, "%lu published, %u abandoned, %u diagnostics, %u mailbox refusals, last seq %u, "
                    "%u numbers issued this boot, %lu polls in %lld ms",
                published, c.grids_abandoned, c.diagnostics_sent, c.send_failures, c.last_sequence,
                tof_package_device::boot_sequence().issued(), polls, k_uptime_get() - started);
    /* Separate mailbox refusals from later transmit completion failures. */
    shell_print(sh, "%u transmit completions failed (frames still queued have reported nothing)",
                tof_package_can::transmit_errors());
    shell_print(sh, "ranging is untouched; `pkg_tof stop` when done. Nothing further is sent on "
                    "these identifiers, so a consumer should now report the stream as stale.");

    return published == grids ? 0 : -EIO;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pkg_tof,
    SHELL_CMD(status, NULL, "lifecycle state, failed step and blob verification", cmd_status),
    SHELL_CMD_ARG(bind, NULL, "bind [hz=10] - bind the lifecycle to i2c4 @0x29; opens nothing",
                  cmd_bind, 1, 1),
    SHELL_CMD(takeover, NULL,
              "take over an UNKNOWN device state: `up` without the quiesce, from idle only",
              cmd_takeover),
    SHELL_CMD(up, NULL, "ensure_verified, probe, quiesce a survivor, open, configure, start",
              cmd_up),
    SHELL_CMD_ARG(read, NULL,
                  "read [frames=8] [budget_ms=5000] - wait for N complete grids, print each",
                  cmd_read, 1, 2),
    SHELL_CMD(stop, NULL, "stop ranging; retryable while the debt stands", cmd_stop),
    SHELL_CMD_ARG(stream, NULL,
                  "stream <data_id> <health_id> boot-local <grids> [budget_ms] - publish grids as "
                  "candidate N frames on can2. Identifiers stay arguments so a bench run can use "
                  "a pair that is NOT the allocation; the sequence source is named because it is "
                  "boot-local",
                  cmd_stream, 5, 1),
    SHELL_CMD_ARG(hold, NULL,
                  "hold [minutes] - extend the unattended revert; bounded, and no confirm exists",
                  cmd_hold, 1, 1),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pkg_tof, &sub_pkg_tof, "Package ToF bench driver (BENCH BUILD ONLY)", NULL);

}  // namespace
}  // namespace lexxhard::tof_package_bench

#endif  // ENABLE_TOF_PACKAGE_BENCH
