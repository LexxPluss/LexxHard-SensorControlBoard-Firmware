/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Single-owner device objects and hooks; policy lives above this layer.
 * Keep configuration, scratch and samples in .bss: one 2,324 B configuration
 * already exceeds a default 2,048 B stack. Size each caller's stack separately.
 */

#include <errno.h>
#include <string.h>

#include "tof_package_device.hpp"

#if defined(ENABLE_TOF_PACKAGE_BENCH) || defined(ENABLE_TOF_PACKAGE_AUTO)

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

extern "C" {
#include "vl53l7cx_api.h"
}
#include "vl53l7cx_port.h"

#include "runtime_progress.hpp"
#include "tof_l7_probe_verdict.hpp"
#include "tof_l7_quiesce.hpp"
#include "tof_l7_runtime.hpp"
#include "tof_l7_step_record.hpp"
#include "tof_watchdog_feeder.hpp"

namespace lexxhard::tof_package_device {
namespace {


/* THE ONE PLACE THIS CONFIGURATION NAMES A BUS. The port takes its controller from the context, so
 * choosing I2C4 is a wiring decision and this is where it is made and visible. J22, PF14/PF15,
 * through the COM16988 differential board. */
vl53l7cx_port_context port_ctx{};

tof_l7::sensor sensor_;
tof_l7::scratch scratch_;
tof_package::lifecycle lifecycle_;
bool bound_{false};

/* Keep address-only scratch in .bss; zero it per call without touching sensor_. */
VL53L7CX_Configuration addressed_scratch_;

/* Patch 0005 writes here. Keep this separate from scratch: each is cleared at a different point. */
struct vl53l7cx_go2_observation go2_scratch_;

/* The last complete grid, kept so `read` can show what arrived rather than only how many did. In
 * .bss for the stack reason above, and because the content has to outlive the hook that filled it. */
tof_l7::sample last_sample_;

/* Keep stream counters in .bss for post-run status; stream init resets them per run. */
tof_package_stream::publisher publisher_;
tof_seq_boot_local::source boot_sequence_;

/* Fail at compile time if sample and wire zone counts diverge. */
static_assert(tof_l7::kZoneCount == tof_package_stream::kZones,
              "the sensor's grid and the wire's grid must have the same number of zones: "
              "every caller hands last_sample_'s arrays to the publisher with the wire's count, "
              "and a smaller sample would be read past its end");

VL53L7CX_Configuration *addressed(uint8_t addr_7bit)
{
    memset(&addressed_scratch_, 0, sizeof(addressed_scratch_));
    addressed_scratch_.platform.port = &port_ctx;
    /* AFTER the memset, which would otherwise clear the pointer it is being given. */
    addressed_scratch_.platform.go2 = &go2_scratch_;
    addressed_scratch_.platform.address = static_cast<uint16_t>(addr_7bit) << 1;
    return &addressed_scratch_;
}

/* ---- what the watchdog can see of this thread ---- */

/* Scope L7 progress to each ULD call, including the waiting phase. This does not
 * monitor silence between calls or blob verification. */
struct watched_uld_operation {
    watched_uld_operation() { runtime_progress::begin(runtime_progress::activity::l7); }
    ~watched_uld_operation() { runtime_progress::end(runtime_progress::activity::l7); }
    watched_uld_operation(const watched_uld_operation &) = delete;
    watched_uld_operation &operator=(const watched_uld_operation &) = delete;
};

/* Stop polling took 5.43 s on this board, beyond the ordinary 5 s L7 bound.
 * Declare only quiesce as long-running; probe, open and start keep the 5 s bound.
 * The declaration has a 30 s cap and also suspends chain-cycle checks, so keep
 * its scope narrow. It is a flag, not a nesting-safe count: custom builds that
 * combine Package with commissioning must revisit this. */
struct declared_long_operation {
    declared_long_operation() { tof_watchdog_feeder::long_operation_begin(); }
    ~declared_long_operation() { tof_watchdog_feeder::long_operation_end(); }
    declared_long_operation(const declared_long_operation &) = delete;
    declared_long_operation &operator=(const declared_long_operation &) = delete;
};

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

    const watched_uld_operation watched{};

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

/* The timeout code is pinned on both sides rather than trusted on one: the pure classifier cannot
 * include the vendor header, so it names the value and this is where the two are compared. */
static_assert(tof_l7_quiesce::kUldTimeout == VL53L7CX_STATUS_TIMEOUT_ERROR,
              "the pure classifier's timeout code no longer matches the ULD's");
static_assert(tof_l7_quiesce::kUldOk == VL53L7CX_STATUS_OK,
              "the pure classifier's OK code no longer matches the ULD's");

/* Copy vendor observations into the testable record. Exhaustive exit mapping makes new
 * vendor values a compile error. */
tof_l7_quiesce::go2_observation observed_go2(const struct vl53l7cx_go2_observation &raw)
{
    tof_l7_quiesce::go2_exit exit{tof_l7_quiesce::go2_exit::not_polled};
    switch (raw.exit_reason) {
    case VL53L7CX_GO2_EXIT_NOT_POLLED:
        exit = tof_l7_quiesce::go2_exit::not_polled;
        break;
    case VL53L7CX_GO2_EXIT_CONFIRMED:
        exit = tof_l7_quiesce::go2_exit::confirmed;
        break;
    case VL53L7CX_GO2_EXIT_EXHAUSTED:
        exit = tof_l7_quiesce::go2_exit::exhausted;
        break;
    }
    return tof_l7_quiesce::go2_observation{
        .first = raw.first,
        .last = raw.last,
        .after_loop = raw.after_loop,
        .status1 = raw.status1,
        .exit = exit,
        .first_valid = raw.first_valid != 0U,
        .last_valid = raw.last_valid != 0U,
        .after_loop_valid = raw.after_loop_valid != 0U,
        .status1_valid = raw.status1_valid != 0U,
        .saw_mcu_stop = raw.saw_mcu_stop != 0U,
        .polls = raw.polls,
        .valid_polls = raw.valid_polls,
    };
}

int hook_quiesce(void *, uint8_t addr_7bit)
{
    /* Declared before the bracket and released after it, so the declaration covers the whole time
     * the operation is in flight and the sample that follows it. */
    const declared_long_operation declared{};
    const watched_uld_operation watched{};
    /* Stops a device that answered before this run began, using calls that need only an address.
     * Deliberately NOT tof_l7::stop(), which requires a lifecycle this sensor object does not have
     * for a session we did not open. */
    VL53L7CX_Configuration *const probe_dev{addressed(addr_7bit)};

    vl53l7cx_port_clear_error(&port_ctx);
    /* Use a 64-bit clock: k_cycle_get_32() wraps in 19.884 s at 216 MHz. */
    const int64_t started_us{k_ticks_to_us_floor64(k_uptime_ticks())};
    const uint8_t uld_status{vl53l7cx_stop_ranging(probe_dev)};
    const uint32_t elapsed_us{
        tof_l7_quiesce::span_us(started_us, k_ticks_to_us_floor64(k_uptime_ticks()))};

    /* Record every return path, including success. Snapshot sticky port errno and
     * transfer count immediately after this read, before another operation runs.
     */
    const tof_l7_quiesce::inputs observed{
        .argument_fault = vl53l7cx_port_argument_fault(),
        .port_errno = vl53l7cx_port_error(&port_ctx),
        .completed = vl53l7cx_port_completed_transfers(&port_ctx),
        .uld_status = uld_status,
    };
    /* Taken here with everything else from this call: the patch clears the structure at the start
     * of the next stop sequence, so reading it later would be reading whatever ran after. */
    tof_l7_quiesce::remember(observed, elapsed_us, observed_go2(go2_scratch_));
    return tof_l7_quiesce::last().rc;
}

int hook_close(void *)
{
    const watched_uld_operation watched{};
    /* The real close, not `sensor_ = {}`. It refuses a sensor that is running or owes a stop --
     * exactly the state the assignment would have erased -- and from `empty` it succeeds and does
     * nothing, which is what lets the lifecycle call it before every open. */
    tof_l7::operation_status st{};
    const int rc{tof_l7::close(&sensor_, &st)};
    tof_l7_steps::remember(tof_l7_steps::which::close, rc, st);
    return rc;
}

int hook_open(void *, uint8_t addr_7bit)
{
    /* The ULD firmware download lives inside this call and is the longest single operation on the
     * path -- the two seconds bounds::l7_ms was chosen to cover. */
    const watched_uld_operation watched{};
    /* A refused open leaves port counters from the previous operation; record from st. */
    tof_l7::operation_status st{};
    const int rc{tof_l7::open(&sensor_, &port_ctx, addr_7bit, &st)};
    tof_l7_steps::remember(tof_l7_steps::which::open, rc, st);
    return rc;
}

int hook_configure(void *, uint8_t frequency_hz)
{
    const watched_uld_operation watched{};
    tof_l7::operation_status st{};
    const int rc{tof_l7::configure(&sensor_, frequency_hz, &st)};
    tof_l7_steps::remember(tof_l7_steps::which::configure, rc, st);
    return rc;
}

int hook_start(void *)
{
    const watched_uld_operation watched{};
    tof_l7::operation_status st{};
    const int rc{tof_l7::start(&sensor_, &st)};
    tof_l7_steps::remember(tof_l7_steps::which::start, rc, st);
    return rc;
}

int hook_read(void *, bool *fresh)
{
    if (fresh == nullptr)
        return -EINVAL;
    *fresh = false;

    const watched_uld_operation watched{};
    /* Into the .bss sample rather than a local: 260 B of stack this path does not need to spend,
     * and the content has to survive the hook for `read` to show it. read_once() clears it on entry
     * and leaves it non-fresh on every refusal, so a stale grid cannot be reported as a new one. */
    tof_l7::operation_status st{};
    const int rc{tof_l7::read_once(&sensor_, &scratch_, &last_sample_, &st)};
    /* Recorded before the early return, so a failed read is in the record too -- `sample_present`
     * is the field that says whether anything came, and it is the sensor layer's answer. */
    tof_l7_steps::remember(tof_l7_steps::which::read, rc, st);
    if (rc != 0)
        return rc;
    *fresh = last_sample_.fresh;
    return 0;
}

int hook_stop(void *)
{
    const watched_uld_operation watched{};
    tof_l7::operation_status st{};
    return tof_l7::stop(&sensor_, &st);
}

}  // namespace

const char *bind_refusal_name(bind_refusal r)
{
    switch (r) {
    case bind_refusal::none:
        return "none";
    case bind_refusal::session_owned:
        return "session_owned";
    case bind_refusal::bus_not_ready:
        return "bus_not_ready";
    case bind_refusal::close_refused:
        return "close_refused";
    case bind_refusal::lifecycle_refused:
        return "lifecycle_refused";
    }
    return "unnamed";
}

bool bound()
{
    return bound_;
}

tof_package::lifecycle &lifecycle()
{
    return lifecycle_;
}

const tof_l7::sample &last_sample()
{
    return last_sample_;
}

tof_package_stream::publisher &publisher()
{
    return publisher_;
}

tof_seq_boot_local::source &boot_sequence()
{
    return boot_sequence_;
}

int issue_boot_local(void *, uint32_t *out)
{
    return boot_sequence_.issue(out);
}

int begin_round(bool by_takeover)
{
    /* Clear prior step evidence only after the lifecycle accepts a new round. */
    tof_l7_steps::begin_round(lifecycle_.can_begin_round());

    return by_takeover ? lifecycle_.take_over_unknown_state() : lifecycle_.bring_up();
}

int bind(uint8_t frequency_hz, bind_refusal *why)
{
    /* Reject rebind before touching a live session or unpaid stop debt. */
    if (why != nullptr)
        *why = bind_refusal::none;

    if (bound_ && !lifecycle_.discardable()) {
        if (why != nullptr)
            *why = bind_refusal::session_owned;
        return -EPERM;
    }

    const struct device *const bus{DEVICE_DT_GET(DT_NODELABEL(i2c4))};

    if (!device_is_ready(bus)) {
        if (why != nullptr)
            *why = bind_refusal::bus_not_ready;
        return -ENODEV;
    }

    /* Closed before the context is replaced rather than assigned over. close() does no transport
     * I/O, so running it against the outgoing context is safe, and it is the second refusal above. */
    {
        tof_l7::operation_status st{};
        if (const int rc{tof_l7::close(&sensor_, &st)}; rc != 0) {
            if (why != nullptr)
                *why = bind_refusal::close_refused;
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

    /* Reinitialize in place so lifecycle init can refuse a live session. */
    if (const int rc{lifecycle_.init(h, c)}; rc != 0) {
        if (why != nullptr)
            *why = bind_refusal::lifecycle_refused;
        bound_ = false;
        return rc;
    }
    bound_ = true;
    return 0;
}
}  // namespace lexxhard::tof_package_device

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO
