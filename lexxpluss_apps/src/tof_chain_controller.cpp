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

// Ownership rules for ENABLE_TOF_CHAIN builds:
//
//   - i2c2 belongs to the ToF chain. The tug encoder is the only other
//     i2c2 user; main.cpp initialises it as disconnected and never starts
//     its thread. The two features are mutually exclusive by construction.
//   - The diagnostic build owns the same resources a different way; the
//     two flags must never combine.
//   - Every use of the chain (this shell command today, the Phase 3
//     acquisition thread tomorrow) holds chain_lock() for its whole
//     session.

#ifdef ENABLE_TOF_CHAIN

#ifdef TOF_I2C_DIAG
#error "ENABLE_TOF_CHAIN and TOF_I2C_DIAG both claim i2c2 and the ToF shell: pick one"
#endif

#include <errno.h>

#include <atomic>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "tof_chain_controller.hpp"
#include "tof_chain_spec.hpp"
#if defined(ENABLE_TOF_CLIFF_ULD)
#include "tof_acquisition.hpp"
#include "tof_cliff_runtime.hpp"
#include "tof_commissioning.hpp"
#endif
#include "tof_enumerator.hpp"
#include "tof_readdress.hpp"

namespace lexxhard::tof_chain_controller {

LOG_MODULE_REGISTER(tof_chain);

namespace {

const struct device *const i2c2_dev{DEVICE_DT_GET(DT_NODELABEL(i2c2))};
const struct gpio_dt_spec data_pin = GPIO_DT_SPEC_GET(DT_PATH(tof_chain), data_gpios);
const struct gpio_dt_spec clock_pin = GPIO_DT_SPEC_GET(DT_PATH(tof_chain), clock_gpios);
constexpr uint32_t kDataSettleMs{DT_PROP(DT_PATH(tof_chain), data_settle_ms)};
constexpr uint32_t kSensorBootMs{DT_PROP(DT_PATH(tof_chain), sensor_boot_ms)};

K_MUTEX_DEFINE(chain_mutex);

// Written by init(), read by the shell thread: one atomic word. 0 means
// initialised; -EAGAIN means init has not run; any other negative errno is
// the failure. The commissioning command refuses to run on a chain whose
// control lines or bus never came up -- operating half-initialised hardware
// would produce verdicts that look like chain findings.
std::atomic<int> init_status{-EAGAIN};

// Explicit zero-length write, the same probe shape the diagnostics used;
// classification of the return code is the one-line glue rule.
int raw_probe(uint8_t addr7)
{
    uint8_t dummy{0};
    struct i2c_msg msg{&dummy, 0, I2C_MSG_WRITE | I2C_MSG_STOP};
    return i2c_transfer(i2c2_dev, &msg, 1, addr7);
}

// VL53 register access: 16-bit big-endian index, as in the ST ULD.
int vl53_rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len)
{
    uint8_t const index[2]{static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xff)};
    return i2c_write_read(i2c2_dev, addr7, index, sizeof index, buf, len);
}

int vl53_wr8(uint8_t addr7, uint16_t reg, uint8_t value)
{
    uint8_t const frame[3]{static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xff), value};
    return i2c_write(i2c2_dev, frame, sizeof frame, addr7);
}

// The production readdress helper's bus, tri-state end to end: its
// collision probe gets the same classified result the census gets, so a
// transport error can never read as "target free" anywhere in the stack.
struct real_bus final : tof_readdress::i2c_ops {
    tof_enum::probe_result probe(uint8_t addr7) override
    {
        return tof_enum::classify_probe_rc(raw_probe(addr7));
    }
    int wr8(uint8_t addr7, uint16_t reg, uint8_t value) override
    {
        return vl53_wr8(addr7, reg, value);
    }
    int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) override
    {
        return vl53_rd(addr7, reg, buf, len);
    }
};

struct zephyr_chain_ops final : tof_enum::chain_ops {
    tof_enum::probe_result probe(uint8_t addr7) override
    {
        return tof_enum::classify_probe_rc(raw_probe(addr7));
    }
    int read_id(tof_enum::model m, uint8_t addr7, tof_enum::id_bytes &out) override
    {
        // Staging and failure policy live in the shared, host-tested helper.
        real_bus bus{};
        return tof_readdress::read_id(m, bus, addr7, out);
    }
    tof_enum::readdress_result readdress(tof_enum::model m, uint8_t old7, uint8_t new7) override
    {
        real_bus bus{};
        return tof_readdress::readdress(m, bus, old7, new7);
    }
    int set_data(bool level) override
    {
        return gpio_pin_set_dt(&data_pin, level ? 1 : 0);
    }
    int pulse_clock() override
    {
        // Drive low FIRST: if an earlier failure left the line high, a
        // naive high-then-low "pulse" would produce no rising edge and
        // silently break the promise that a fresh run recovers the chain.
        if (int const rc{gpio_pin_set_dt(&clock_pin, 0)}; rc != 0)
            return rc;
        k_busy_wait(100);
        if (int const rc{gpio_pin_set_dt(&clock_pin, 1)}; rc != 0)
            return rc;
        k_busy_wait(100);
        if (int const rc{gpio_pin_set_dt(&clock_pin, 0)}; rc != 0)
            return rc;
        k_busy_wait(100);
        return 0;
    }
    void wait(wait_reason reason) override
    {
        k_msleep(reason == wait_reason::data_settle ? kDataSettleMs : kSensorBootMs);
    }
};

const char *outcome_name(tof_enum::outcome o)
{
    using tof_enum::outcome;
    switch (o) {
    case outcome::enumerated:              return "enumerated";
    case outcome::retained:                return "retained";
    case outcome::absent:                  return "absent";
    case outcome::ambiguous_identity:      return "ambiguous_identity";
    case outcome::unexpected_retained:     return "unexpected_retained";
    case outcome::unexpected_address:      return "unexpected_address";
    case outcome::verified_device_missing: return "verified_device_missing";
    case outcome::wrong_model:             return "wrong_model";
    case outcome::transport_failed:        return "transport_failed";
    case outcome::readdress_failed:        return "readdress_failed";
    case outcome::control_failed:          return "control_failed";
    case outcome::not_attempted:           return "not_attempted";
    }
    return "?";
}

const char *stage_name(tof_enum::readdress_stage s)
{
    using tof_enum::readdress_stage;
    switch (s) {
    case readdress_stage::none:         return "none";
    case readdress_stage::validate:     return "validate";
    case readdress_stage::collision:    return "collision";
    case readdress_stage::page_select:  return "page_select";
    case readdress_stage::addr_write:   return "addr_write";
    case readdress_stage::verify:       return "verify";
    case readdress_stage::page_restore: return "page_restore";
    }
    return "?";
}

const char *status_name(tof_enum::chain_status s)
{
    using tof_enum::chain_status;
    switch (s) {
    case chain_status::complete: return "COMPLETE";
    case chain_status::degraded: return "DEGRADED";
    case chain_status::failed:   return "FAILED";
    }
    return "?";
}

const char *control_stage_name(tof_enum::control_stage s)
{
    using tof_enum::control_stage;
    switch (s) {
    case control_stage::none:          return "none";
    case control_stage::data_low:      return "data_low";
    case control_stage::alloff_pulse:  return "alloff_pulse";
    case control_stage::data_high:     return "data_high";
    case control_stage::advance_pulse: return "advance_pulse";
    }
    return "?";
}

int cmd_enum(const struct shell *shell, size_t, char **)
{
    if (int const st{init_status.load()}; st != 0) {
        shell_error(shell, "chain glue not initialised (rc=%d): refusing to run", st);
        return -ENODEV;
    }
    if (k_mutex_lock(&chain_mutex, K_NO_WAIT) != 0) {
        shell_error(shell, "chain is busy (acquisition or another enum holds the lock)");
        return -EBUSY;
    }

    auto const spec{tof_chain::dasher_spec()};
    zephyr_chain_ops ops{};
    auto const r{tof_enum::enumerate(ops, spec)};
    k_mutex_unlock(&chain_mutex);

    shell_print(shell, "status: %s (spec_error=%d)", status_name(r.status),
                static_cast<int>(r.spec));
    shell_print(shell, "control: known=%d failed_at=%s rc=%d; data_high=%d pulses=%u",
                r.control_state_known, control_stage_name(r.control_failed_at),
                r.control_rc, r.data_commanded_high, r.pulses_issued);
    shell_print(shell, "frozen_at=%d interrupted_at=%d", r.frozen_at, r.interrupted_at);
    for (size_t k{0}; k < r.positions; ++k) {
        const auto &p{r.at[k]};
        shell_print(shell,
                    "pos%u: %s addr=0x%02x id=%02x/%02x rc=%d stage=%s offending=0x%02x "
                    "enable_cmd=%d",
                    static_cast<unsigned>(k + 1), outcome_name(p.verdict), p.address,
                    p.seen.first, p.seen.second, p.rc, stage_name(p.readdress.failed_at),
                    p.offending_addr, p.enable_commanded_high);
    }
    shell_print(shell, "source_allowed: right(0)=%d left(1)=%d",
                r.source_allowed[0], r.source_allowed[1]);
    shell_print(shell, "note: driver return-code hardware verification PENDING "
                       "(empty addr->nack, clamped SCL->transport, live->ack)");
    // Distinct exit codes so scripts cannot mistake a degraded chain for a
    // complete one: 0 only for COMPLETE, -ENODATA for DEGRADED, -EIO FAILED.
    if (r.status == tof_enum::chain_status::complete)
        return 0;
    return r.status == tof_enum::chain_status::degraded ? -ENODATA : -EIO;
}

#if defined(ENABLE_TOF_CLIFF_ULD)

const char *stage_label(tof_commissioning::stage st)
{
    using tof_commissioning::stage;
    switch (st) {
    case stage::none:               return "proven";
    case stage::not_configured:     return "not_configured";
    case stage::epoch_out_of_range: return "epoch_out_of_range";
    case stage::quiesce_failed:     return "quiesce_failed";
    case stage::chain_busy:         return "chain_busy";
    case stage::attempt_refused:    return "attempt_refused";
    case stage::evidence_refused:   return "evidence_refused";
    case stage::commit_refused:     return "commit_refused";
    }
    return "?";
}

int cmd_cliff_prove(const struct shell *shell, size_t argc, char **argv)
{
    if (int const st{init_status.load()}; st != 0) {
        shell_error(shell, "chain glue not initialised (rc=%d): refusing to run", st);
        return -ENODEV;
    }

    /* The epoch is mandatory and has no default. Under the commissioning profile the HOST owns it and
     * is the component that persists it; a firmware-invented value would be an epoch no operator
     * recorded. Parsed wide and range-checked by the orchestration, so 256 is refused rather than
     * truncated to a different epoch. */
    if (argc != 2) {
        shell_error(shell, "usage: tof cliff prove <host_epoch 0-255>");
        return -EINVAL;
    }
    char *end{nullptr};
    unsigned long const parsed{strtoul(argv[1], &end, 0)};
    if (end == argv[1] || *end != '\0' || parsed > 0xFFFFFFFFUL) {
        shell_error(shell, "bad epoch '%s'", argv[1]);
        return -EINVAL;
    }

    /* The bootstrap has to have finished, and saying WHICH step failed matters: "not ready" sends
     * an operator to look at the chain, which is the wrong place when the truth is that can2 never
     * came up or that this image never ran the bootstrap at all. */
    if (!tof_cliff_runtime::ready()) {
        shell_error(shell, "cliff runtime not ready (%s): refusing to commission",
                    tof_cliff_runtime::stage_name(tof_cliff_runtime::current_stage()));
        return -EPERM;
    }

    static zephyr_chain_ops ops{};
    tof_commissioning::config cfg{};
    cfg.chain = &chain_mutex;
    cfg.ops = &ops;
    /* THE spec, not a copy of it. The authority compares every proof against this same object; a
     * local static here would mean commissioning walked one chain description while the authority
     * checked the result against another. */
    cfg.spec = &tof_cliff_runtime::spec();
    /* The tested primitive itself, with no wrapper in between.
     *
     * try_stop(), not stop(): stop() takes the chain with K_FOREVER, so a wrapper around it would
     * block right here and the session's K_NO_WAIT acquire -- the whole reason a busy chain is a
     * refusal rather than a wait -- would never be reached. And try_stop(), not "try_stop plus a
     * check": is_idle() also takes the chain with K_FOREVER, so verifying the quiesce that way
     * would put the block back one line later.
     *
     * Pointing the hook straight at it is deliberate. A one-line wrapper here would be production
     * glue that no host suite links, i.e. exactly where a K_FOREVER could reappear unnoticed;
     * assigning the function under test leaves nothing to drift. It is also the right home for the
     * bounded join once there is an acquisition thread: quiescing is acquisition's business, not
     * the shell's. */
    cfg.quiesce = tof_acq::try_stop;
    if (int const rc{tof_commissioning::init(cfg)}; rc != 0) {
        shell_error(shell, "commissioning not configurable (%d)", rc);
        return rc;
    }

    auto const r{tof_commissioning::prove(static_cast<uint32_t>(parsed))};

    shell_print(shell, "result: %s", stage_label(r.failed_at));
    shell_print(shell, "detail: rc=%d begin=%d proof=%d commit=%d isolation_rc=%d", r.rc,
                static_cast<int>(r.begin), static_cast<int>(r.proof),
                static_cast<int>(r.commit), r.isolation_rc);
    shell_print(shell, "walk1: %s  walk2: %s", status_name(r.walk1.status),
                status_name(r.walk2.status));
    shell_print(shell, "isolation: attempted=%d answered=0x%02x prev=0x%02x id=%02x/%02x",
                r.isolation.attempted, r.isolation.answering_addr, r.isolation.prev_addr,
                r.isolation.seen.first, r.isolation.seen.second);

    if (!r.proven())
        return -EIO;

    /* The descriptors were keyed inside the commit, by the authority's install callback -- there is
     * no step here to do it, which is the point: a command that could key them separately could key
     * them from a mapping that was never proven, and a failure between the two used to leave a
     * PROVEN authority describing a different chain. Asserted rather than assumed, because "the
     * commit says it succeeded" and "the keys are the current mapping's" are different claims. */
    if (!tof_cliff_runtime::mapping_applied()) {
        shell_error(shell, "commit reported success but the descriptors are not keyed to it: "
                           "refusing to report a usable mapping");
        return -EIO;
    }

    /* Proven, keyed, and deliberately going no further. Starting acquisition is the next commit's
     * job -- there is no acquisition thread yet -- and the PROVEN clamp is still shut regardless, so
     * no measurement frame can leave this board even now. Saying so here keeps an operator from
     * reading "proven" as "producing". */
    shell_print(shell, "mapping installed under epoch %lu and descriptors keyed; acquisition NOT "
                       "started (no thread yet) and the PROVEN clamp is still in force",
                parsed);
    return 0;
}

int cmd_cliff_start(const struct shell *shell, size_t, char **)
{
    /* Separate from `prove` on purpose. Proving a mapping and starting to produce measurements are
     * two decisions, and an operator must be able to make the first without the second -- inspect
     * the proof, then start. A prove that started acquisition implicitly would also mean any
     * re-prove silently restarted production.
     *
     * Every refusal below comes from the runtime, not from re-checked conditions here: a command
     * that re-implemented the gate could disagree with it. */
    if (int const st{init_status.load()}; st != 0) {
        shell_error(shell, "chain glue not initialised (rc=%d)", st);
        return -ENODEV;
    }
    if (!tof_cliff_runtime::ready()) {
        shell_error(shell, "cliff runtime not ready (%s)",
                    tof_cliff_runtime::stage_name(tof_cliff_runtime::current_stage()));
        return -EPERM;
    }
    if (tof_acq::thread_running()) {
        shell_error(shell, "acquisition thread is already running; nothing to do");
        return -EALREADY;
    }
    if (!tof_cliff_runtime::mapping_applied()) {
        /* Either nothing was ever proven, or a later attempt revoked it. Both mean the descriptors
         * are not keyed to the mapping the authority currently reports, and a cycle would produce
         * facts nothing can be keyed by. */
        shell_error(shell, "no proven mapping is installed: run `tof cliff prove <epoch>` first");
        return -EPERM;
    }

    if (int const rc{tof_cliff_runtime::start_acquisition()}; rc != 0) {
        shell_error(shell, "acquisition refused to start (%d)", rc);
        return rc;
    }

    shell_print(shell, "acquisition thread started");
    /* Said explicitly, because "started" and "measurements are on the wire" are different claims
     * and only the clamp decides the second one. */
    shell_print(shell, "measurement frames leave this board only if the PROVEN clamp is lifted; "
                       "health frames were already flowing since boot");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof_cliff,
    SHELL_CMD(start, NULL,
              "start the acquisition thread (requires a proven, installed mapping)",
              cmd_cliff_start),
    SHELL_CMD_ARG(prove, NULL,
                  "prove the cliff mapping: stop acquisition, walk -> isolate -> walk, install "
                  "under <host_epoch 0-255>",
                  cmd_cliff_prove, 2, 0),
    SHELL_SUBCMD_SET_END
);

#endif  // ENABLE_TOF_CLIFF_ULD

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof,
    SHELL_CMD(enum, NULL,
              "manual commissioning: enumerate the ToF chain (holds the chain lock)",
              cmd_enum),
#if defined(ENABLE_TOF_CLIFF_ULD)
    SHELL_CMD(cliff, &sub_tof_cliff, "cliff mapping commissioning", NULL),
#endif
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(tof, &sub_tof, "ToF chain commands", NULL);

}  // namespace

k_mutex &chain_lock()
{
    return chain_mutex;
}

bool glue_ready()
{
    return init_status.load() == 0;
}

void init()
{
    if (!device_is_ready(i2c2_dev)) {
        LOG_ERR("i2c2 not ready");
        init_status.store(-ENODEV);
        return;
    }
    if (!gpio_is_ready_dt(&data_pin) || !gpio_is_ready_dt(&clock_pin)) {
        LOG_ERR("chain control pin controller not ready");
        init_status.store(-ENODEV);
        return;
    }
    if (int const rc{gpio_pin_configure_dt(&data_pin, GPIO_OUTPUT_INACTIVE)}; rc != 0) {
        LOG_ERR("data pin not configurable (%d)", rc);
        init_status.store(rc);
        return;
    }
    if (int const rc{gpio_pin_configure_dt(&clock_pin, GPIO_OUTPUT_INACTIVE)}; rc != 0) {
        LOG_ERR("clock pin not configurable (%d)", rc);
        init_status.store(rc);
        return;
    }
    init_status.store(0);
    // Deliberately NO enumeration here: `tof enum` is a manual commissioning
    // step, and the acquisition thread (Phase 3) will own the boot-time
    // sequence once it exists.
    LOG_INF("tof chain glue ready (data settle %u ms, sensor boot %u ms; "
            "DS20001 provisional timing)",
            kDataSettleMs, kSensorBootMs);

#if defined(ENABLE_TOF_CLIFF_ULD)
    /* The cliff subsystem's ONE bootstrap, from the ONE context allowed to run it: main(), before
     * any per-feature thread starts. tof_acq reads configured_/active_ outside the chain lock on
     * exactly that basis, so init() and teardown() must never be called from anywhere else.
     *
     * After the control lines, because a subsystem whose enable lines are not configurable has
     * nothing to acquire from. A failure here is logged and left in the stage: the shell command
     * reports which step failed, and the health path is still what a consumer hears. */
    if (const int rc{tof_cliff_runtime::bootstrap(tof_cliff_runtime::config_from_devicetree())};
        rc != 0) {
        LOG_ERR("cliff runtime bootstrap failed at %s (%d)",
                tof_cliff_runtime::stage_name(tof_cliff_runtime::current_stage()), rc);
    }
#endif
}

}  // namespace lexxhard::tof_chain_controller

#endif  // ENABLE_TOF_CHAIN
