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
#include "tof_l7_blob_provider.hpp"
#if defined(ENABLE_TOF_L7_ULD)
#include "tof_l7_runtime.hpp"
#endif
#if defined(ENABLE_TOF_CLIFF_ULD)
#include "tof_acquisition.hpp"
#include "tof_cliff_packer.hpp"
#include "tof_cliff_runtime.hpp"
#include "tof_commissioning.hpp"
#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 6
/* The budget probe is C, and this is the whole of its interface: one call, made after the bootstrap
 * this file performs. Declared here rather than in a header of its own because there is exactly one
 * caller and it must stay that way. */
extern "C" int tof_cliff_budget_run_after_bootstrap(void);
#endif
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

const char *bus_state_name(tof_commissioning::bus_state b)
{
    using tof_commissioning::bus_state;
    switch (b) {
    case bus_state::unknown:      return "unknown";
    case bus_state::proof_100k:   return "100k";
    case bus_state::product_400k: return "400k";
    }
    return "?";
}

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
    case stage::proof_speed_refused:   return "proof_speed_refused";
    case stage::product_speed_refused: return "product_speed_refused";
    case stage::identity_recheck_failed: return "identity_recheck_failed";
    case stage::commit_refused:     return "commit_refused";
    }
    return "?";
}

/* The production retime, and the ONE place the two speeds become register values.
 *
 * It does not take the chain lock: commissioning calls it while holding it, and the shell command
 * below takes the lock itself around its own call. A lock in here would deadlock the first and be
 * redundant in the second.
 *
 * Nor does it check whether acquisition is running. That check belongs to the callers, which are
 * in a position to know: commissioning has already quiesced and holds the chain, and the shell
 * command refuses under the lock. Repeating it here would be a third opinion about the same fact. */
int set_bus_speed_hw(tof_commissioning::bus_speed s)
{
    if (!device_is_ready(i2c2_dev))
        return -ENODEV;

    const uint32_t speed{s == tof_commissioning::bus_speed::proof_100k ? I2C_SPEED_STANDARD
                                                                       : I2C_SPEED_FAST};
    return i2c_configure(i2c2_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed));
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
    /* The transaction owns the bus speed for the whole run: 100 kHz for the walks, 400 kHz before
     * anything is published, and back to 100 kHz if it gives up. The bench `i2cspeed` command still
     * exists and still changes nothing else, but a proof no longer depends on an operator having
     * run it -- and must not, since the bus can be left at either speed by a previous failure. */
    cfg.set_bus_speed = set_bus_speed_hw;
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
    shell_print(shell, "speed: rc=%d bus=%s restore_attempted=%d restore_rc=%d", r.speed_rc,
                bus_state_name(r.final_bus), static_cast<int>(r.restore_attempted), r.restore_rc);
    if (r.failed_at == tof_commissioning::stage::identity_recheck_failed)
        shell_print(shell, "recheck: pos=%u addr=0x%02x silent=%d read_rc=%d id=%02x/%02x",
                    r.recheck.position, r.recheck.address, static_cast<int>(r.recheck.silent),
                    r.recheck.read_rc, r.recheck.seen.first, r.recheck.seen.second);
    if (!r.proven() && r.final_bus != tof_commissioning::bus_state::proof_100k)
        shell_warn(shell, "the bus is NOT back at 100 kHz. Nothing was published and no mapping was "
                          "installed, so this is recoverable: the next `tof cliff prove` sets the "
                          "speed itself rather than trusting this restore.");

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

    /* Proven, keyed, and deliberately going no further. The acquisition thread now exists -- this
     * command simply does not start it, because starting it is a separate decision from proving a
     * mapping. Saying so here keeps an operator from reading "proven" as "producing": with the
     * clamp gone, starting acquisition is now the only thing between this line and frames on the
     * bus, which makes the distinction more important than it was, not less. */
    shell_print(shell, "mapping installed under epoch %lu and descriptors keyed; the acquisition "
                       "thread was NOT started, so nothing is being measured or published yet",
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
     * and only a PROVEN mapping decides the second one. */
    shell_print(shell, "measurement frames leave this board while the mapping is PROVEN; health "
                       "frames were already flowing since boot");
    return 0;
}

int cmd_cliff_read(const struct shell *shell, size_t argc, char **argv)
{
    /* Answers "does this sensor range", nothing more. A partial chain cannot reach PROVEN by rule,
     * so no measurement frame exists to inspect on the bus -- but whether a given L4 produces a
     * plausible distance through the real ULD and the real I2C port is a separate and answerable
     * question, and this is where it gets answered. Output goes to the operator, not to CAN. */
    if (int const st{init_status.load()}; st != 0) {
        shell_error(shell, "chain glue not initialised (rc=%d)", st);
        return -ENODEV;
    }

    char *end{nullptr};
    unsigned long const pos{strtoul(argv[1], &end, 0)};

    if (end == argv[1] || *end != '\0' || pos < 1 || pos > 6) {
        shell_error(shell, "usage: tof cliff read <position 1-6> [attempts] [gap_ms]");
        return -EINVAL;
    }

    /* attempts/gap_ms default to one immediate check, which is exactly what one acquisition cycle
     * sees. Asking for a measured distance means asking for the wait explicitly. */
    unsigned long attempts{1};
    unsigned long gap_ms{0};

    if (argc >= 3) {
        attempts = strtoul(argv[2], &end, 0);
        if (end == argv[2] || *end != '\0' || attempts < 1 ||
            attempts > tof_cliff_runtime::kMaxProbeAttempts) {
            shell_error(shell, "attempts must be 1-%u", tof_cliff_runtime::kMaxProbeAttempts);
            return -EINVAL;
        }
    }
    if (argc == 4) {
        gap_ms = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || gap_ms > tof_cliff_runtime::kMaxProbeGapMs) {
            shell_error(shell, "gap_ms must be 0-%u", tof_cliff_runtime::kMaxProbeGapMs);
            return -EINVAL;
        }
    }

    tof_cliff_runtime::probe_result r{};
    int const rc{tof_cliff_runtime::probe_position(pos, r, attempts, gap_ms)};

    if (rc != 0) {
        /* Named, because each one sends the reader somewhere different: EBUSY means stop the
         * acquisition thread, ENOTSUP means that position is a grid sensor, EPERM means the
         * bootstrap never completed. */
        shell_error(shell, "probe refused (%d)%s", rc,
                    rc == -EBUSY    ? ": the acquisition thread owns the ULD" :
                    rc == -ENOTSUP  ? ": that position is not a cliff sensor" :
                    rc == -EPERM    ? ": the cliff runtime is not ready" : "");
        return rc;
    }

    /* Only the steps that actually ran are printed with a result. probe_position stops at the
     * first failure, so "open=-116 start=0" would report a zero that is the initialiser rather
     * than a success -- and a reader would conclude start worked. */
    if (r.open_rc != 0) {
        shell_print(shell, "pos%lu addr=0x%02x role_id=%u open=%d stage=%s errno=%d uld=%d", pos,
                    r.addr_7bit, r.role_id, r.open_rc, tof_cliff_stage_name(r.status.stage),
                    r.status.port_errno, r.status.uld_rc);
        shell_print(shell, "  configure/start/read NOT attempted (has this position been "
                           "enumerated? run `tof enum` first)");
        return -EIO;
    }
    if (r.start_rc != 0) {
        shell_print(shell, "pos%lu addr=0x%02x role_id=%u open=0 start=%d stage=%s errno=%d uld=%d",
                    pos, r.addr_7bit, r.role_id, r.start_rc,
                    tof_cliff_stage_name(r.status.stage), r.status.port_errno, r.status.uld_rc);
        shell_print(shell, "  read NOT attempted");
        return -EIO;
    }
    shell_print(shell, "pos%lu addr=0x%02x role_id=%u open=0 start=0 read=%d attempts_used=%u",
                pos, r.addr_7bit, r.role_id, r.read_rc, r.attempts_used);
    shell_print(shell, "  fresh=%d targets=%u entries=%u rearm_failed=%d stage=%s errno=%d",
                r.sample.fresh, r.sample.target_count, r.sample.entry_count,
                r.status.rearm_failed, tof_cliff_stage_name(r.status.stage),
                r.status.port_errno);
    for (uint8_t e{0}; e < r.sample.entry_count && e < TOF_CLIFF_MAX_TARGETS; ++e) {
        shell_print(shell, "  target[%u] range=%d mm status=%u", e,
                    r.sample.entries[e].range_mm, r.sample.entries[e].range_status);
    }
    if (!r.sample.fresh)
        shell_print(shell, "  no frame within %lu check(s): retry with more attempts and a gap, "
                           "e.g. `tof cliff read %lu 20 20`", attempts, pos);

    if (r.configure_rc != 0)
        shell_print(shell, "  configure failed (%d): the sequence stopped, nothing was read",
                    r.configure_rc);

    /* Stated so that a good reading is not mistaken for a validated data path. */
    shell_print(shell, "diagnostic only: nothing was published, the mapping state is unchanged");
    shell_print(shell, "ranged at the descriptor's own profile, same as acquisition uses");
    return 0;
}

#if defined(ENABLE_TOF_CLIFF_BENCH_PACK)

/* This command may only exist in a build the contract already forbids releasing. If that flag ever
 * flips, this assertion stops the build and forces someone to decide deliberately whether a
 * command that prints a production-shaped payload belongs in a releasable image. */
static_assert(tof_cliff_contract::kReleaseForbidden,
              "tof cliff pack is bench-only: it must not exist in a releasable contract build");

/* Named rather than numeric, because the whole point of these three enumerations is that the
 * follow-up differs per value; a transcript full of small integers sends the reader back to the
 * header to find out which finding it recorded. */
const char *result_name(tof_cliff_packer::result r)
{
    switch (r) {
    case tof_cliff_packer::result::frame_ready: return "frame_ready";
    case tof_cliff_packer::result::no_frame:    return "no_frame";
    case tof_cliff_packer::result::unencodable: return "unencodable";
    }
    return "?";
}

const char *status_class_name(tof_cliff_contract::status_class c)
{
    switch (c) {
    case tof_cliff_contract::status_class::valid_range:  return "valid_range";
    case tof_cliff_contract::status_class::no_target:    return "no_target";
    case tof_cliff_contract::status_class::sensor_fault: return "sensor_fault";
    case tof_cliff_contract::status_class::no_sample:    return "no_sample";
    }
    return "?";
}

const char *reason_name(tof_cliff_packer::reason w)
{
    using tof_cliff_packer::reason;
    switch (w) {
    case reason::none:                                return "none";
    case reason::zero_targets_with_unexpected_status: return "zero_targets_with_unexpected_status";
    case reason::zero_targets_entry_count_not_one:    return "zero_targets_entry_count_not_one";
    case reason::entry_count_mismatch:                return "entry_count_mismatch";
    case reason::none_status_among_targets:           return "none_status_among_targets";
    case reason::valid_range_negative:                return "valid_range_negative";
    case reason::valid_range_is_sentinel:             return "valid_range_is_sentinel";
    case reason::status_undefined:                    return "status_undefined";
    case reason::target_count_malformed:              return "target_count_malformed";
    case reason::no_entries:                          return "no_entries";
    case reason::source_id_out_of_range:              return "source_id_out_of_range";
    case reason::reduction_inconsistent:              return "reduction_inconsistent";
    }
    return "?";
}

/* BENCH ONLY. Reconfigures i2c2's bitrate at runtime.
 *
 * WHY THIS EXISTS. Enumeration and sensor bring-up have turned out to want different bitrates on
 * this machine: at 400 kHz a commissioning walk has never once reached COMPLETE, while at 100 kHz
 * every walk completes but VL53L4CX bring-up fails with -ETIMEDOUT in boot/data_init. The
 * devicetree bitrate is only the boot default -- i2c_stm32_runtime_configure() is the driver's
 * .configure entry point, and the driver's own init reaches the initial speed through it -- so one
 * image can do the walk at one speed and the reads at another without rebooting, which is the only
 * way to join the two halves that have each been verified separately.
 *
 * WHAT IT DOES NOT DO. It does not touch the mapping, the publisher or the acquisition
 * thread, and it refuses while acquisition runs: that thread owns the chain, and changing the bus
 * timing underneath a cycle in flight would corrupt a read rather than fail it. It takes the chain
 * lock so a commissioning walk cannot be halfway through either.
 *
 * WHAT THE OPERATOR STILL OWES. A bitrate change is not a proof. If bring-up succeeds at the new
 * speed, that says the sensors answer there -- it does not re-establish that the mapping proved at
 * the other speed still describes this chain. Probe the identities before trusting it, and if
 * anything fails, revoke rather than proceed: a mapping that cannot be read at the acquisition
 * speed must not stay PROVEN. */
int cmd_cliff_i2cspeed(const struct shell *shell, size_t, char **argv)
{
    if (int const st{init_status.load()}; st != 0) {
        shell_error(shell, "chain glue not initialised (rc=%d)", st);
        return -ENODEV;
    }

    char *end{nullptr};
    unsigned long const khz{strtoul(argv[1], &end, 0)};
    uint32_t speed{0};

    if (end == argv[1] || *end != '\0') {
        shell_error(shell, "usage: tof cliff i2cspeed <100|400>");
        return -EINVAL;
    }
    if (khz == 100)
        speed = I2C_SPEED_STANDARD;
    else if (khz == 400)
        speed = I2C_SPEED_FAST;
    else {
        /* Only the two the contract and the diagnostic overlay actually use. A free-form kHz field
         * would invite a value nobody has characterised on this harness. */
        shell_error(shell, "only 100 or 400 kHz: %lu is not a speed this chain is characterised at",
                    khz);
        return -EINVAL;
    }

    if (!device_is_ready(i2c2_dev)) {
        shell_error(shell, "i2c2 not ready");
        return -ENODEV;
    }

    /* The acquisition check lives INSIDE the lock, and that placement is the whole point. Checked
     * before the lock, it is a time-of-check-to-time-of-use hole: the thread can start after the
     * check, this command then waits out the cycle in flight, acquires the lock the scheduler
     * released at the cycle boundary, and retimes the bus while acquisition is still running --
     * exactly what the check exists to prevent. Under the lock the answer cannot change before it
     * is used, and it is also the context the flag is written in. */
    k_mutex_lock(&chain_mutex, K_FOREVER);
    if (tof_acq::thread_running()) {
        k_mutex_unlock(&chain_mutex);
        shell_error(shell, "acquisition thread is running: it owns the chain, and retiming the bus "
                           "under a cycle in flight would corrupt a read rather than fail it");
        return -EBUSY;
    }
    int const rc{i2c_configure(i2c2_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed))};
    k_mutex_unlock(&chain_mutex);

    if (rc != 0) {
        shell_error(shell, "i2c_configure failed (%d): the bus is left at whatever the driver did "
                           "with it, so re-run this before trusting any read",
                    rc);
        return rc;
    }
    shell_print(shell, "i2c2 reconfigured to %lu kHz. The mapping and the acquisition "
                       "thread were NOT touched; a bitrate change is not a proof, so probe the "
                       "identities before trusting a read, and revoke rather than proceed if "
                       "anything fails at this speed.",
                khz);
    return 0;
}

/* BENCH ONLY. Encodes one real sample into the contract's measurement payload and prints it.
 *
 * WHY THIS IS NOT A GATE BYPASS. It transmits nothing. The publisher and the authorisation
 * callback are untouched, so no unauthorised measurement can reach the bus from
 * this board -- the end of this path is an operator's terminal. Verifying the CAN hop and the
 * SCBDriver decoder is a separate, deliberate act: the operator injects these bytes with cansend.
 * Giving the firmware itself the ability to emit an unproven measurement would create that code
 * path permanently, and "the downstream is isolated" is a runtime property; "the firmware cannot
 * emit it" is a structural one.
 *
 * WHY source_id IS AN ARGUMENT. encode_measurement() refuses source_id >= 4, and an unproven
 * mapping keys every descriptor to kRoleUnassigned (255) -- correctly, because without a proof
 * there IS no legitimate source_id. So the operator has to declare one, which puts the fact that
 * nothing was proven at the call site instead of hiding it. The output says so on every line that
 * carries bytes, because those bytes are byte-identical to a production frame's payload and must
 * never be quoted as evidence that this board published one.
 *
 * attempts/gap_ms default to one immediate check, the same as `read`: the default reproduces what
 * one acquisition cycle sees, which is usually "no sample yet". Asking for a frame means asking
 * for the wait. */
int cmd_cliff_pack(const struct shell *shell, size_t argc, char **argv)
{
    namespace pk = tof_cliff_packer;

    if (int const st{init_status.load()}; st != 0) {
        shell_error(shell, "chain glue not initialised (rc=%d)", st);
        return -ENODEV;
    }

    char *end{nullptr};
    auto const num{[&](const char *s, unsigned long limit, unsigned long &out) {
        out = strtoul(s, &end, 0);
        return end != s && *end == '\0' && out <= limit;
    }};

    unsigned long pos{0}, source_id{0}, epoch{0}, cyc{0}, attempts{1}, gap_ms{0};

    if (!num(argv[1], 6, pos) || pos < 1) {
        shell_error(shell, "usage: tof cliff pack <pos 1-6> <source_id 0-3> "
                           "[epoch] [cycle_seq] [attempts] [gap_ms]");
        return -EINVAL;
    }
    /* Range-checked here as well as in the packer, so the refusal names the argument rather than
     * arriving as an opaque encode failure. */
    if (!num(argv[2], tof_cliff_contract::kSourceCount - 1, source_id)) {
        shell_error(shell, "source_id must be 0-%u", tof_cliff_contract::kSourceCount - 1);
        return -EINVAL;
    }
    if (argc >= 4 && !num(argv[3], 255, epoch)) {
        shell_error(shell, "epoch must be 0-255");
        return -EINVAL;
    }
    if (argc >= 5 && !num(argv[4], 255, cyc)) {
        shell_error(shell, "cycle_seq must be 0-255");
        return -EINVAL;
    }
    if (argc >= 6 && (!num(argv[5], tof_cliff_runtime::kMaxProbeAttempts, attempts) ||
                      attempts < 1)) {
        shell_error(shell, "attempts must be 1-%u", tof_cliff_runtime::kMaxProbeAttempts);
        return -EINVAL;
    }
    if (argc == 7 && !num(argv[6], tof_cliff_runtime::kMaxProbeGapMs, gap_ms)) {
        shell_error(shell, "gap_ms must be 0-%u", tof_cliff_runtime::kMaxProbeGapMs);
        return -EINVAL;
    }

    tof_cliff_runtime::probe_result r{};
    if (int const rc{tof_cliff_runtime::probe_position(pos, r, attempts, gap_ms)}; rc != 0) {
        shell_error(shell, "probe refused (%d)%s", rc,
                    rc == -EBUSY   ? ": the acquisition thread owns the ULD" :
                    rc == -ENOTSUP ? ": that position is not a cliff sensor" :
                    rc == -EPERM   ? ": the cliff runtime is not ready" : "");
        return rc;
    }
    /* Reported one stage at a time. Printing "open=-116 start=0" reads as though start succeeded,
     * but probe_position never calls start when open fails: that zero is the initialiser, not a
     * result. Naming the un-attempted steps is the same distinction as no_frame vs unencodable. */
    if (r.open_rc != 0) {
        shell_error(shell, "pos%lu open=%d stage=%s errno=%d uld=%d -- configure/start/read NOT "
                           "attempted (has this position been enumerated? run `tof enum` first)",
                    pos, r.open_rc, tof_cliff_stage_name(r.status.stage), r.status.port_errno,
                    r.status.uld_rc);
        return -EIO;
    }
    if (r.start_rc != 0) {
        shell_error(shell, "pos%lu start=%d stage=%s errno=%d uld=%d -- read NOT attempted", pos,
                    r.start_rc, tof_cliff_stage_name(r.status.stage), r.status.port_errno,
                    r.status.uld_rc);
        return -EIO;
    }
    /* A failed read leaves the sample at its default, which reduces to no_frame -- so without this
     * check a real transport or protocol error would be reported as "nothing was produced this
     * cycle, which is correct", and the command would exit successfully. The two are opposite
     * findings and must not share an exit path. */
    if (r.read_rc != 0) {
        shell_error(shell, "pos%lu read failed (%d) stage=%s errno=%d uld=%d: refusing to reduce a "
                           "sample that was never read", pos, r.read_rc,
                    tof_cliff_stage_name(r.status.stage), r.status.port_errno, r.status.uld_rc);
        return -EIO;
    }

    shell_print(shell, "pack: pos%lu addr=0x%02x attempts_used=%u", pos, r.addr_7bit,
                r.attempts_used);
    shell_print(shell, "sample: fresh=%d targets=%u entries=%u", r.sample.fresh,
                r.sample.target_count, r.sample.entry_count);

    pk::reduction const red{pk::reduce(r.sample)};
    shell_print(shell, "reduction: outcome=%s class=%s range_mm=%u raw_status=%u observed=%u",
                result_name(red.outcome), status_class_name(red.cls), red.range_mm, red.raw_status,
                red.observed_status);

    /* no_frame and unencodable are opposite findings and the packer's own header says so:
     * no_frame is the contract's correct answer for a cycle that produced nothing, while
     * unencodable means the input has no representation on the wire -- a defect in the read layer
     * or the ULD, named by `why`. Collapsing them, as this command first did, reports a protocol
     * anomaly as normal operation. */
    if (red.outcome == pk::result::no_frame) {
        shell_print(shell, "no frame: no new sample this read, and transmitting nothing is the "
                           "contract's correct outcome -- ask for the wait (attempts/gap_ms) to "
                           "observe a frame");
        return 0;
    }
    if (red.outcome != pk::result::frame_ready) {
        shell_error(shell, "unencodable: this sample has no representation on the wire "
                           "(why=%s, observed_status=%u) -- an anomaly in the read layer or the "
                           "ULD, not a quiet cycle",
                    reason_name(red.why), red.observed_status);
        return -EPROTO;
    }

    uint8_t frame[8]{};
    pk::reason why{pk::reason::none};
    if (!pk::encode_measurement(red, static_cast<uint8_t>(source_id),
                                static_cast<uint8_t>(epoch), static_cast<uint8_t>(cyc), frame,
                                &why)) {
        shell_error(shell, "packer refused the encode (why=%s)", reason_name(why));
        return -EIO;
    }

    /* Printed so the transcript is self-contained evidence: which position and address produced
     * it, which identity fields the operator declared, which frame kind and length it would be,
     * and which contract text and generated artefact set define that layout. Without the last two
     * a captured payload cannot be replayed against the right decoder a month later. */
    shell_print(shell, "BENCH-ONLY payload for can_id=0x%03x dlc=%u, source_id=%lu DECLARED BY "
                       "OPERATOR (no proof exists):",
                tof_cliff_contract::kMeasId, tof_cliff_contract::kDlc, source_id);
    shell_print(shell, "  %02x %02x %02x %02x %02x %02x %02x %02x", frame[0], frame[1], frame[2],
                frame[3], frame[4], frame[5], frame[6], frame[7]);
    shell_print(shell, "fields: source_id=%lu epoch=%lu cycle_seq=%lu", source_id, epoch, cyc);
    shell_print(shell, "contract: sha=%s", tof_cliff_contract::kContractSha256);
    shell_print(shell, "artefact: %s", tof_cliff_contract::kArtefactSetId);
    shell_print(shell, "NOT TRANSMITTED: no CAN frame was sent, the publisher was not used, the "
                       "mapping state is unchanged. These bytes "
                       "are not evidence that this board published a measurement.");
    return 0;
}

/* BENCH ONLY: read several frames from ONE sensor session.
 *
 * `tof cliff read` stops at the first fresh frame and then stops the sensor, so every L4 sample
 * this project has recorded -- on every machine, across every run -- is the first frame after a
 * restart, and every one of them reported no target. The existing evidence has therefore only
 * ever observed frame one, and a first-frame effect cannot be ruled out. This command does not
 * decide whether the sensors can range; it removes the reason we cannot tell.
 *
 * stream_count is printed per frame because on THIS image `fresh` means only that the device
 * reported data ready -- the stream-count replay check is later work and is not in this build --
 * so the counter is the only thing that shows whether successive frames are successive ranging
 * sequences rather than one buffer read repeatedly.
 *
 * Frames are printed as they arrive. Nothing is buffered: the shell thread has a few hundred
 * bytes of stack headroom on this board and assertions are not compiled in, so an array of
 * frames here would corrupt memory rather than fail. */
void stream_frame_to_shell(void *ctx, unsigned index, const struct tof_cliff_sample &s)
{
    const struct shell *shell{static_cast<const struct shell *>(ctx)};
    shell_print(shell, "  frame[%u] stream_count=%u targets=%u entries=%u", index, s.stream_count,
                s.target_count, s.entry_count);
    for (unsigned e{0}; e < s.entry_count && e < TOF_CLIFF_MAX_TARGETS; ++e)
        shell_print(shell, "    entry[%u] range=%d mm status=%u", e, s.entries[e].range_mm,
                    s.entries[e].range_status);
}

int cmd_cliff_stream(const struct shell *shell, size_t argc, char **argv)
{
    unsigned long pos{0}, frames{10}, gap_ms{40}, max_attempts{tof_cliff_runtime::kMaxProbeAttempts};

    const auto num = [](const char *s, unsigned long limit, unsigned long &out) {
        char *end{nullptr};
        out = strtoul(s, &end, 0);
        return end != s && *end == '\0' && out <= limit;
    };

    if (!num(argv[1], 6, pos) || pos == 0) {
        shell_error(shell, "usage: tof cliff stream <position 1-6> [frames] [gap_ms] [max_attempts]");
        return -EINVAL;
    }
    if (argc >= 3 && (!num(argv[2], tof_cliff_runtime::kMaxProbeAttempts, frames) || frames == 0)) {
        shell_error(shell, "frames must be 1-%u", tof_cliff_runtime::kMaxProbeAttempts);
        return -EINVAL;
    }
    if (argc >= 4 && !num(argv[3], tof_cliff_runtime::kMaxProbeGapMs, gap_ms)) {
        shell_error(shell, "gap_ms must be 0-%u", tof_cliff_runtime::kMaxProbeGapMs);
        return -EINVAL;
    }
    if (argc >= 5 &&
        (!num(argv[4], tof_cliff_runtime::kMaxProbeAttempts, max_attempts) || max_attempts == 0)) {
        shell_error(shell, "max_attempts must be 1-%u", tof_cliff_runtime::kMaxProbeAttempts);
        return -EINVAL;
    }
    if (max_attempts < frames) {
        shell_error(shell, "max_attempts (%lu) must be at least frames (%lu)", max_attempts, frames);
        return -EINVAL;
    }

    tof_cliff_runtime::stream_result r{};
    shell_print(shell, "pos%lu: one open/configure/start, then %lu frame(s) without restarting",
                pos, frames);
    const int rc{tof_cliff_runtime::stream_position(pos, r, frames, gap_ms, max_attempts,
                                                    stream_frame_to_shell,
                                                    const_cast<struct shell *>(shell))};
    if (rc != 0) {
        shell_error(shell, "stream refused (%d)%s", rc,
                    rc == -EBUSY ? " -- the acquisition thread owns the chain" : "");
        return rc;
    }

    shell_print(shell, "pos%lu addr=0x%02x role_id=%u open=%d start=%d last_read=%d", pos,
                r.addr_7bit, r.role_id, r.open_rc, r.start_rc, r.last_read_rc);
    shell_print(shell, "frames=%u/%lu attempts_used=%u gap_ms=%lu", r.frames_collected, frames,
                r.attempts_used, gap_ms);
    if (r.open_rc != 0 || r.start_rc != 0)
        shell_print(shell, "  session did not start; nothing was read");
    if (r.configure_rc != 0)
        shell_print(shell, "  configure failed (%d): the session did not start", r.configure_rc);
    shell_print(shell, "diagnostic only: nothing was transmitted, the mapping is unchanged and the "
                       "publisher was not involved.");
    shell_print(shell, "ranged at the descriptor's own profile (devicetree), not a ULD default.");
    return 0;
}

#endif // ENABLE_TOF_CLIFF_BENCH_PACK



/* Read-only. It starts nothing, stops nothing, touches no device, takes no lock and reads no
 * mapping; it prints counters the acquisition thread maintains and returns. It is NOT gated and
 * has nothing to gate: there is no state it could put the board into.
 *
 * It exists because success is silent everywhere else. A working cycle logs nothing by design, so
 * "the acquisition thread is alive" and "the acquisition thread is reading four sensors every
 * cycle" were indistinguishable from outside the board -- and while the PROVEN clamp was in place
 * the health frame's cycle fields were zeroed too, which left nothing at all to read. Inferring the second from the first, or from the heartbeat, is exactly
 * the kind of guess this project has had to retract before.
 *
 * AT MOST TWO FIELDS PER LINE, and that is a stack budget rather than a formatting preference. The
 * shell thread's high-water mark is already 1856/1984 -- about a hundred bytes spare -- and
 * CONFIG_ASSERT is off, so anything that overruns it corrupts memory silently and presents as a
 * sensor fault. An earlier version of this command claimed "one field at a time" in its comment
 * and then passed seven arguments to a single call, which is the same defect with a reassuring
 * label on it. Nothing is buffered here: no struct, no array, and the only local is the one status
 * word, which has to be held because reading it twice would stop being a snapshot.
 *
 * What the short argument lists buy is a smaller frame, NOT the absence of stack arguments:
 * shell_print expands to a variadic call that still places something on the stack whatever the
 * visible argument count. The number that matters is the measured one -- this function's prologue
 * is a single stmdb of eight registers and no sub sp, so 32 bytes, against 64 for the seven
 * argument version it replaced. Re-measure it rather than reasoning about it if this changes.
 *
 * THE ROW LABEL IS THE ACQUISITION DESCRIPTOR INDEX, not a contract source_id. They do not agree
 * and must not be printed as if they did: on this chain acq 0 and 1 are the grid sensors and the
 * four cliff L4s are acq 2..5, whose contract source_ids are 0..3. A row labelled "src2" could be
 * read as either, and the two readings name different corners of the robot. */
int cmd_cliff_stats(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "cycles_completed %u", tof_acq::cycles_completed());
    shell_print(shell, "foreign_lifecycle_calls %u", tof_acq::foreign_lifecycle_calls());

    for (int i{0}; i < 6; ++i) {
        const uint32_t id{tof_acq::source_identity(i)};

        if (tof_acq::identity_is_cliff(id))
            shell_print(shell, "acq%d cliff role %d", i, tof_acq::identity_role(id));
        else
            shell_print(shell, "acq%d not-cliff", i);

        shell_print(shell, "acq%d reads %u", i, tof_acq::source_reads(i));
        shell_print(shell, "acq%d samples %u", i, tof_acq::source_samples(i));
        shell_print(shell, "acq%d read_errors %u", i, tof_acq::source_read_errors(i));
        shell_print(shell, "acq%d rearm_failures %u", i, tof_acq::source_rearm_failures(i));

        /* Fetched once and decoded twice, because two fetches would put this cycle's stage beside
         * the previous cycle's errno -- the exact pairing the packed word exists to prevent. */
        const uint32_t st{tof_acq::source_last_status(i)};
        shell_print(shell, "acq%d last_stage %d", i, tof_acq::last_status_stage(st));
        shell_print(shell, "acq%d last_errno %d", i, tof_acq::last_status_errno(st));
    }

    /* Said rather than left to be assumed: a source that never started is indistinguishable in
     * these numbers from one that started and has never been read, and both read zero. The role id
     * is the CONTRACT's source_id for that corner; the acq index is not. */
    shell_print(shell, "cumulative since init; reads==0 means never started OR thread not running");
    shell_print(shell, "acq index is NOT the contract source_id -- read the role line");
    return 0;
}

/* Where a cycle's time goes, and how much of what it read was new.
 *
 * Its own command rather than more lines in `stats`, because it answers a different question and
 * because that function's frame is already sized against a shell stack with about a hundred bytes
 * spare. The same rule applies here and for the same reason: AT MOST TWO FIELDS PER LINE, no
 * buffers, no locals but the ones that have to be held.
 *
 * Microseconds throughout. `max` is a high-water mark since init(), and it is the number the safety
 * argument needs: a mean rate says nothing about the longest a corner went without a new
 * measurement.
 *
 * TWO THINGS THESE NUMBERS ARE NOT. publish_us covers the snapshot word, the packer and the CAN
 * sends together, so a large value does not by itself accuse the bus -- separating the sends needs
 * a clock inside the publisher and is worth doing only once this number says it is worth doing.
 * And the parts do not sum to the gap: what is left over is scheduling latency, the thread becoming
 * runnable after its wait and being preempted by anything above it.
 *
 * THE ROW LABEL IS THE ACQUISITION DESCRIPTOR INDEX, not a contract source_id -- same warning as
 * `stats`, same reason. */
int cmd_cliff_timing(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "cycles_completed %u", tof_acq::cycles_completed());
    shell_print(shell, "lock_wait_us last %u", tof_acq::cycle_lock_wait_us_last());
    shell_print(shell, "lock_wait_us max %u", tof_acq::cycle_lock_wait_us_max());
    shell_print(shell, "work_us last %u", tof_acq::cycle_work_us_last());
    shell_print(shell, "work_us max %u", tof_acq::cycle_work_us_max());
    shell_print(shell, "publish_us last %u", tof_acq::cycle_publish_us_last());
    shell_print(shell, "publish_us max %u", tof_acq::cycle_publish_us_max());
    shell_print(shell, "total_us last %u", tof_acq::cycle_total_us_last());
    shell_print(shell, "total_us max %u", tof_acq::cycle_total_us_max());
    shell_print(shell, "wait_us last %u", tof_acq::cycle_wait_us_last());
    shell_print(shell, "wait_us max %u", tof_acq::cycle_wait_us_max());
    shell_print(shell, "cycle_gap_us max %u", tof_acq::cycle_gap_us_max());
    shell_print(shell, "cycle_overruns %u", tof_acq::cycle_overruns());

    for (int i{0}; i < 6; ++i) {
        shell_print(shell, "acq%d read_us_last %u", i, tof_acq::source_read_us_last(i));
        shell_print(shell, "acq%d read_us_max %u", i, tof_acq::source_read_us_max(i));
        shell_print(shell, "acq%d new_measurements %u", i, tof_acq::source_new_measurements(i));
        shell_print(shell, "acq%d repeat_measurements %u", i,
                    tof_acq::source_repeat_measurements(i));
    }

    /* Said rather than left to be assumed -- and said WITHOUT an arithmetic relation, because
     * there is not one to state. cycle_gap_us is a high-water mark while the parts carry `last`
     * values from whichever cycle wrote them, and a gap spans the wait at the END of one cycle and
     * the lock wait at the START of the next, so the parts of any single cycle do not add up to
     * it. Printing an inequality invited exactly the subtraction that cannot be done. */
    shell_print(shell, "each figure stands alone; they are NOT terms of one sum");
    shell_print(shell, "gap is a max over cycles; the others are that cycle's own last value");
    shell_print(shell, "publish_us is snapshot + pack + CAN send, NOT the bus time alone");
    shell_print(shell, "cycle_overruns moving means the period is too short for the work");
    shell_print(shell, "repeat_measurements means fresh was set but StreamCount had not moved");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof_cliff,
#if defined(ENABLE_TOF_CLIFF_BENCH_PACK)
    SHELL_CMD_ARG(i2cspeed, NULL,
                  "BENCH ONLY: <100|400> -- retime i2c2 at runtime (walk and bring-up want "
                  "different speeds on this harness); touches no mapping and is not a proof",
                  cmd_cliff_i2cspeed, 2, 0),
    SHELL_CMD_ARG(stream, NULL,
                  "BENCH ONLY: <pos> [frames] [gap_ms] [max_attempts] -- read several frames from "
                  "ONE session; `read` only ever shows the first frame after a restart",
                  cmd_cliff_stream, 2, 3),
    SHELL_CMD_ARG(pack, NULL,
                  "BENCH ONLY: <pos> <source_id> [epoch] [cycle_seq] [attempts] [gap_ms] -- "
                  "encode one real sample and PRINT it; transmits nothing",
                  cmd_cliff_pack, 3, 4),
#endif
    SHELL_CMD(timing, NULL,
              "BENCH: where a cycle's time goes, and how much of what it read was new",
              cmd_cliff_timing),
    SHELL_CMD(stats, NULL,
              "diagnostic: cumulative acquisition counters (read-only; touches no device)",
              cmd_cliff_stats),
    SHELL_CMD_ARG(read, NULL,
                  "diagnostic: <pos> [attempts] [gap_ms] -- does this sensor range? "
                  "(default 1 check = what one acquisition cycle sees)",
                  cmd_cliff_read, 2, 2),
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

int cmd_l7_blob(const struct shell *shell, size_t, char **)
{
#if defined(ENABLE_TOF_L7_ULD)
    const tof_l7_runtime::snapshot runtime{tof_l7_runtime::current()};

    shell_print(shell, "runtime: %s", tof_l7_runtime::stage_name(runtime.current_stage));
    if (runtime.current_stage == tof_l7_runtime::stage::not_started ||
        runtime.current_stage == tof_l7_runtime::stage::verifying) {
        shell_print(shell, "verification has not completed");
        return -EAGAIN;
    }
    const tof_l7_blob::report &rep{runtime.verification};
#else
    /* Reports what is stored, and deliberately does NOT verify it against an expectation: this
     * firmware has no VL53L7CX ULD yet, so it has nothing to compare against and inventing one here
     * would be a claim rather than a check. What it can answer is the question bring-up actually
     * asks -- "is a record there, and which one" -- and it answers it from the same reader the
     * verification uses. */
    const tof_l7_blob::report rep{tof_l7_blob::stored_header()};
#endif

    shell_print(shell, "storage partition: %zu bytes", rep.region_size);
    shell_print(shell, "record: %s", tof_l7_blob::status_name(rep.st));
    if (!rep.stored.parsed) {
        shell_print(shell, "no usable header; nothing can be said about the payload");
        return rep.st == tof_l7_blob::status::ok ? 0 : -ENOENT;
    }

    shell_print(shell, "format %u, payload %u bytes", rep.stored.format_version,
                rep.stored.payload_len);
    shell_fprintf(shell, SHELL_NORMAL, "payload sha256: ");
    for (size_t i{0}; i < tof_l7_blob::kDigestSize; ++i)
        shell_fprintf(shell, SHELL_NORMAL, "%02x", rep.stored.payload_digest[i]);
    shell_fprintf(shell, SHELL_NORMAL, "\n");
#if defined(ENABLE_TOF_L7_ULD)
    if (runtime.firmware_available) {
        shell_print(shell, "verified against this signed image; %zu payload bytes authorised",
                    runtime.firmware_size);
        return 0;
    }
    shell_print(shell, "payload refused; L7 ranging remains unavailable");
    return -EIO;
#else
    /* The integrity of the payload is NOT asserted by this command: it read the header only. Saying
     * so matters -- an operator who reads "ok" here must not conclude the blob is intact. */
    shell_print(shell, "header only: the payload was not hashed, so this says nothing about whether "
                       "it is intact or which ULD it belongs to");
    return 0;
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof_l7,
    SHELL_CMD(blob, NULL, "report the device-firmware record stored in the storage partition",
              cmd_l7_blob),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof,
    SHELL_CMD(enum, NULL,
              "manual commissioning: enumerate the ToF chain (holds the chain lock)",
              cmd_enum),
#if defined(ENABLE_TOF_CLIFF_ULD)
    SHELL_CMD(cliff, &sub_tof_cliff, "cliff mapping commissioning", NULL),
#endif
    SHELL_CMD(l7, &sub_tof_l7, "grid sensor provisioning", NULL),
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

#if defined(ENABLE_TOF_L7_ULD)
    /* Integrity failure disables only L7. Cliff health must still come up: losing the hanging-object
     * feature is already fail-open for that hazard, and suppressing the independent cliff channel
     * would make the failure larger while hiding the diagnosis. */
    if (const int rc{tof_l7_runtime::bootstrap()}; rc != 0) {
        const auto state{tof_l7_runtime::current()};
        LOG_ERR("L7 runtime bootstrap failed at %s (%s, %d); L7 remains unavailable",
                tof_l7_runtime::stage_name(state.current_stage),
                tof_l7_blob::status_name(state.verification.st), rc);
    }
#endif

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
#if defined(TOF_CLIFF_BUDGET) && TOF_CLIFF_BUDGET >= 6
    /* The budget probe's walk, from HERE rather than from its own SYS_INIT.
     *
     * It used to run at APPLICATION init level, before main() -- so it brought sensors up against
     * pins this function had not configured yet, and it bootstrapped the subsystem a second time,
     * whose -EALREADY main() then logged as a bootstrap failure. Measuring an image is not a reason
     * to wire it differently from the product: the probe now runs after the same bootstrap
     * production uses, in the same order, and its only remaining job is to drive one cycle so the
     * path cannot be collected. */
    if (const int rc{tof_cliff_budget_run_after_bootstrap()}; rc != 0)
        LOG_ERR("cliff budget walk failed (%d)", rc);
#endif
#endif
}

}  // namespace lexxhard::tof_chain_controller

#endif  // ENABLE_TOF_CHAIN
