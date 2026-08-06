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

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "tof_chain_controller.hpp"
#include "tof_chain_spec.hpp"
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
        uint8_t buf[2]{};
        if (m == tof_enum::model::l7cx) {
            // ULD is_alive shape: page 0, read 0x0000/0x0001, page 2.
            int rc{vl53_wr8(addr7, tof_readdress::kL7PageReg, 0x00)};
            if (rc == 0)
                rc = vl53_rd(addr7, tof_readdress::kL7IdReg, buf, sizeof buf);
            int const restore{vl53_wr8(addr7, tof_readdress::kL7PageReg, 0x02)};
            if (rc != 0)
                return rc;
            if (restore != 0)
                return restore;  // a device left on page 0 is not a usable read
        } else {
            if (int const rc{vl53_rd(addr7, tof_readdress::kL4IdReg, buf, sizeof buf)}; rc != 0)
                return rc;
        }
        out.first = buf[0];
        out.second = buf[1];
        return 0;
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
    return r.status == tof_enum::chain_status::failed ? -EIO : 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof,
    SHELL_CMD(enum, NULL,
              "manual commissioning: enumerate the ToF chain (holds the chain lock)",
              cmd_enum),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(tof, &sub_tof, "ToF chain commands", NULL);

}  // namespace

k_mutex &chain_lock()
{
    return chain_mutex;
}

void init()
{
    if (!device_is_ready(i2c2_dev))
        LOG_ERR("i2c2 not ready");
    if (gpio_pin_configure_dt(&data_pin, GPIO_OUTPUT_INACTIVE) != 0 ||
        gpio_pin_configure_dt(&clock_pin, GPIO_OUTPUT_INACTIVE) != 0)
        LOG_ERR("chain control pins not configurable");
    // Deliberately NO enumeration here: `tof enum` is a manual commissioning
    // step, and the acquisition thread (Phase 3) will own the boot-time
    // sequence once it exists.
    LOG_INF("tof chain glue ready (data settle %u ms, sensor boot %u ms; "
            "DS20001 provisional timing)",
            kDataSettleMs, kSensorBootMs);
}

}  // namespace lexxhard::tof_chain_controller

#endif  // ENABLE_TOF_CHAIN
