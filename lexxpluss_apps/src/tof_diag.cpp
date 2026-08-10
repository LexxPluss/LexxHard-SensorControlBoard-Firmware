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

// ToF differential-I2C bring-up diagnostics (AMRSW-2322, design Phase D).
//
// Everything here is diagnostic-only and compiled exclusively under
// TOF_I2C_DIAG (Makefile target firmware_tof_i2c_diag). The build also
// disables the tug encoder polling thread -- the only other i2c2 user --
// so shell commands own the bus and the logic-analyser traces stay clean.
//
// The commands exist to answer, in this order:
//   - is the i2c2 wiring usable at all (probe/scan against the on-board
//     ADS7138 at 0x17: PF0/PF1 are suspected swapped vs the I2C2 AF mapping)
//   - if the hardware controller fails, does a bit-banged master in the
//     board-net orientation (PF0=SCL / PF1=SDA, `bitbang probe ... swap`)
//     get an ACK -- the decisive crossed-pin evidence
//   - once the bus works: enable chain position 1 via the LPn shift
//     register, probe 0x29, read the VL53 model ID, re-address, advance.

#ifdef TOF_I2C_DIAG

#include <cerrno>
#include <cstdlib>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "tof_diag.hpp"
#include "tof_diag_bitbang.hpp"
#include "tof_diag_readdress.hpp"

// Defined in tof_diag_pinctrl.c: hands PF0/PF1 back to the I2C2 alternate
// function. Lives in a C file because PINCTRL_DT_DEFINE does not compile
// as C++ (designated-initializer ordering).
extern "C" int tof_diag_pinctrl_restore(void);

// Also defined in tof_diag_pinctrl.c: raises SparePinGPIO1..4 to
// very-high-speed and reads that setting back from OSPEEDR. Must be
// re-applied after every gpio_pin_configure_dt() on these pins -- see the
// comment on tof_diag_spare_gpio_speed_apply() for why.
extern "C" int tof_diag_spare_gpio_speed_apply(void);
extern "C" const char *tof_diag_spare_gpio_speed_label(int index);

namespace lexxhard::tof_diag {

LOG_MODULE_REGISTER(tof_diag);

namespace {

const struct device *const i2c2_dev{DEVICE_DT_GET(DT_NODELABEL(i2c2))};
const struct device *const gpiof_dev{DEVICE_DT_GET(DT_NODELABEL(gpiof))};

// SparePinGPIO1..4 on J29 (PI15 / PJ1 / PJ2 / PJ3). SparePinGPIO5 is
// comm_mode and deliberately absent. Which physical J29 wire is LPn and
// which is LPn_CLK is a bench question -- hence `lpn use <n> <m>`.
const struct gpio_dt_spec spare_pins[4]{
    GPIO_DT_SPEC_GET(DT_NODELABEL(spare_gpio_1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(spare_gpio_2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(spare_gpio_3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(spare_gpio_4), gpios),
};

int lpn_index{-1}, clk_index{-1}; // 0-based into spare_pins, -1 = unassigned

bool parse_u32(const char *arg, uint32_t &out)
{
    char *end{nullptr};
    long const value{strtol(arg, &end, 0)};
    if (end == arg || *end != '\0' || value < 0)
        return false;
    out = static_cast<uint32_t>(value);
    return true;
}

bool parse_addr7(const char *arg, uint8_t &addr)
{
    uint32_t value{0};
    if (!parse_u32(arg, value) || value < 0x03 || value > 0x77)
        return false;
    addr = static_cast<uint8_t>(value);
    return true;
}

// On this driver (i2c_ll_stm32_v2.c) a NACK, a bus error and the
// controller's transfer timeout all return -EIO, so the label makes no
// claim errno cannot back. Discrimination comes from the elapsed time
// printed next to it, the bitbang path and the logic analyser.
const char *errno_label(int err)
{
    switch (err) {
    case 0:    return "OK";
    case -EIO: return "-EIO (transfer failed: NACK, bus error or controller "
                      "timeout are indistinguishable on this driver)";
    default:   return "error (see errno)";
    }
}

// Runs one i2c operation and reports its duration: the only signal the
// hardware path has for telling a fast NACK from a driver-timeout failure.
template <typename F>
int timed_i2c(F &&operation, uint32_t &elapsed_us)
{
    uint32_t const start{k_cycle_get_32()};
    int const ret{operation()};
    elapsed_us = static_cast<uint32_t>(k_cyc_to_us_floor64(k_cycle_get_32() - start));
    return ret;
}

// Zero-length write probe, same shape as Zephyr's own `i2c scan`.
// `use_read` swaps it for a one-byte read for devices that dislike it.
int hw_probe(uint8_t addr7, bool use_read)
{
    if (use_read) {
        uint8_t byte{0};
        return i2c_read(i2c2_dev, &byte, 1, addr7);
    }
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

void print_stats(const struct shell *shell, const stress_stats &stats)
{
    shell_print(shell, "attempts:%u ok:%u failed_fast(<%uus):%u failed_slow:%u "
                       "data_mismatch:%u worst:%uus last_err:%d -> %s",
                stats.attempts, stats.ok, stats.slow_threshold_us, stats.failed_fast,
                stats.failed_slow, stats.data_mismatch, stats.worst_elapsed_us,
                stats.last_error, stats.all_ok() ? "CLEAN" : "ERRORS");
    if (stats.failed_fast + stats.failed_slow > 0)
        shell_print(shell, "note: errno cannot separate NACK / bus error / timeout on this "
                           "driver; fast failures are NACK-like, slow ones timeout-like");
}

// Owns PF0/PF1 as open-drain GPIOs for the duration of one shell command.
// The destructor runs on every exit path (there are no exceptions in this
// build) and hands the pins back to the I2C2 alternate function via
// pinctrl. Safe because the shell is the only bus user in this build: the
// STM32 driver leaves the peripheral disabled between transfers and the
// tug encoder thread is compiled out.
class bitbang_session : public pin_ops {
public:
    explicit bitbang_session(bool swap)
        : sda_pin(swap ? 1u : 0u), scl_pin(swap ? 0u : 1u) {}
    ~bitbang_session() override {
        int const ret{tof_diag_pinctrl_restore()};
        if (ret != 0)
            LOG_ERR("pinctrl restore failed (%d): PF0/PF1 may still be GPIO, "
                    "hardware i2c2 unusable until reboot", ret);
    }
    int configure() const {
        if (!device_is_ready(gpiof_dev))
            return -ENODEV;
        gpio_flags_t const flags{GPIO_INPUT | GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN | GPIO_PULL_UP};
        int ret{gpio_pin_configure(gpiof_dev, sda_pin, flags)};
        if (ret == 0)
            ret = gpio_pin_configure(gpiof_dev, scl_pin, flags);
        return ret;
    }
    void sda_drive_low() override { gpio_pin_set_raw(gpiof_dev, sda_pin, 0); }
    void sda_release() override { gpio_pin_set_raw(gpiof_dev, sda_pin, 1); }
    bool sda_read() override { return gpio_pin_get_raw(gpiof_dev, sda_pin) > 0; }
    void scl_drive_low() override { gpio_pin_set_raw(gpiof_dev, scl_pin, 0); }
    void scl_release() override { gpio_pin_set_raw(gpiof_dev, scl_pin, 1); }
    bool scl_read() override { return gpio_pin_get_raw(gpiof_dev, scl_pin) > 0; }
    void delay_half_bit() override { k_busy_wait(10); } // ~50 kHz, diagnostic speed
private:
    gpio_pin_t const sda_pin, scl_pin;
};

int cmd_info(const struct shell *shell, size_t, char **)
{
    shell_print(shell, "i2c2 (PF0/PF1 via PCA9615): %s",
                device_is_ready(i2c2_dev) ? "ready" : "NOT READY");
    shell_print(shell, "lpn role: %s=%d clk=%d (1-4 into SparePinGPIO, -1 = unassigned)",
                "lpn", lpn_index + 1, clk_index + 1);
    for (int i{0}; i < 4; ++i) {
        int const level{gpio_pin_get_dt(&spare_pins[i])};
        shell_print(shell, "SparePinGPIO%d: level=%d speed=%s", i + 1, level,
                    tof_diag_spare_gpio_speed_label(i));
    }
    shell_print(shell, "note: SparePinGPIO5 is comm_mode, not touched by this tool");
    return 0;
}

int cmd_speed(const struct shell *shell, size_t, char **argv)
{
    uint32_t khz{0};
    uint32_t speed{0};
    if (parse_u32(argv[1], khz)) {
        if (khz == 100)
            speed = I2C_SPEED_STANDARD;
        else if (khz == 400)
            speed = I2C_SPEED_FAST;
        else if (khz == 1000)
            speed = I2C_SPEED_FAST_PLUS;
    }
    if (speed == 0) {
        shell_error(shell, "speed must be 100, 400 or 1000 (kHz)");
        return -EINVAL;
    }
    int const ret{i2c_configure(i2c2_dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed))};
    shell_print(shell, "i2c_configure(%u kHz): %d %s", khz, ret, errno_label(ret));
    return ret;
}

int cmd_scan(const struct shell *shell, size_t argc, char **argv)
{
    bool const use_read{argc > 1 && strcmp(argv[1], "read") == 0};
    int found{0};
    for (uint8_t addr{0x08}; addr <= 0x77; ++addr) {
        if (hw_probe(addr, use_read) == 0) {
            shell_print(shell, "0x%02x: ACK", addr);
            ++found;
        }
    }
    shell_print(shell, "scan done (%s probe), %d device(s)", use_read ? "read" : "write", found);
    return 0;
}

int cmd_probe(const struct shell *shell, size_t argc, char **argv)
{
    uint8_t addr{0};
    if (!parse_addr7(argv[1], addr)) {
        shell_error(shell, "bad 7-bit address: %s", argv[1]);
        return -EINVAL;
    }
    bool const use_read{argc > 2 && strcmp(argv[2], "read") == 0};
    uint32_t elapsed_us{0};
    int const ret{timed_i2c([&] { return hw_probe(addr, use_read); }, elapsed_us)};
    shell_print(shell, "hw probe 0x%02x (%s): %d %s, %u us", addr, use_read ? "read" : "write",
                ret, errno_label(ret), elapsed_us);
    return 0;
}

int cmd_bb_probe(const struct shell *shell, size_t argc, char **argv)
{
    uint8_t addr{0};
    if (!parse_addr7(argv[1], addr)) {
        shell_error(shell, "bad 7-bit address: %s", argv[1]);
        return -EINVAL;
    }
    bool const swap{argc > 2 && strcmp(argv[2], "swap") == 0};
    bitbang_session session(swap);
    int const ret{session.configure()};
    if (ret != 0) {
        shell_error(shell, "gpio configure failed: %d", ret);
        return ret;
    }
    master bus(session);
    probe_result const result{bus.probe(addr)};
    shell_print(shell, "bitbang probe 0x%02x [%s]: %s", addr,
                swap ? "swap: PF0=SCL PF1=SDA (board-net orientation)"
                     : "PF0=SDA PF1=SCL (I2C2 AF orientation)",
                to_string(result));
    return 0;
}

int cmd_bb_clear(const struct shell *shell, size_t argc, char **argv)
{
    bool const swap{argc > 1 && strcmp(argv[1], "swap") == 0};
    bitbang_session session(swap);
    int const ret{session.configure()};
    if (ret != 0) {
        shell_error(shell, "gpio configure failed: %d", ret);
        return ret;
    }
    master bus(session);
    if (bus.bus_clear()) {
        shell_print(shell, "bus clear done (9 clocks + STOP, %s)", swap ? "swap" : "no swap");
        return 0;
    }
    shell_error(shell, "bus clear FAILED: SCL held low throughout (%s) -- clocking cannot "
                       "recover a clamped clock; check power and the harness",
                swap ? "swap" : "no swap");
    return -EIO;
}

int configure_role_pin(int index)
{
    return gpio_pin_configure_dt(&spare_pins[index], GPIO_INPUT | GPIO_OUTPUT_INACTIVE);
}

int cmd_lpn_use(const struct shell *shell, size_t, char **argv)
{
    uint32_t lpn{0}, clk{0};
    if (!parse_u32(argv[1], lpn) || !parse_u32(argv[2], clk) ||
        lpn < 1 || lpn > 4 || clk < 1 || clk > 4 || lpn == clk) {
        shell_error(shell, "usage: lpn use <lpn 1-4> <clk 1-4>, distinct");
        return -EINVAL;
    }
    lpn_index = static_cast<int>(lpn) - 1;
    clk_index = static_cast<int>(clk) - 1;
    int ret{configure_role_pin(lpn_index)};
    if (ret == 0)
        ret = configure_role_pin(clk_index);
    // gpio_pin_configure_dt() above resets OSPEEDR to low speed on both
    // pins as a side effect (STM32 driver detail, see tof_diag_pinctrl.c);
    // reapply very-high-speed to all four candidates every time roles are
    // (re)assigned, not just once at boot.
    int const speed_ret{tof_diag_spare_gpio_speed_apply()};
    if (ret == 0)
        ret = speed_ret;
    shell_print(shell, "lpn=SparePinGPIO%u clk=SparePinGPIO%u (configure: %d, speed reapply: %d), "
                       "both driven low",
                lpn, clk, ret, speed_ret);
    return ret;
}

bool roles_assigned(const struct shell *shell)
{
    if (lpn_index < 0 || clk_index < 0) {
        shell_error(shell, "assign roles first: lpn use <lpn 1-4> <clk 1-4>");
        return false;
    }
    return true;
}

// Clock timing, split into the three segments of one pulse plus a settle.
// Split rather than a single "pulse width" on purpose: a chain position can
// fail to latch because the high time is too short for the far end to see an
// edge, OR because the low time is too short for the far end to return to a
// low level, in which case the next attempt produces no rising edge at all.
// One combined knob cannot tell those two apart. Defaults reproduce the
// original fixed behaviour so an unconfigured build measures the old baseline.
struct clk_timing {
    uint32_t low_before_us{100};
    uint32_t high_us{100};
    uint32_t low_after_us{100};
    uint32_t post_pulse_settle_ms{0};
};
clk_timing clk_t;

// Upper bounds exist only to keep a typo from parking the shell for minutes;
// they are not a statement about what the hardware needs.
constexpr uint32_t kMaxSegmentUs{100000};
constexpr uint32_t kMaxSettleMs{10000};

void clk_pulse()
{
    // Drive low first: if a previous run left the line high, a naive
    // high-then-low sequence produces no rising edge and the chain silently
    // fails to advance. This mirrors the production glue's ordering.
    gpio_pin_set_dt(&spare_pins[clk_index], 0);
    k_busy_wait(clk_t.low_before_us);
    gpio_pin_set_dt(&spare_pins[clk_index], 1);
    k_busy_wait(clk_t.high_us);
    gpio_pin_set_dt(&spare_pins[clk_index], 0);
    k_busy_wait(clk_t.low_after_us);
    if (clk_t.post_pulse_settle_ms > 0)
        k_msleep(clk_t.post_pulse_settle_ms);
}

int cmd_lpn_timing(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 1) {
        shell_print(shell,
                    "clk timing: low_before=%u us high=%u us low_after=%u us settle=%u ms",
                    clk_t.low_before_us, clk_t.high_us, clk_t.low_after_us,
                    clk_t.post_pulse_settle_ms);
        return 0;
    }
    // All four or none: a partial set would silently leave some segments at
    // the previous run's value, which is exactly how a timing matrix gets
    // misattributed.
    if (argc != 5) {
        shell_error(shell, "usage: lpn timing <low_before_us> <high_us> <low_after_us> <settle_ms>");
        return -EINVAL;
    }
    clk_timing next{};
    if (!parse_u32(argv[1], next.low_before_us) || !parse_u32(argv[2], next.high_us) ||
        !parse_u32(argv[3], next.low_after_us) || !parse_u32(argv[4], next.post_pulse_settle_ms)) {
        shell_error(shell, "usage: lpn timing <low_before_us> <high_us> <low_after_us> <settle_ms>");
        return -EINVAL;
    }
    if (next.low_before_us > kMaxSegmentUs || next.high_us > kMaxSegmentUs ||
        next.low_after_us > kMaxSegmentUs) {
        shell_error(shell, "each us segment must be <= %u", kMaxSegmentUs);
        return -EINVAL;
    }
    if (next.post_pulse_settle_ms > kMaxSettleMs) {
        shell_error(shell, "settle must be <= %u ms", kMaxSettleMs);
        return -EINVAL;
    }
    clk_t = next;
    shell_print(shell,
                "clk timing set: low_before=%u us high=%u us low_after=%u us settle=%u ms",
                clk_t.low_before_us, clk_t.high_us, clk_t.low_after_us,
                clk_t.post_pulse_settle_ms);
    return 0;
}

int cmd_lpn_alloff(const struct shell *shell, size_t argc, char **argv)
{
    if (!roles_assigned(shell))
        return -EINVAL;
    uint32_t pulses{8};
    if (argc > 1 && !parse_u32(argv[1], pulses)) {
        shell_error(shell, "bad pulse count");
        return -EINVAL;
    }
    gpio_pin_set_dt(&spare_pins[lpn_index], 0);
    k_busy_wait(clk_t.low_before_us);
    for (uint32_t i{0}; i < pulses; ++i)
        clk_pulse();
    shell_print(shell,
                "LPn low, %u clock pulses: all chain positions disabled "
                "[low_before=%u us high=%u us low_after=%u us settle=%u ms]",
                pulses, clk_t.low_before_us, clk_t.high_us, clk_t.low_after_us,
                clk_t.post_pulse_settle_ms);
    return 0;
}

int cmd_lpn_first(const struct shell *shell, size_t, char **)
{
    if (!roles_assigned(shell))
        return -EINVAL;
    gpio_pin_set_dt(&spare_pins[lpn_index], 1);
    shell_print(shell, "LPn high: chain position 1 comms-enabled");
    return 0;
}

int cmd_lpn_pulse(const struct shell *shell, size_t argc, char **argv)
{
    if (!roles_assigned(shell))
        return -EINVAL;
    uint32_t pulses{1};
    if (argc > 1 && !parse_u32(argv[1], pulses)) {
        shell_error(shell, "bad pulse count");
        return -EINVAL;
    }
    for (uint32_t i{0}; i < pulses; ++i)
        clk_pulse();
    // The timing is echoed with every pulse so that a transcript can never be
    // read back without knowing which configuration produced it.
    shell_print(shell,
                "%u clock pulse(s): LPn level shifted along the chain "
                "[low_before=%u us high=%u us low_after=%u us settle=%u ms]",
                pulses, clk_t.low_before_us, clk_t.high_us, clk_t.low_after_us,
                clk_t.post_pulse_settle_ms);
    return 0;
}

int cmd_pin(const struct shell *shell, size_t, char **argv)
{
    uint32_t index{0};
    if (!parse_u32(argv[1], index) || index < 1 || index > 4) {
        shell_error(shell, "pin index must be 1-4 (SparePinGPIO5 is comm_mode, off limits)");
        return -EINVAL;
    }
    const struct gpio_dt_spec *pin{&spare_pins[index - 1]};
    if (strcmp(argv[2], "read") == 0) {
        gpio_pin_configure_dt(pin, GPIO_INPUT);
        shell_print(shell, "SparePinGPIO%u = %d (input)", index, gpio_pin_get_dt(pin));
        return 0;
    }
    if (strcmp(argv[2], "0") == 0 || strcmp(argv[2], "1") == 0) {
        int const level{argv[2][0] - '0'};
        gpio_pin_configure_dt(pin, GPIO_INPUT | GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(pin, level);
        shell_print(shell, "SparePinGPIO%u driven %d", index, level);
        return 0;
    }
    shell_error(shell, "usage: pin <1-4> <0|1|read>");
    return -EINVAL;
}

int cmd_vl53_id_l7(const struct shell *shell, size_t, char **argv)
{
    uint8_t addr{0};
    if (!parse_addr7(argv[1], addr)) {
        shell_error(shell, "bad 7-bit address: %s", argv[1]);
        return -EINVAL;
    }
    // ULD is_alive sequence: page 0, read 0x0000/0x0001, page 2.
    uint8_t id{0}, revision{0};
    int ret{vl53_wr8(addr, 0x7fff, 0x00)};
    if (ret == 0)
        ret = vl53_rd(addr, 0x0000, &id, 1);
    if (ret == 0)
        ret = vl53_rd(addr, 0x0001, &revision, 1);
    int const restore{vl53_wr8(addr, 0x7fff, 0x02)};
    if (ret != 0) {
        shell_error(shell, "read failed: %d %s (page restore: %d)", ret, errno_label(ret), restore);
        return ret;
    }
    if (restore != 0) {
        // No verdict on a device left on the wrong register page: the id
        // bytes were read, but nothing after this point can be trusted.
        shell_error(shell, "0x%02x: device_id=0x%02x revision=0x%02x but page-2 restore FAILED (%d): "
                           "device left on page 0, retry before trusting further reads",
                    addr, id, revision, restore);
        return restore;
    }
    shell_print(shell, "0x%02x: device_id=0x%02x revision=0x%02x -> %s (VL53L7CX expects 0xf0/0x02)",
                addr, id, revision, (id == 0xf0 && revision == 0x02) ? "MATCH" : "MISMATCH");
    return 0;
}

int cmd_vl53_id_l4(const struct shell *shell, size_t, char **argv)
{
    uint8_t addr{0};
    if (!parse_addr7(argv[1], addr)) {
        shell_error(shell, "bad 7-bit address: %s", argv[1]);
        return -EINVAL;
    }
    // VL53L4CX identification: model id 0x010f, module type 0x0110.
    uint8_t buf[2]{};
    int const ret{vl53_rd(addr, 0x010f, buf, sizeof buf)};
    if (ret != 0) {
        shell_error(shell, "read failed: %d %s", ret, errno_label(ret));
        return ret;
    }
    shell_print(shell, "0x%02x: model_id=0x%02x module_type=0x%02x -> %s (VL53L4CX expects 0xeb/0xaa)",
                addr, buf[0], buf[1], (buf[0] == 0xeb && buf[1] == 0xaa) ? "MATCH" : "MISMATCH");
    return 0;
}

void print_l7_postmortem(const struct shell *shell, const lexxhard::tof_diag_readdress::l7_result &r,
                         uint8_t old_addr, uint8_t new_addr)
{
    shell_print(shell, "post-mortem (LPn untouched): 0x%02x %s, 0x%02x %s",
                old_addr, r.old_probe_rc == 0 ? "ACK" : "silent",
                new_addr, r.new_probe_rc == 0 ? "ACK" : "silent");
    if (r.old_probe_rc == 0 && r.new_probe_rc != 0)
        shell_print(shell, "verdict: the address write did not take effect -- safe to stop");
    else if (r.old_probe_rc != 0 && r.new_probe_rc == 0)
        shell_print(shell, "verdict: device answers on the new address only");
    else if (r.old_probe_rc != 0 && r.new_probe_rc != 0)
        shell_print(shell, "verdict: no answer on either address -- capture SP1 level, power and an "
                           "analyser trace BEFORE lpn alloff (alloff changes the enable/signal "
                           "scene, and on L4 boards it also resets the dynamic address)");
    else
        shell_print(shell, "verdict: both addresses ACK -- address collision or multiple devices "
                           "enabled, stop immediately");
}

int cmd_vl53_setaddr_l7(const struct shell *shell, size_t, char **argv)
{
    namespace readdress = lexxhard::tof_diag_readdress;
    uint8_t old_addr{0}, new_addr{0};
    if (!parse_addr7(argv[1], old_addr) || !parse_addr7(argv[2], new_addr)) {
        shell_error(shell, "usage: vl53 setaddr_l7 <old7> <new7>");
        return -EINVAL;
    }
    // Register choice, staging, failure semantics and the post-mortem live
    // in the pure, host-tested helper; this is only the bus glue plus the
    // operator decision table.
    struct hw_ops final : readdress::i2c_ops {
        int probe(uint8_t addr7) override { return hw_probe(addr7, false); }
        int wr8(uint8_t addr7, uint16_t reg, uint8_t value) override { return vl53_wr8(addr7, reg, value); }
        int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) override { return vl53_rd(addr7, reg, buf, len); }
    } ops;
    auto const r{readdress::readdress_l7(ops, old_addr, new_addr)};
    switch (r.failed_at) {
    case readdress::l7_stage::validate:
        shell_error(shell, "old and new address are identical; a same-address move proves nothing");
        break;
    case readdress::l7_stage::collision:
        shell_error(shell, "0x%02x already ACKs: refusing to merge two devices onto one address", new_addr);
        break;
    case readdress::l7_stage::page_select:
        shell_error(shell, "page-0 select failed on 0x%02x: %d %s", old_addr, r.rc, errno_label(r.rc));
        print_l7_postmortem(shell, r, old_addr, new_addr);
        break;
    case readdress::l7_stage::addr_write:
        shell_error(shell, "address write (reg 0x0004) failed on 0x%02x: %d %s",
                    old_addr, r.rc, errno_label(r.rc));
        print_l7_postmortem(shell, r, old_addr, new_addr);
        break;
    case readdress::l7_stage::verify:
        if (r.rc == -ENODEV) {
            shell_error(shell, "0x%02x -> 0x%02x, device_id=0x%02x revision=0x%02x -> MISMATCH "
                               "(VL53L7CX expects 0xf0/0x02), not a success (page-2 restore: %d)",
                        old_addr, new_addr, r.device_id, r.revision, r.restore_rc);
        } else {
            shell_error(shell, "id read on new address 0x%02x failed: %d %s",
                        new_addr, r.rc, errno_label(r.rc));
            print_l7_postmortem(shell, r, old_addr, new_addr);
        }
        break;
    case readdress::l7_stage::page_restore:
        shell_error(shell, "page-2 restore failed on 0x%02x: %d %s -- device may be left on page 0, "
                           "retry before trusting further reads",
                    new_addr, r.rc, errno_label(r.rc));
        print_l7_postmortem(shell, r, old_addr, new_addr);
        break;
    case readdress::l7_stage::done:
        shell_print(shell, "0x%02x -> 0x%02x, device_id=0x%02x revision=0x%02x -> MATCH (VL53L7CX expects 0xf0/0x02)%s",
                    old_addr, new_addr, r.device_id, r.revision,
                    r.write_ack_lost ? " [note: address-write ACK was lost but the move verified; treat the link as suspect]" : "");
        break;
    }
    return r.rc;
}

int cmd_vl53_setaddr_l4(const struct shell *shell, size_t, char **argv)
{
    namespace readdress = lexxhard::tof_diag_readdress;
    uint8_t old_addr{0}, new_addr{0};
    if (!parse_addr7(argv[1], old_addr) || !parse_addr7(argv[2], new_addr)) {
        shell_error(shell, "usage: vl53 setaddr_l4 <old7> <new7>");
        return -EINVAL;
    }
    // The register choice, ordering and failure semantics live in the pure,
    // host-tested helper (tof_diag_readdress.cpp); this is only the bus glue.
    struct hw_ops final : readdress::i2c_ops {
        int probe(uint8_t addr7) override { return hw_probe(addr7, false); }
        int wr8(uint8_t addr7, uint16_t reg, uint8_t value) override { return vl53_wr8(addr7, reg, value); }
        int rd(uint8_t addr7, uint16_t reg, uint8_t *buf, size_t len) override { return vl53_rd(addr7, reg, buf, len); }
    } ops;
    auto const r{readdress::readdress_l4(ops, old_addr, new_addr)};
    switch (r.failed_at) {
    case readdress::stage::validate:
        shell_error(shell, "old and new address are identical; a same-address move proves nothing");
        break;
    case readdress::stage::collision:
        shell_error(shell, "0x%02x already ACKs: refusing to merge two devices onto one address", new_addr);
        break;
    case readdress::stage::write:
        shell_error(shell, "address write failed on 0x%02x: %d %s", old_addr, r.rc, errno_label(r.rc));
        break;
    case readdress::stage::read:
        shell_error(shell, "device did not answer on new address 0x%02x: %d %s",
                    new_addr, r.rc, errno_label(r.rc));
        break;
    case readdress::stage::verify:
        shell_error(shell, "0x%02x -> 0x%02x, model_id=0x%02x module_type=0x%02x -> MISMATCH "
                           "(VL53L4CX expects 0xeb/0xaa), not a success",
                    old_addr, new_addr, r.model_id, r.module_type);
        break;
    case readdress::stage::done:
        shell_print(shell, "0x%02x -> 0x%02x, model_id=0x%02x module_type=0x%02x -> MATCH (VL53L4CX expects 0xeb/0xaa)",
                    old_addr, new_addr, r.model_id, r.module_type);
        break;
    }
    return r.rc;
}

int cmd_stress_probe(const struct shell *shell, size_t, char **argv)
{
    uint8_t addr{0};
    uint32_t count{0};
    if (!parse_addr7(argv[1], addr) || !parse_u32(argv[2], count) || count == 0 || count > 100000) {
        shell_error(shell, "usage: stress probe <addr7> <n 1-100000>");
        return -EINVAL;
    }
    stress_stats stats;
    for (uint32_t i{0}; i < count; ++i) {
        uint32_t elapsed_us{0};
        int const ret{timed_i2c([&] { return hw_probe(addr, false); }, elapsed_us)};
        stats.count(ret, elapsed_us);
    }
    shell_print(shell, "note: address-probe stress only proves ACK/NACK stability, not data integrity");
    print_stats(shell, stats);
    return 0;
}

// Shared body for the two representative-transfer commands. Reads `len`
// bytes from `reg` `count` times; the first successful read becomes the
// reference and later payloads are compared against it. This -- not the
// address probe -- is the evidence class for installed-harness quality.
int stress_xfer_common(const struct shell *shell, char **argv, bool reg16)
{
    static uint8_t reference[384], current[384];
    uint8_t addr{0};
    uint32_t reg{0}, len{0}, count{0};
    if (!parse_addr7(argv[1], addr) || !parse_u32(argv[2], reg) ||
        !parse_u32(argv[3], len) || !parse_u32(argv[4], count) ||
        len == 0 || len > sizeof reference || count == 0 || count > 100000 ||
        reg > (reg16 ? 0xffffu : 0xffu)) {
        shell_error(shell, "usage: stress %s <addr7> <reg> <len 1-384> <n 1-100000>",
                    reg16 ? "xfer" : "xfer8");
        return -EINVAL;
    }
    stress_stats stats;
    bool have_reference{false};
    for (uint32_t i{0}; i < count; ++i) {
        uint32_t elapsed_us{0};
        int const ret{timed_i2c([&] {
            if (reg16)
                return vl53_rd(addr, static_cast<uint16_t>(reg), current, len);
            uint8_t const index{static_cast<uint8_t>(reg)};
            return i2c_write_read(i2c2_dev, addr, &index, 1, current, len);
        }, elapsed_us)};
        stats.count(ret, elapsed_us);
        if (ret == 0) {
            if (!have_reference) {
                memcpy(reference, current, len);
                have_reference = true;
            } else if (memcmp(reference, current, len) != 0) {
                ++stats.data_mismatch;
            }
        }
    }
    shell_print(shell, "%u transfers of %u B from reg 0x%x @0x%02x", count, len, reg, addr);
    print_stats(shell, stats);
    if (stats.data_mismatch > 0)
        shell_print(shell, "note: mismatches can be legitimate on live registers; use a static region");
    return 0;
}

int cmd_stress_xfer(const struct shell *shell, size_t, char **argv)
{
    return stress_xfer_common(shell, argv, true);
}

int cmd_stress_xfer8(const struct shell *shell, size_t, char **argv)
{
    return stress_xfer_common(shell, argv, false);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_bitbang,
    SHELL_CMD_ARG(probe, NULL, "probe <addr7> [swap] - GPIO bit-bang probe; swap = PF0=SCL/PF1=SDA", cmd_bb_probe, 2, 1),
    SHELL_CMD_ARG(clear, NULL, "clear [swap] - 9 clocks + STOP bus recovery", cmd_bb_clear, 1, 1),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lpn,
    SHELL_CMD_ARG(use, NULL, "use <lpn 1-4> <clk 1-4> - assign SparePinGPIO roles", cmd_lpn_use, 3, 0),
    SHELL_CMD_ARG(alloff, NULL, "alloff [pulses=8] - LPn low + pulses: disable whole chain", cmd_lpn_alloff, 1, 1),
    SHELL_CMD_ARG(first, NULL, "first - LPn high: enable chain position 1", cmd_lpn_first, 1, 0),
    SHELL_CMD_ARG(pulse, NULL, "pulse [n=1] - shift LPn level along the chain", cmd_lpn_pulse, 1, 1),
    SHELL_CMD_ARG(timing, NULL,
                  "timing [low_before_us high_us low_after_us settle_ms] - show or set clock timing",
                  cmd_lpn_timing, 1, 4),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_vl53,
    SHELL_CMD_ARG(id_l7, NULL, "id_l7 <addr7> - VL53L7CX device id (expect 0xf0/0x02)", cmd_vl53_id_l7, 2, 0),
    SHELL_CMD_ARG(id_l4, NULL, "id_l4 <addr7> - VL53L4CX model id (expect 0xeb/0xaa)", cmd_vl53_id_l4, 2, 0),
    SHELL_CMD_ARG(setaddr_l7, NULL, "setaddr_l7 <old7> <new7> - reassign VL53L7CX address", cmd_vl53_setaddr_l7, 3, 0),
    SHELL_CMD_ARG(setaddr_l4, NULL, "setaddr_l4 <old7> <new7> - reassign VL53L4CX address (VL53L1 core, reg 0x0001)", cmd_vl53_setaddr_l4, 3, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_stress,
    SHELL_CMD_ARG(probe, NULL, "probe <addr7> <n> - ACK/NACK stability only", cmd_stress_probe, 3, 0),
    SHELL_CMD_ARG(xfer, NULL, "xfer <addr7> <reg16> <len> <n> - representative transfers, 16-bit index (VL53)", cmd_stress_xfer, 5, 0),
    SHELL_CMD_ARG(xfer8, NULL, "xfer8 <addr7> <reg8> <len> <n> - representative transfers, 8-bit index (ADS7138)", cmd_stress_xfer8, 5, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tof_diag,
    SHELL_CMD(info, NULL, "i2c2 readiness, pin roles and levels", cmd_info),
    SHELL_CMD_ARG(speed, NULL, "speed <100|400|1000> - reconfigure i2c2 bitrate (kHz)", cmd_speed, 2, 0),
    SHELL_CMD_ARG(scan, NULL, "scan [read] - hw probe 0x08-0x77", cmd_scan, 1, 1),
    SHELL_CMD_ARG(probe, NULL, "probe <addr7> [read] - single hw probe", cmd_probe, 2, 1),
    SHELL_CMD(bitbang, &sub_bitbang, "GPIO bit-bang master (crossed-pin diagnosis)", NULL),
    SHELL_CMD(lpn, &sub_lpn, "LPn shift-register chain enable", NULL),
    SHELL_CMD_ARG(pin, NULL, "pin <1-4> <0|1|read> - raw SparePinGPIO control", cmd_pin, 3, 0),
    SHELL_CMD(vl53, &sub_vl53, "VL53 identification and addressing", NULL),
    SHELL_CMD(stress, &sub_stress, "repeated-operation error statistics", NULL),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(tof_diag, &sub_tof_diag, "ToF I2C bring-up diagnostics (DIAG BUILD ONLY)", NULL);

} // namespace

void init()
{
    LOG_WRN("TOF_I2C_DIAG build: tug encoder polling disabled, `tof_diag` shell active");
    if (!device_is_ready(i2c2_dev))
        LOG_ERR("i2c2 not ready");
    int const speed_ret{tof_diag_spare_gpio_speed_apply()};
    if (speed_ret != 0)
        LOG_ERR("SparePinGPIO1-4 very-high-speed pinctrl apply failed (%d)", speed_ret);
}

}

#endif // TOF_I2C_DIAG
