/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DEV ONLY. See tof_diag_i2c.hpp for what this measures, and for what measuring it costs.
 */

#include "tof_diag_i2c.hpp"

#if defined(TOF_DIAG_HANG)

#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

namespace lexxhard::tof_diag_i2c {

namespace {

static_assert(sizeof(record) == 0x634, "the layout is part of the reading procedure");
static_assert(offsetof(record, ev_none) == 0x038);
static_assert(offsetof(record, er_flag) == 0x03c);
static_assert(offsetof(record, er_none) == 0x054);
static_assert(offsetof(record, rxne_with_len0) == 0x058);
static_assert(offsetof(record, advance_no_data) == 0x068);
static_assert(offsetof(record, last_isr) == 0x074);
static_assert(offsetof(record, last_err_isr) == 0x08c);
static_assert(offsetof(record, orphan_rxne) == 0x0a4);
static_assert(offsetof(record, port_begin) == 0x0b4);
static_assert(offsetof(record, port_buf_base) == 0x0dc);
static_assert(offsetof(record, timingr_boot) == 0x0ec);
static_assert(offsetof(record, coarse_ms) == 0x0f4);
static_assert(offsetof(record, pre_arm) == 0x100);
static_assert(offsetof(record, post_arm) == 0x120);
static_assert(offsetof(record, ring_next) == 0x220);
static_assert(offsetof(record, ring) == 0x230);
static_assert(offsetof(record, magic_end) == 0x630);
/* Inside DTCM, and clear of the 0x1f4-byte progress record that starts at 0x2001F000. */
static_assert(kRecordAddress >= 0x2001F000 + 0x1f4);
static_assert(kRecordAddress >= DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm)));
static_assert(kRecordAddress + sizeof(record) <=
              DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm)) + DT_REG_SIZE(DT_CHOSEN(zephyr_dtcm)));

/* The bus this is about. Every other I2C controller's interrupts are ignored, so the counts mean
 * what the decoder says they mean. */
const struct device *const bus{DEVICE_DT_GET(DT_NODELABEL(i2c2))};
constexpr uintptr_t kBase{DT_REG_ADDR(DT_NODELABEL(i2c2))};
constexpr uintptr_t kCr1{kBase + 0x00};
constexpr uintptr_t kCr2{kBase + 0x04};
constexpr uintptr_t kTimingr{kBase + 0x10};
constexpr uintptr_t kIsr{kBase + 0x18};

constexpr unsigned kIrqEvent{DT_IRQ_BY_NAME(DT_NODELABEL(i2c2), event, irq)};
constexpr unsigned kIrqError{DT_IRQ_BY_NAME(DT_NODELABEL(i2c2), error, irq)};
/* NVIC register files; one 32-bit word per 32 interrupts. */
constexpr uintptr_t kNvicIser{0xE000E100};
constexpr uintptr_t kNvicIspr{0xE000E200};
constexpr uintptr_t kNvicIabr{0xE000E300};

/* ev_flag[] and er_flag[] are indexed by ISR bit position, which lets the interrupt walk only the
 * bits that are actually set instead of testing all of them. The asserts are what makes that
 * shortcut safe: if anyone reorders the buckets, this stops compiling rather than miscounting. */
constexpr int kEventShift{1};  // TXIS is bit 1, and the nine that follow are consecutive
constexpr int kErrorShift{8};  // BERR is bit 8, likewise
constexpr uint32_t kEventCounted{((1U << kEventFlags) - 1U) << kEventShift};
constexpr uint32_t kErrorCounted{((1U << kErrorFlags) - 1U) << kErrorShift};
static_assert(kEventCounted == (bit_txis | bit_rxne | bit_addr | bit_nackf | bit_stopf | bit_tc |
                                bit_tcr | bit_berr | bit_arlo | bit_ovr));
static_assert(kErrorCounted == (bit_berr | bit_arlo | bit_ovr | bit_pecerr | bit_timeout |
                                bit_alert));
static_assert(kErrorCounted == kAnyErrorFlag);

/* Count the set bits only: an ordinary entry carries one or two, not ten. Cortex-M does this in a
 * rbit/clz pair, which is the difference between a handful of instructions per interrupt and a
 * ten-iteration loop -- and this measurement is about interrupt timing. */
inline void count_flags(volatile uint32_t *bucket, uint32_t isr, uint32_t counted, int shift)
{
    uint32_t bits{isr & counted};
    while (bits != 0U) {
        const int b{__builtin_ctz(bits)};
        bits &= bits - 1U;
        bucket[b - shift] = bucket[b - shift] + 1;
    }
}

volatile record &rec()
{
    return *reinterpret_cast<volatile record *>(kRecordAddress);
}

uint32_t reg(uintptr_t address)
{
    return *reinterpret_cast<volatile uint32_t *>(address);
}

/* What the interrupts copy instead of reading the clock; the 1 s sampler moves it. See the header
 * on the observer effect: a kernel clock read per I2C interrupt would be a heavier change to the
 * timing than the race being hunted.
 *
 * volatile because this is written in one interrupt (the board controller's 1 s timer) and read in
 * another (i2c2): a single word, so no tearing, but the compiler must not cache it across the I2C
 * hook. Today's code reloads it anyway; the keyword is the contract, not an optimisation barrier
 * this build happens to need. */
volatile uint32_t coarse_ms_{0};

/* Previous totals, so a window carries a rate rather than a running sum. Only the 1 s sampler
 * touches these. */
uint32_t last_event_total{0};
uint32_t last_error_total{0};
bool was_armed{false};
uint32_t post_arm_taken{0};

uint32_t nvic_bits()
{
    const uint32_t iser{reg(kNvicIser + 4 * (kIrqEvent / 32))};
    const uint32_t ispr{reg(kNvicIspr + 4 * (kIrqEvent / 32))};
    const uint32_t iabr{reg(kNvicIabr + 4 * (kIrqEvent / 32))};
    /* Both interrupts live in the same 32-interrupt group on this part; the static assert keeps
     * that assumption honest rather than silently reading the wrong word. */
    static_assert(kIrqEvent / 32 == kIrqError / 32);
    const uint32_t ev{1U << (kIrqEvent % 32)};
    const uint32_t er{1U << (kIrqError % 32)};
    return ((iser & ev) ? 1U : 0U) | ((ispr & ev) ? 2U : 0U) | ((iabr & ev) ? 4U : 0U) |
           ((iser & er) ? 16U : 0U) | ((ispr & er) ? 32U : 0U) | ((iabr & er) ? 64U : 0U);
}

void fill(volatile window &w, uint32_t now_ms, uint32_t event_delta, uint32_t error_delta,
          uint32_t sw_len)
{
    w.ms = now_ms;
    w.event_delta = event_delta;
    w.error_delta = error_delta;
    w.isr = reg(kIsr);
    w.cr1 = reg(kCr1);
    w.cr2 = reg(kCr2);
    w.nvic = nvic_bits();
    w.sw_len = sw_len;
}

} // namespace

int init_record_at_boot()
{
    volatile record &r{rec()};
    for (size_t i{0}; i < sizeof(record) / sizeof(uint32_t); ++i)
        reinterpret_cast<volatile uint32_t *>(kRecordAddress)[i] = 0;
    r.version = kVersion;
    r.magic_end = kMagicEnd;
    r.magic = kMagic;
    return 0;
}

void sample(uint32_t now_ms, bool armed)
{
    volatile record &r{rec()};
    if (r.magic != kMagic)
        return;

    coarse_ms_ = now_ms;
    r.coarse_ms = now_ms;

    const uint32_t event_total{r.event_entries};
    const uint32_t error_total{r.error_entries};
    const uint32_t event_delta{event_total - last_event_total};
    const uint32_t error_delta{error_total - last_error_total};
    last_event_total = event_total;
    last_error_total = error_total;
    r.timingr_last = reg(kTimingr);
    /* Not at boot: the bus is configured after PRE_KERNEL_1, so the first sample is the earliest
     * moment the value means anything. */
    if (r.timingr_boot == 0U)
        r.timingr_boot = r.timingr_last;

    const uint32_t sw_len{r.last_sw_len};

    /* The last quiet second before the risky path started: the same board's own healthy rate, which
     * is the only honest thing to compare a storm against. */
    if (!armed)
        fill(r.pre_arm, now_ms, event_delta, error_delta, sw_len);

    if (armed && !was_armed) {
        was_armed = true;
        post_arm_taken = 0;
    }
    /* Healthy WITH the L7 running: the second baseline, and the one that says what a normal grid
     * read costs in interrupts. */
    if (armed && post_arm_taken < static_cast<uint32_t>(kPostArm)) {
        fill(r.post_arm[post_arm_taken], now_ms, event_delta, error_delta, sw_len);
        ++post_arm_taken;
    }

    const uint32_t slot{r.ring_next % kWindows};
    fill(r.ring[slot], now_ms, event_delta, error_delta, sw_len);
    r.ring_next = r.ring_next + 1;
    r.windows_sampled = r.windows_sampled + 1;
}

int shell_status(const struct shell *sh)
{
    const volatile record &r{rec()};
    static const char *const ev_names[kEventFlags]{"TXIS", "RXNE", "ADDR", "NACKF", "STOPF",
                                                   "TC",   "TCR",  "BERR", "ARLO",  "OVR"};
    static const char *const er_names[kErrorFlags]{"BERR",   "ARLO",    "OVR",
                                                   "PECERR", "TIMEOUT", "ALERT"};
    shell_print(sh, "magic %08x/%08x v%u   windows %u   coarse %u ms", r.magic, r.magic_end,
                r.version, r.windows_sampled, r.coarse_ms);
    shell_print(sh, "entries  event %u  error %u  (ev none %u, er none %u)", r.event_entries,
                r.error_entries, r.ev_none, r.er_none);
    for (int i{0}; i < kEventFlags; ++i)
        shell_fprintf(sh, SHELL_NORMAL, "%s %u  ", ev_names[i], r.ev_flag[i]);
    shell_print(sh, "");
    for (int i{0}; i < kErrorFlags; ++i)
        shell_fprintf(sh, SHELL_NORMAL, "%s %u  ", er_names[i], r.er_flag[i]);
    shell_print(sh, "");
    shell_print(sh, "FAULT    orphan_rxne %u (first %u ms, isr 0x%08x cr2 0x%08x)  <- must stay 0",
                r.orphan_rxne, r.orphan_rxne_ms, r.orphan_rxne_isr, r.orphan_rxne_cr2);
    shell_print(sh, "backgnd  rxne_len0 %u (%u ms)  txis_len0 %u (%u ms)  advance_no_data %u (%u ms,"
                    " isr 0x%08x)",
                r.rxne_with_len0, r.rxne_with_len0_ms, r.txis_with_len0, r.txis_with_len0_ms,
                r.advance_no_data, r.advance_no_data_ms, r.advance_no_data_isr);
    shell_print(sh, "last ev  isr 0x%08x cr1 0x%08x cr2 0x%08x len %u buf 0x%08x at ~%u ms",
                r.last_isr, r.last_cr1, r.last_cr2, r.last_sw_len, r.last_sw_buf, r.last_entry_ms);
    shell_print(sh, "last er  isr 0x%08x cr1 0x%08x cr2 0x%08x len %u buf 0x%08x at ~%u ms",
                r.last_err_isr, r.last_err_cr1, r.last_err_cr2, r.last_err_sw_len,
                r.last_err_sw_buf, r.last_err_ms);
    shell_print(sh, "port     %u/%u  addr8 0x%02x addr7 0x%02x %s reg 0x%04x total %u chunk %u "
                    "off %u len %u base 0x%08x rc %d  (%u -> %u ms)",
                r.port_begin, r.port_end, r.port_addr8, r.port_addr,
                r.port_is_read ? "read" : "write", r.port_reg, r.port_total_len, r.port_chunk_index,
                r.port_chunk_off, r.port_chunk_len, r.port_buf_base,
                static_cast<int>(r.port_last_rc), r.port_begin_ms, r.port_end_ms);
    /* Only meaningful while the driver is on this chunk's payload: during the two register-index
     * bytes current.buf points at the port layer's own stack. */
    if (r.port_buf_base != 0U && r.last_sw_buf >= r.port_buf_base &&
        r.last_sw_buf <= r.port_buf_base + r.port_chunk_len)
        shell_print(sh, "         inside the chunk: %u of %u bytes (255 = the reload boundary)",
                    r.last_sw_buf - r.port_buf_base, r.port_chunk_len);
    shell_print(sh, "timingr  boot 0x%08x last 0x%08x", r.timingr_boot, r.timingr_last);
    shell_print(sh, "pre-arm  %u ms  ev/s %u  er/s %u  isr 0x%08x cr1 0x%08x cr2 0x%08x nvic 0x%02x",
                r.pre_arm.ms, r.pre_arm.event_delta, r.pre_arm.error_delta, r.pre_arm.isr,
                r.pre_arm.cr1, r.pre_arm.cr2, r.pre_arm.nvic);
    for (int i{0}; i < kPostArm; ++i) {
        if (r.post_arm[i].ms == 0U)
            continue;
        shell_print(sh, "post[%d]  %u ms  ev/s %u  er/s %u  isr 0x%08x", i, r.post_arm[i].ms,
                    r.post_arm[i].event_delta, r.post_arm[i].error_delta, r.post_arm[i].isr);
    }
    /* The last four rolling windows: enough to see a rate change starting, without flooding a
     * console that is also carrying the run. */
    for (int back{4}; back >= 1; --back) {
        if (r.ring_next < static_cast<uint32_t>(back))
            continue;
        const uint32_t slot{(r.ring_next - static_cast<uint32_t>(back)) % kWindows};
        shell_print(sh, "ring-%d   %u ms  ev/s %u  er/s %u  isr 0x%08x cr1 0x%08x len %u", back,
                    r.ring[slot].ms, r.ring[slot].event_delta, r.ring[slot].error_delta,
                    r.ring[slot].isr, r.ring[slot].cr1, r.ring[slot].sw_len);
    }
    return 0;
}

void port_begin(uint32_t addr8, uint16_t addr7, bool is_read, uint16_t reg_index,
                uint32_t total_len, uint32_t chunk_index, uint32_t chunk_off, uint32_t chunk_len,
                uintptr_t buf_base)
{
    volatile record &r{rec()};
    r.port_addr8 = addr8;
    r.port_addr = addr7;
    r.port_is_read = is_read ? 1U : 0U;
    r.port_reg = reg_index;
    r.port_total_len = total_len;
    r.port_chunk_index = chunk_index;
    r.port_chunk_off = chunk_off;
    r.port_chunk_len = chunk_len;
    r.port_buf_base = static_cast<uint32_t>(buf_base);
    r.port_begin_ms = k_uptime_get_32();
    r.port_begin = r.port_begin + 1;
}

void port_end(int rc)
{
    volatile record &r{rec()};
    r.port_last_rc = static_cast<uint32_t>(rc);
    r.port_end_ms = k_uptime_get_32();
    r.port_end = r.port_end + 1;
}

} // namespace lexxhard::tof_diag_i2c

/* The hooks. They run in the I2C interrupt, at a rate that may itself be the fault being measured,
 * so the fast path makes no kernel call: no logging, no atomics, no clock. The event ISR of one
 * controller is the only writer of these fields and it cannot preempt itself. The one exception is
 * the first occurrence of each signature, which reads the real clock once because that instant is
 * the point of the exercise. */
extern "C" void lexx_i2c_forensics_event(const struct device *dev, uint32_t isr, uint32_t cr1,
                                         uint32_t cr2, uint32_t sw_len, const uint8_t *sw_buf)
{
    using namespace lexxhard::tof_diag_i2c;
    if (dev != bus)
        return;
    volatile record &r{rec()};
    if (r.magic != kMagic)
        return;

    r.event_entries = r.event_entries + 1;
    count_flags(r.ev_flag, isr, kEventCounted, kEventShift);

    const uint32_t what{classify_event(isr, sw_len)};
    if (what & entry_no_flag)
        r.ev_none = r.ev_none + 1;
    if (what & entry_advance_no_data) {
        if (r.advance_no_data == 0U) {
            r.advance_no_data_ms = k_uptime_get_32();
            r.advance_no_data_isr = isr;
        }
        r.advance_no_data = r.advance_no_data + 1;
    }
    if (what & entry_rxne_with_len0) {
        if (r.rxne_with_len0 == 0U)
            r.rxne_with_len0_ms = k_uptime_get_32();
        r.rxne_with_len0 = r.rxne_with_len0 + 1;
    }
    if (what & entry_orphan_rxne) {
        if (r.orphan_rxne == 0U) {
            r.orphan_rxne_ms = k_uptime_get_32();
            r.orphan_rxne_isr = isr;
            r.orphan_rxne_cr2 = cr2;
        }
        r.orphan_rxne = r.orphan_rxne + 1;
    }
    if (what & entry_txis_with_len0) {
        if (r.txis_with_len0 == 0U)
            r.txis_with_len0_ms = k_uptime_get_32();
        r.txis_with_len0 = r.txis_with_len0 + 1;
    }

    r.last_isr = isr;
    r.last_cr1 = cr1;
    r.last_cr2 = cr2;
    r.last_sw_len = sw_len;
    r.last_sw_buf = reinterpret_cast<uintptr_t>(sw_buf);
    r.last_entry_ms = coarse_ms_;
}

extern "C" void lexx_i2c_forensics_error(const struct device *dev, uint32_t isr, uint32_t cr1,
                                         uint32_t cr2, uint32_t sw_len, const uint8_t *sw_buf)
{
    using namespace lexxhard::tof_diag_i2c;
    if (dev != bus)
        return;
    volatile record &r{rec()};
    if (r.magic != kMagic)
        return;

    r.error_entries = r.error_entries + 1;
    count_flags(r.er_flag, isr, kErrorCounted, kErrorShift);
    if ((isr & kAnyErrorFlag) == 0U)
        r.er_none = r.er_none + 1;

    /* Kept separately from the event snapshot: an error that starts the trouble would otherwise be
     * overwritten within microseconds by the entries that follow it. */
    r.last_err_isr = isr;
    r.last_err_cr1 = cr1;
    r.last_err_cr2 = cr2;
    r.last_err_sw_len = sw_len;
    r.last_err_sw_buf = reinterpret_cast<uintptr_t>(sw_buf);
    r.last_err_ms = coarse_ms_;
}

extern "C" void lexx_tof_port_begin(uint32_t addr8, uint16_t addr7, int is_read, uint16_t reg_index,
                                    uint32_t total_len, uint32_t chunk_index, uint32_t chunk_off,
                                    uint32_t chunk_len, const uint8_t *buf_base)
{
    lexxhard::tof_diag_i2c::port_begin(addr8, addr7, is_read != 0, reg_index, total_len, chunk_index,
                                       chunk_off, chunk_len,
                                       reinterpret_cast<uintptr_t>(buf_base));
}

extern "C" void lexx_tof_port_end(int rc)
{
    lexxhard::tof_diag_i2c::port_end(rc);
}

#endif // TOF_DIAG_HANG
