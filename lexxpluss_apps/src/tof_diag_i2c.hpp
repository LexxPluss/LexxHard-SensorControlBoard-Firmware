/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DEV ONLY: interrupt-level forensics for the i2c2 bus, built only into the L7 hang-isolation
 * images (TOF_DIAG_HANG).
 *
 * WHY IT EXISTS. The diag1 long run of 2026-09-23 stopped every thread on the SCB in the same
 * millisecond -- acquisition inside an L7 read of sensor 0, the system work queue and the zcan loop
 * with nothing in flight -- while the board controller's 1 s timer callback went on running for
 * another 5.7 s until the watchdog withheld the feed. Thread level dead, interrupt level alive.
 *
 * MEASURED, 2026-09-23 16:33 (run2, evidence in allmemory/hanging_object/l7_e2e_2026-09-22):
 * the board wedges at interrupt level on i2c2. For fourteen consecutive one-second windows the
 * state was identical -- ISR RXNE|TXE|BUSY, CR1 with RXIE still enabled, the driver's current.len
 * already 0, CR2 holding NBYTES 65 with RELOAD still set, and TC, TCR, STOPF and NACKF all clear.
 * The event ISR then takes no branch at all: it will not read RXDR (that is guarded by
 * current.len), it will not generate a STOP and it will not disable the interrupt, so it is
 * re-entered about 890,000 times a second against a healthy 35,500. Every thread starves, the
 * 500 ms transfer timeout expires in kernel time but its thread never runs again, and only the
 * timer callback -- interrupt level -- survives to let the watchdog reset the board.
 *
 * The hypothesis this block was first built to test is REFUTED: advance_no_data stayed 0 through
 * 27 million interrupts, so the ISR's unconditional buffer advance is not how the desync starts.
 * What starts it is still open; the registers point at the final reload segment.
 *
 * OBSERVER EFFECT. Measuring an interrupt from inside it is not free, and this is a race about
 * interrupt timing, so the cost is kept where it can be stated rather than pretended away. The fast
 * path makes no kernel call at all: it compares flags, increments counters and stores registers. In
 * particular it does NOT read the uptime -- the 1 s sampler leaves a coarse epoch behind and the
 * fast path copies that, so ordinary entry timestamps are good to about a second. A real clock read
 * happens at most four times in the life of a boot: once on the FIRST occurrence of each of the
 * four things worth an exact instant, where that instant is what the whole run is for. What remains
 * is still a handful of instructions added to every I2C interrupt: if a run with this
 * instrumentation stops reproducing
 * the hang, the honest reading is "the window moved", not "the fault is gone".
 *
 * WHAT EACH COUNTER ACTUALLY MEANS, now that one run has been read:
 *   - orphan_rxne_len0     RXNE with current.len == 0 and NONE of TC, TCR, STOPF or NACKF. This is
 *                          the wedged state itself: the entry the ISR cannot service and cannot
 *                          clear. It is the fault indicator.
 *   - rxne_with_len0       RXNE with current.len == 0, whatever else is set. This is NOT a fault:
 *                          it fired 13.8 million times in one run and tracked the L7 reads about
 *                          one for one while the board was healthy, because the state is reached
 *                          routinely and left again within microseconds. Kept as the background
 *                          rate that orphan_rxne_len0 has to be read against.
 *   - advance_no_data      an entry with current.len != 0 carrying neither RXNE nor TXIS. Measured
 *                          at 0; kept because a change would mean something new.
 *   - event_entries delta  per second. A storm is orders of magnitude above the same board's own
 *                          healthy windows, which is why the pre-arm and post-arm windows are kept
 *                          rather than compared against an assumed "normal" rate.
 *   - ev_none              entries whose flags had already gone by the time the ISR read them. The
 *                          later upstream rework of this driver exists because that shape is real.
 *   - er_flag / last_err_* every error source ERRIE can raise, each in its own bucket, plus the raw
 *                          registers of the last one. An error the old ISR does not clear is the
 *                          most direct way to get an interrupt that re-enters for ever, so "some
 *                          error we did not classify" must never be an available answer.
 *
 * If all of these stay flat while the threads stop anyway, i2c2 is not the culprit and the next
 * suspects are a scheduler lock or memory corruption.
 */

#pragma once

#include <stdint.h>

#if defined(TOF_DIAG_HANG)

struct shell;

namespace lexxhard::tof_diag_i2c {

/* The 0x1f4-byte progress record sits at 0x2001F000 and ends at 0x2001F1F4. This block starts at
 * the next 0x100 boundary and is read back the same way, one word at a time with `devmem`. */
inline constexpr uintptr_t kRecordAddress{0x2001F200};
inline constexpr uint32_t kMagic{0x46433249};    // "I2CF"
inline constexpr uint32_t kMagicEnd{0x45433249}; // "I2CE"
/* 4: orphan_rxne and its three words were inserted at 0x0a4 and everything after them moved. A
 * reader that does not check this will decode a version 3 block at the wrong offsets and believe
 * the result, so the decoder refuses anything but an exact match. */
inline constexpr uint32_t kVersion{4};
inline constexpr int kWindows{32};    // rolling, the last 32 seconds before the reset
inline constexpr int kPostArm{8};     // kept: the first 8 seconds of the risky path
inline constexpr int kEventFlags{10}; // TXIS RXNE ADDR NACKF STOPF TC TCR BERR ARLO OVR
inline constexpr int kErrorFlags{6};  // BERR ARLO OVR PECERR TIMEOUT ALERT

/* I2C_ISR bit positions. */
enum isr_bit : uint32_t {
    bit_txis = 1U << 1,
    bit_rxne = 1U << 2,
    bit_addr = 1U << 3,
    bit_nackf = 1U << 4,
    bit_stopf = 1U << 5,
    bit_tc = 1U << 6,
    bit_tcr = 1U << 7,
    bit_berr = 1U << 8,
    bit_arlo = 1U << 9,
    bit_ovr = 1U << 10,
    bit_pecerr = 1U << 11,
    bit_timeout = 1U << 12,
    bit_alert = 1U << 13,
};
/* ADDR is a target-mode condition and never part of a controller transfer here, so it does not
 * count as "this entry carried a flag". */
inline constexpr uint32_t kAnyEventFlag{bit_txis | bit_rxne | bit_nackf | bit_stopf | bit_tc |
                                        bit_tcr | bit_berr | bit_arlo | bit_ovr | bit_pecerr |
                                        bit_timeout | bit_alert};
/* Everything ERRIE can raise. PEC, SMBus timeout and SMBAlert are not configured on this bus, which
 * is a reason to watch for them rather than to assume they cannot appear: an error source nobody
 * clears is the shortest path to an interrupt that re-enters for ever. */
inline constexpr uint32_t kAnyErrorFlag{bit_berr | bit_arlo | bit_ovr | bit_pecerr | bit_timeout |
                                        bit_alert};

/* What one event-interrupt entry means. */
enum entry : uint32_t {
    /* current.len != 0 and neither RXNE nor TXIS: the driver would advance current.buf and
     * current.len for a byte that was never transferred. Measured at 0 -- not how this starts. */
    entry_advance_no_data = 1U << 0,
    /* RXNE with current.len == 0. Ordinary: the healthy background, about one per read. */
    entry_rxne_with_len0 = 1U << 1,
    entry_txis_with_len0 = 1U << 2,
    /* Nothing was set by the time the ISR read the register. */
    entry_no_flag = 1U << 3,
    /* THE FAULT. RXNE with current.len == 0 and no completion flag to act on: the ISR will not read
     * RXDR, will not issue a STOP and will not mask the interrupt, so it re-enters until the board
     * is reset. Distinguishing this from the harmless case above is the whole point of the
     * counter -- the aggregate rxne_with_len0 cannot, because both look the same in it. */
    entry_orphan_rxne = 1U << 4,
};

/* Pure, so the host test and the interrupt share one definition of the fault being looked for. */
/* What the ISR can still act on when the buffer is already full: a completion or an abort. An RXNE
 * arriving with none of these is the entry nothing will consume. */
inline constexpr uint32_t kServiceable{bit_tc | bit_tcr | bit_stopf | bit_nackf};

inline constexpr uint32_t classify_event(uint32_t isr, uint32_t sw_len)
{
    uint32_t bits{0};
    if (sw_len != 0U && (isr & (bit_rxne | bit_txis)) == 0U)
        bits |= entry_advance_no_data;
    if (sw_len == 0U && (isr & bit_rxne) != 0U) {
        bits |= entry_rxne_with_len0;
        if ((isr & kServiceable) == 0U)
            bits |= entry_orphan_rxne;
    }
    if (sw_len == 0U && (isr & bit_txis) != 0U)
        bits |= entry_txis_with_len0;
    if ((isr & kAnyEventFlag) == 0U)
        bits |= entry_no_flag;
    return bits;
}

/* One second of bus behaviour. Deltas rather than totals, because the question is a rate. */
struct window {
    uint32_t ms;
    uint32_t event_delta;
    uint32_t error_delta;
    uint32_t isr;    // I2C_ISR at the sample
    uint32_t cr1;    // I2C_CR1: which interrupts are still enabled
    uint32_t cr2;    // I2C_CR2: NBYTES = (cr2 >> 16) & 0xff, RELOAD = bit 24
    uint32_t nvic;   // bit 0/1/2 = event IRQ enabled/pending/active, bit 4/5/6 = error IRQ
    uint32_t sw_len; // the driver's current.len at the last ISR entry
};

/* Every field is a 32-bit word so `devmem` can read it back. Offsets are part of the reading
 * procedure. */
struct record {
    uint32_t magic;                // 0x000
    uint32_t version;              // 0x004
    uint32_t event_entries;        // 0x008 i2c2 event ISR entries since boot
    uint32_t error_entries;        // 0x00c i2c2 error ISR entries since boot
    uint32_t ev_flag[kEventFlags]; // 0x010 per-flag entry counts, one entry may count in several
    uint32_t ev_none;              // 0x038 entry with none of the flags above set
    uint32_t er_flag[kErrorFlags]; // 0x03c
    uint32_t er_none;              // 0x054 an error interrupt raising no error flag we know of
    uint32_t rxne_with_len0;       // 0x058 RXNE with len already 0, whatever else is set. ORDINARY:
                                   //       about one per read on a healthy board. The background
                                   //       rate that orphan_rxne (+0x0a4) is read against
    uint32_t rxne_with_len0_ms;    // 0x05c when that was first seen (exact: a real clock read)
    uint32_t txis_with_len0;       // 0x060
    uint32_t txis_with_len0_ms;    // 0x064 (exact)
    uint32_t advance_no_data;      // 0x068 len != 0, no RXNE and no TXIS: measured at 0
    uint32_t advance_no_data_ms;   // 0x06c (exact)
    uint32_t advance_no_data_isr;  // 0x070 the flags that entry did carry
    uint32_t last_isr;             // 0x074 the last event entry, whatever it was
    uint32_t last_cr1;             // 0x078
    uint32_t last_cr2;             // 0x07c
    uint32_t last_sw_len;          // 0x080
    uint32_t last_sw_buf;          // 0x084 the driver's current.buf, to see progress
    uint32_t last_entry_ms;        // 0x088 coarse: the last 1 s sample's clock, not a clock read
    uint32_t last_err_isr;         // 0x08c the same for the last ERROR interrupt, kept apart so a
    uint32_t last_err_cr1;         // 0x090 storm of event entries cannot overwrite the one error
    uint32_t last_err_cr2;         // 0x094 that may have started it
    uint32_t last_err_sw_len;      // 0x098
    uint32_t last_err_sw_buf;      // 0x09c
    uint32_t last_err_ms;          // 0x0a0 coarse
    /* THE FAULT INDICATOR: RXNE, len 0, and no TC/TCR/STOPF/NACKF to act on. Expected to stay 0 on
     * a healthy run however large rxne_with_len0 grows. */
    uint32_t orphan_rxne;          // 0x0a4
    uint32_t orphan_rxne_ms;       // 0x0a8 (exact: the first one)
    uint32_t orphan_rxne_isr;      // 0x0ac
    uint32_t orphan_rxne_cr2;      // 0x0b0 NBYTES and RELOAD as the wedged entry saw them
    /* The port layer around each ULD transaction. Written before the call and after it returns, so
     * a stuck transfer leaves begin > end with everything about it still readable. These are copies:
     * the 1 s sampler must never dereference the driver's current.msg, which may already be gone. */
    uint32_t port_begin;           // 0x0b4
    uint32_t port_end;             // 0x0b8
    uint32_t port_addr8;           // 0x0bc the ULD platform's 8-bit wire address: which sensor this
                                   //       was, as the port layer knows it
    uint32_t port_addr;            // 0x0c0 7-bit address
    uint32_t port_is_read;         // 0x0c4
    uint32_t port_reg;             // 0x0c8 16-bit register index of this chunk
    uint32_t port_total_len;       // 0x0cc the whole ULD request
    uint32_t port_chunk_index;     // 0x0d0 which chunk of it
    uint32_t port_chunk_off;       // 0x0d4
    uint32_t port_chunk_len;       // 0x0d8 > 255: the controller splits it again into 255 + rest
    /* Where this chunk's payload starts. last_sw_buf - port_buf_base is how far the driver got
     * INSIDE the chunk, which is the only way to tell the 255-byte segment from the 73-byte one --
     * that split happens inside the controller driver and the port layer never sees it. */
    uint32_t port_buf_base;        // 0x0dc
    uint32_t port_begin_ms;        // 0x0e0 exact: the port layer runs in a thread
    uint32_t port_end_ms;          // 0x0e4 exact
    uint32_t port_last_rc;         // 0x0e8
    /* TIMINGR is set when the bus is configured, but a bench command can change the bitrate at
     * runtime, so both the boot value and the latest one are kept. */
    uint32_t timingr_boot;         // 0x0ec
    uint32_t timingr_last;         // 0x0f0
    /* The epoch the interrupt copies instead of reading the clock. It stops advancing exactly when
     * the 1 s callback stops, which is itself worth seeing. */
    uint32_t coarse_ms;            // 0x0f4
    uint32_t reserved0[2];         // 0x0f8
    window pre_arm;                // 0x100 the last window before `arm`: the healthy L4-only baseline
    window post_arm[kPostArm];     // 0x120 the first 8 windows after `arm`: healthy WITH the L7
    uint32_t ring_next;            // 0x220
    uint32_t windows_sampled;      // 0x224
    uint32_t reserved1[2];         // 0x228
    window ring[kWindows];         // 0x230 rolling: the seconds around the stall
    uint32_t magic_end;            // 0x630
};

int init_record_at_boot();

/* Called once a second from the same timer callback that feeds the watchdog, so the windows keep
 * being written while every thread is starved. ISR context. */
void sample(uint32_t now_ms, bool armed);

/* `tofdiag i2c`: the same block, live, for the pre-arm and post-arm checks. */
int shell_status(const struct shell *sh);

/* The ULD porting layer calls these around one I2C transaction. */
void port_begin(uint32_t addr8, uint16_t addr7, bool is_read, uint16_t reg_index,
                uint32_t total_len, uint32_t chunk_index, uint32_t chunk_off, uint32_t chunk_len,
                uintptr_t buf_base);
void port_end(int rc);

} // namespace lexxhard::tof_diag_i2c

/* The driver hooks. C linkage: they are called from the patched Zephyr I2C driver, which only has
 * them when CONFIG_I2C_STM32_LEXX_ISR_FORENSICS is on, and that is set by the diagnostic overlay
 * alone. See patches/zephyr/0003-i2c-stm32-v2-isr-forensics.patch. */
extern "C" {
struct device;
void lexx_i2c_forensics_event(const struct device *dev, uint32_t isr, uint32_t cr1, uint32_t cr2,
                              uint32_t sw_len, const uint8_t *sw_buf);
void lexx_i2c_forensics_error(const struct device *dev, uint32_t isr, uint32_t cr1, uint32_t cr2,
                              uint32_t sw_len, const uint8_t *sw_buf);

/* Called from the ULD porting layer, which is C, around each I2C transaction it issues. */
void lexx_tof_port_begin(uint32_t addr8, uint16_t addr7, int is_read, uint16_t reg_index,
                         uint32_t total_len, uint32_t chunk_index, uint32_t chunk_off,
                         uint32_t chunk_len, const uint8_t *buf_base);
void lexx_tof_port_end(int rc);
}

#endif // TOF_DIAG_HANG
