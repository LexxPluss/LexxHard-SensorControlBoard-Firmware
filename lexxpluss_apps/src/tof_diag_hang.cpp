/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_diag_hang.hpp"

#if defined(TOF_DIAG_HANG)

#if TOF_DIAG_HANG != 1 && TOF_DIAG_HANG != 2
#error "TOF_DIAG_HANG must be 1 (real L7, no grid TX) or 2 (static grid, no L7)"
#endif
#if !defined(ENABLE_TOF_L7_ULD) || !defined(TOF_DEV_NO_AUTO_CONFIRM)
#error "TOF_DIAG_HANG needs ENABLE_TOF_L7_ULD and TOF_DEV_NO_AUTO_CONFIRM"
#endif

#include <stddef.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/fatal.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#include "tof_acquisition.hpp"
#include "tof_l7_sensor.hpp"

namespace lexxhard::tof_diag {

namespace {

static_assert(sizeof(record) == 0x1c0, "the record layout is part of the reading procedure");
static_assert(offsetof(record, wdt_withheld) == 0x8c);
static_assert(offsetof(record, fatal_reason) == 0x98);
static_assert(offsetof(record, ring) == 0xbc);
static_assert(offsetof(record, magic_end) == 0x1bc);
/* Inside DTCM, and the record does not run off its end. */
static_assert(kRecordAddress >= DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm)));
static_assert(kRecordAddress + sizeof(record) <=
              DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm)) + DT_REG_SIZE(DT_CHOSEN(zephyr_dtcm)));

volatile record &rec()
{
    return *reinterpret_cast<volatile record *>(kRecordAddress);
}

/* Counters written by more than one thread go through these; single-writer fields are plain stores. */
void inc(volatile uint32_t &field)
{
    atomic_inc(reinterpret_cast<atomic_t *>(const_cast<uint32_t *>(&field)));
}

uint32_t now_ms()
{
    return k_uptime_get_32();
}

void ring(event code, uint32_t arg)
{
    volatile record &r{rec()};
    const uint32_t slot{static_cast<uint32_t>(atomic_inc(reinterpret_cast<atomic_t *>(
                            const_cast<uint32_t *>(&r.ring_next)))) %
                        kRing};
    r.ring[slot].ms = now_ms();
    r.ring[slot].event = (static_cast<uint32_t>(code) << 24) | (arg & 0xFFFFFFU);
}

watch_state watch_{};
bool withheld_latched_{false};

} // namespace

int init_record_at_boot()
{
    volatile record &r{rec()};
    /* Fresh every boot of THIS image. What the previous boot left is read from E3, which is what
     * runs after the reset -- this image never sees its own previous record. */
    for (size_t i{0}; i < sizeof(record) / sizeof(uint32_t); ++i)
        reinterpret_cast<volatile uint32_t *>(kRecordAddress)[i] = 0;
    r.mode = TOF_DIAG_HANG;
    r.fatal_reason = 0xFFFFFFFFU;
    r.magic_end = kMagicEnd;
    r.magic = kMagic;
    ring(ev_boot, TOF_DIAG_HANG);
    return 0;
}

bool feed_allowed()
{
    volatile record &r{rec()};
    const uint32_t now{now_ms()};

    r.eval_ms = now;
    if (withheld_latched_)
        return false;

    const watch_input in{now,         r.acq_begin,    r.acq_end,    r.send_begin, r.send_end,
                         r.health_begin, r.health_end, r.zcan_loops};
    const uint32_t why{evaluate(watch_, in)};

    if (why != 0) {
        withheld_latched_ = true;
        r.wdt_reason = why;
        r.wdt_withheld_ms = now;
        r.wdt_withheld = 1;
        ring(ev_withheld, why);
        return false;
    }
    r.wdt_feeds = r.wdt_feeds + 1;
    return true;
}

bool armed()
{
    return rec().armed != 0;
}

void arm()
{
    volatile record &r{rec()};
    if (r.armed != 0)
        return;
    r.armed_ms = now_ms();
    r.armed = 1;
    ring(ev_armed, TOF_DIAG_HANG);
}

void cycle_begin()
{
    volatile record &r{rec()};
    r.acq_begin_ms = now_ms();
    r.acq_begin = r.acq_begin + 1; // acquisition thread only
}

void cycle_end()
{
    volatile record &r{rec()};
    r.acq_end_ms = now_ms();
    r.acq_end = r.acq_end + 1;
}

void send_begin(uint16_t can_id)
{
    volatile record &r{rec()};
    r.send_id = can_id;
    r.send_begin_ms = now_ms();
    inc(r.send_begin); // acquisition thread and the system work queue
}

void send_end(int rc)
{
    volatile record &r{rec()};
    r.send_rc = static_cast<uint32_t>(rc);
    if (rc != 0) {
        inc(r.send_fail);
        ring(ev_send_fail, (r.send_id << 8) | (static_cast<uint32_t>(rc) & 0xFFU));
    }
    inc(r.send_end);
}

void health_begin()
{
    inc(rec().health_begin);
}

void health_end()
{
    inc(rec().health_end);
}

void zcan_loop()
{
    volatile record &r{rec()};
    r.zcan_loops = r.zcan_loops + 1; // zcan_main only
}

void grid_suppressed()
{
    volatile record &r{rec()};
    r.grid_suppressed = r.grid_suppressed + 1;
}

void grid_sent()
{
    volatile record &r{rec()};
    r.grid_sent = r.grid_sent + 1;
}

/* ------------------------------------------------------------------ the grid ops of each mode --- */

namespace {

namespace acq = lexxhard::tof_acq;

/* Which of the two grid objects a void* is. The descriptors hand the same two pointers every time. */
void *devs_[2]{};
uint8_t addr_[2]{};
uint8_t freq_[2]{};
bool opened_[2]{};
uint8_t open_attempts_[2]{};
bool first_fresh_[2]{};
#if TOF_DIAG_HANG == 2
uint32_t last_fresh_ms_[2]{};
#endif

int index_of(void *dev)
{
    for (int i{0}; i < 2; ++i) {
        if (devs_[i] == dev)
            return i;
    }
    for (int i{0}; i < 2; ++i) {
        if (devs_[i] == nullptr) {
            devs_[i] = dev;
            return i;
        }
    }
    return -1;
}

/* open/configure/start are DEFERRED in both modes: at bring-up they only remember what they were
 * asked, touch no bus and succeed, so the source counts as started and the cycle calls read. */
int diag_open(void *dev, uint8_t addr_7bit, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    const int i{index_of(dev)};
    if (i < 0)
        return -EINVAL;
    addr_[i] = addr_7bit;
    opened_[i] = false;
    open_attempts_[i] = 0;
    return 0;
}

int diag_configure(void *dev, uint8_t frequency_hz, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    const int i{index_of(dev)};
    if (i < 0)
        return -EINVAL;
    freq_[i] = frequency_hz;
    return 0;
}

int diag_start(void *, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    return 0;
}

int diag_stop(void *dev, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
#if TOF_DIAG_HANG == 1
    const int i{index_of(dev)};
    if (i >= 0 && opened_[i]) {
        opened_[i] = false;
        return tof_l7::stop(static_cast<tof_l7::sensor *>(dev), st);
    }
#else
    (void)dev;
#endif
    return 0;
}

#if TOF_DIAG_HANG == 1
/* The real L7, opened on the first read after `arm`, inside the cycle and under the chain lock --
 * where start() would have done it. Two attempts per sensor per bring-up, then it stays closed. */
int diag_read(void *dev, void *scratch, tof_l7::sample *out, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    *out = tof_l7::sample{};
    const int i{index_of(dev)};
    if (i < 0)
        return -EINVAL;
    if (!armed())
        return 0;

    volatile record &r{rec()};
    auto *s{static_cast<tof_l7::sensor *>(dev)};

    if (!opened_[i]) {
        if (open_attempts_[i] >= 2)
            return 0;
        ++open_attempts_[i];
        r.l7_open_begin[i] = r.l7_open_begin[i] + 1;
        ring(ev_open_begin, static_cast<uint32_t>(i));
        int rc{tof_l7::open(s, addr_[i], st)};
        if (rc == 0)
            rc = tof_l7::configure(s, freq_[i], st);
        if (rc == 0)
            rc = tof_l7::start(s, st);
        r.l7_open_rc[i] = static_cast<uint32_t>(rc);
        r.l7_open_end[i] = r.l7_open_end[i] + 1;
        ring(ev_open_end, (static_cast<uint32_t>(i) << 16) | (static_cast<uint32_t>(rc) & 0xFFFFU));
        if (rc != 0)
            return rc;
        opened_[i] = true;
    }

    r.l7_read_begin[i] = r.l7_read_begin[i] + 1;
    const int rc{tof_l7::read_once(s, static_cast<tof_l7::scratch *>(scratch), out, st)};
    r.l7_read_rc[i] = static_cast<uint32_t>(rc);
    r.l7_read_end[i] = r.l7_read_end[i] + 1;
    if (rc == 0 && st->sample_present && out->fresh) {
        r.l7_fresh[i] = r.l7_fresh[i] + 1;
        if (!first_fresh_[i]) {
            first_fresh_[i] = true;
            ring(ev_first_fresh, static_cast<uint32_t>(i));
        }
    }
    return rc;
}
#else
/* No L7 bus access at all. After `arm`, a fixed grid every 200 ms per sensor -- the 5 Hz the real
 * sensors produce -- through the real publisher, packer and synchronous CAN send. */
int diag_read(void *dev, void *, tof_l7::sample *out, tof_l7::operation_status *st)
{
    *st = tof_l7::operation_status{};
    *out = tof_l7::sample{};
    const int i{index_of(dev)};
    if (i < 0)
        return -EINVAL;
    if (!armed())
        return 0;

    volatile record &r{rec()};
    const uint32_t now{now_ms()};

    r.l7_read_begin[i] = r.l7_read_begin[i] + 1;
    if (last_fresh_ms_[i] == 0 || now - last_fresh_ms_[i] >= 200) {
        last_fresh_ms_[i] = now;
        out->fresh = true;
        out->silicon_temperature_degc = 30;
        for (size_t z{0}; z < tof_l7::kZoneCount; ++z) {
            out->target_count[z] = 1;
            out->distance_mm[z] = static_cast<uint16_t>(1000 + 10 * z + 500 * i);
            out->target_status[z] = 5; // trusted
        }
        st->sample_present = true;
        r.l7_fresh[i] = r.l7_fresh[i] + 1;
        if (!first_fresh_[i]) {
            first_fresh_[i] = true;
            ring(ev_first_fresh, static_cast<uint32_t>(i));
        }
    }
    r.l7_read_end[i] = r.l7_read_end[i] + 1;
    return 0;
}
#endif

const acq::grid_source_ops kDiagGridOps{diag_open, diag_configure, diag_start, diag_read, diag_stop};

} // namespace

const acq::grid_source_ops &grid_ops()
{
    return kDiagGridOps;
}

/* ------------------------------------------------------------------ shell --- */

namespace {

int cmd_arm(const struct shell *sh, size_t, char **)
{
    arm();
    shell_print(sh, "armed (mode %d): %s", TOF_DIAG_HANG,
                TOF_DIAG_HANG == 1 ? "real L7 opens on the next cycle; grid frames stay off CAN"
                                   : "static grid frames go on CAN from the next cycle");
    return 0;
}

int cmd_status(const struct shell *sh, size_t, char **)
{
    const volatile record &r{rec()};
    shell_print(sh, "mode %u armed %u at %u ms  now %u ms  magic %08x/%08x", r.mode, r.armed,
                r.armed_ms, now_ms(), r.magic, r.magic_end);
    shell_print(sh, "acq   begin %u end %u  (last %u / %u ms)", r.acq_begin, r.acq_end,
                r.acq_begin_ms, r.acq_end_ms);
    shell_print(sh, "send  begin %u end %u fail %u  last id 0x%03x rc %d at %u ms", r.send_begin,
                r.send_end, r.send_fail, r.send_id, static_cast<int>(r.send_rc), r.send_begin_ms);
    shell_print(sh, "grid  sent %u suppressed %u", r.grid_sent, r.grid_suppressed);
    shell_print(sh, "health begin %u end %u   zcan loops %u", r.health_begin, r.health_end,
                r.zcan_loops);
    for (int i{0}; i < 2; ++i)
        shell_print(sh, "l7[%d] open %u/%u rc %d  read %u/%u rc %d fresh %u", i, r.l7_open_begin[i],
                    r.l7_open_end[i], static_cast<int>(r.l7_open_rc[i]), r.l7_read_begin[i],
                    r.l7_read_end[i], static_cast<int>(r.l7_read_rc[i]), r.l7_fresh[i]);
    shell_print(sh, "wdt   feeds %u withheld %u reason 0x%x at %u ms (eval %u ms)", r.wdt_feeds,
                r.wdt_withheld, r.wdt_reason, r.wdt_withheld_ms, r.eval_ms);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tofdiag,
    SHELL_CMD(arm, NULL, "start the risky path of this image", cmd_arm),
    SHELL_CMD(status, NULL, "progress record", cmd_status),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(tofdiag, &sub_tofdiag, "L7 hang-isolation diagnostics (DEV)", NULL);

} // namespace

} // namespace lexxhard::tof_diag

/* Record, then halt exactly as the default handler does; the IWDG resets the halted board. */
void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
    using namespace lexxhard::tof_diag;
    volatile record &r{rec()};

    r.fatal_ms = k_uptime_get_32();
    r.fatal_pc = esf != nullptr ? esf->basic.pc : 0;
    r.fatal_lr = esf != nullptr ? esf->basic.lr : 0;
    const char *name{k_thread_name_get(k_current_get())};
    bool ended{name == nullptr};
    for (size_t i{0}; i < sizeof r.fatal_thread; ++i) {
        if (!ended && (name[i] == '\0' || i + 1 == sizeof r.fatal_thread))
            ended = true;
        r.fatal_thread[i] = ended ? '\0' : name[i];
    }
    r.fatal_reason = reason;
    ring(ev_fatal, reason);

    LOG_PANIC();
    k_fatal_halt(reason);
}

/* Before anything that could mark progress runs. At global scope: SYS_INIT places a section entry. */
static int tof_diag_init_record()
{
    return lexxhard::tof_diag::init_record_at_boot();
}
SYS_INIT(tof_diag_init_record, PRE_KERNEL_1, 0);

#endif // TOF_DIAG_HANG
