/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * See tof_watchdog_feeder.hpp. THIS FILE CONTAINS THE ONLY wdt_feed() CALL IN THE IMAGE.
 */

#include "tof_watchdog_feeder.hpp"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "tof_progress.hpp"
#include "tof_task_watchdog.hpp"
#include "tof_watchdog_tombstone.hpp"

LOG_MODULE_REGISTER(tof_wdt, CONFIG_LOG_DEFAULT_LEVEL);

namespace lexxhard::tof_watchdog_feeder {

namespace {

namespace wd = tof_task_watchdog;
namespace tomb = tof_watchdog_tombstone;

/* Twenty feeds inside one 10,000 ms watchdog window. Short enough that losing several in a row is
 * still survivable, long enough that the feeder is asleep for essentially all of it. */
constexpr int32_t kFeedPeriodMs{500};

/* A fifth of the watchdog window. Long enough to cover a first feed on a loaded boot, short enough
 * that expiring here leaves time for the log line to be read before the reset it implies. */
constexpr int32_t kFirstFeedWaitMs{2000};

/* Above the acquisition thread (7 in the devicetree) so a loaded but healthy board cannot starve
 * the feeder into resetting itself, and preemptible and mostly asleep so it cannot be why a watched
 * thread does not run. */
constexpr int kPriority{5};
constexpr size_t kStackSize{1024};

K_THREAD_STACK_DEFINE(stack_, kStackSize);
k_thread thread_{};

K_SEM_DEFINE(released_, 0, 1);
K_SEM_DEFINE(first_feed_, 0, 1);

const struct device *wdt_{nullptr};
int channel_{0};

atomic_t baseline_point_{};
atomic_t l7_expected_{};
atomic_t long_operation_{};
atomic_t long_began_ms_{};
atomic_t feeds_{};
atomic_t last_fed_ms_{};

/* Consecutive refusals from the driver before the feeder gives up and records why. Three at the
 * 500 ms period is 1.5 s -- long enough that a transient does not end the boot, short enough that
 * the tombstone is committed well inside the 10 s window the refusals are burning. */
constexpr int kFeedFailureLimit{3};

/* Owned by the feeder thread and by nothing else, which is what keeps the decision single-threaded
 * while its inputs arrive from five others. */
wd::state state_{};
wd::bounds bounds_{};

uint32_t now_ms()
{
    return k_uptime_get_32();
}

/* WHERE THE TOMBSTONE GOES, and why this is not simply the constant.
 *
 * On the board the reserved region exists and the record survives a reset. In a host suite it does
 * not: 0x2001FF00 is an unmapped address in a Linux process and writing to it would end the test
 * run rather than fail it. The devicetree node is the honest discriminator -- an image with no
 * reserved region has nowhere durable to put a record, so it uses ordinary RAM and the record does
 * not outlive the boot, which is exactly what a host test is. */
#if DT_NODE_EXISTS(DT_NODELABEL(forensics_dtcm))
volatile uint32_t *tombstone_region()
{
    return reinterpret_cast<volatile uint32_t *>(tomb::kAddress);
}
#else
uint32_t host_region_[tomb::kSize / sizeof(uint32_t)]{};
volatile uint32_t *tombstone_region()
{
    return host_region_;
}
#endif

void build_identity(uint32_t out[4])
{
#if defined(VERSION)
#define TOF_WDT_STR2(x) #x
#define TOF_WDT_STR(x) TOF_WDT_STR2(x)
    const char *const v{TOF_WDT_STR(VERSION)};
#else
    const char *const v{"unversioned"};
#endif
    char buf[16]{};
    for (size_t i{0}; i < sizeof(buf) && v[i] != '\0'; ++i)
        buf[i] = v[i];
    memcpy(out, buf, sizeof(buf));
}

/* Called once, on the transition this whole subsystem exists to produce. */
void commit_tombstone(const wd::input &in, uint32_t reason, int feed_rc)
{
    tomb::record r{};
    build_identity(r.build_id);
    r.phase = static_cast<uint32_t>(state_.current);
    r.reason = reason;
    r.stopped_ms = in.now_ms;
    /* The last SUCCESSFUL feed. The gap between it and the stop is what says whether the board went
     * down on the first refusal or limped for a while first. */
    r.last_fed_ms = static_cast<uint32_t>(atomic_get(&last_fed_ms_));
    r.feed_rc = feed_rc;
    r.acq_begun = in.acquisition.begun;      r.acq_ended = in.acquisition.ended;
    r.send_acq_begun = in.send_acq.begun;    r.send_acq_ended = in.send_acq.ended;
    r.send_workq_begun = in.send_workq.begun;r.send_workq_ended = in.send_workq.ended;
    r.health_begun = in.health.begun;        r.health_ended = in.health.ended;
    r.l7_begun = in.l7.begun;                r.l7_ended = in.l7.ended;
    r.zcan_loops = in.zcan_loops;
    r.long_active = in.long_operation ? 1U : 0U;
    r.long_began_ms = in.long_operation_began_ms;
    r.boot_seq = 0;

    tomb::write_once(tombstone_region(), r);
}

void feeder(void *, void *, void *)
{
    /* Nothing is fed, and nothing is judged, until the watchdog exists. */
    k_sem_take(&released_, K_FOREVER);

    bool stopped{false};
    int consecutive_failures{0};
    for (;;) {
        if (!stopped) {
            const tof_progress::snapshot p{tof_progress::read()};

            wd::input in{};
            in.now_ms = now_ms();
            in.baseline_point = atomic_get(&baseline_point_) != 0;
            in.l7_expected = atomic_get(&l7_expected_) != 0;
            in.acquisition = {p.at[static_cast<size_t>(tof_progress::activity::acquisition)].begun,
                              p.at[static_cast<size_t>(tof_progress::activity::acquisition)].ended};
            in.send_acq = {p.at[static_cast<size_t>(tof_progress::activity::send_acq)].begun,
                           p.at[static_cast<size_t>(tof_progress::activity::send_acq)].ended};
            in.send_workq = {p.at[static_cast<size_t>(tof_progress::activity::send_workq)].begun,
                             p.at[static_cast<size_t>(tof_progress::activity::send_workq)].ended};
            in.health = {p.at[static_cast<size_t>(tof_progress::activity::health)].begun,
                         p.at[static_cast<size_t>(tof_progress::activity::health)].ended};
            in.l7 = {p.at[static_cast<size_t>(tof_progress::activity::l7)].begun,
                     p.at[static_cast<size_t>(tof_progress::activity::l7)].ended};
            in.zcan_loops = p.zcan_loops;
            in.long_operation = atomic_get(&long_operation_) != 0;
            in.long_operation_began_ms = static_cast<uint32_t>(atomic_get(&long_began_ms_));

            if (wd::feed_allowed(state_, bounds_, in)) {
                /* THE ONLY wdt_feed() IN THE IMAGE. */
                if (const int rc{wdt_feed(wdt_, channel_)}; rc == 0) {
                    /* After the call, not the sample taken before it. The inputs were read at the
                     * top of this iteration and the feed is the last thing in it; on a loaded board
                     * those are not the same instant, and the number this records is read later to
                     * decide how long the board went unfed. */
                    atomic_set(&last_fed_ms_, static_cast<atomic_val_t>(now_ms()));
                    consecutive_failures = 0;
                    if (atomic_inc(&feeds_) == 0)
                        k_sem_give(&first_feed_);
                } else {
                    /* A REFUSED FEED ENDS IN THE SAME RESET AS A WITHHELD ONE, and used to leave
                     * nothing behind but a log line the reset erased. After a few in a row the
                     * cause is recorded and the feeder stops trying, so the tombstone is committed
                     * inside the window the refusals are burning rather than after it closes. */
                    LOG_ERR("wdt_feed refused by the driver (%d)", rc);
                    if (++consecutive_failures >= kFeedFailureLimit) {
                        /* BOTH, and the phase is not optional: the tombstone records it and
                         * current() reports `withheld` from it, so setting only the reason left a
                         * record that said feed_api_failed while still claiming the feeder was
                         * running. */
                        state_.why = wd::feed_api_failed;
                        state_.current = wd::phase::stopped;
                        commit_tombstone(in, wd::feed_api_failed, rc);
                        LOG_ERR("giving up after %d refused feeds; this boot will reset",
                                consecutive_failures);
                        stopped = true;
                    }
                }
            } else {
                /* Committed BEFORE the feed is withheld. The other order leaves a reset whose cause
                 * was still being written when the board went down. */
                commit_tombstone(in, state_.why, 0);
                LOG_ERR("watchdog feed withheld: reason 0x%08x after %u feeds; this boot will reset",
                        state_.why, static_cast<unsigned>(atomic_get(&feeds_)));
                stopped = true;
            }
        }
        k_msleep(kFeedPeriodMs);
    }
}

}  // namespace

int start()
{
    const k_tid_t id{k_thread_create(&thread_, stack_, K_THREAD_STACK_SIZEOF(stack_), feeder,
                                     nullptr, nullptr, nullptr, kPriority, 0, K_NO_WAIT)};
    if (id == nullptr)
        return -EAGAIN;
    k_thread_name_set(id, "tof_wdt_feed");
    return 0;
}

void release(const struct device *wdt, int channel)
{
    wdt_ = wdt;
    channel_ = channel;
    k_sem_give(&released_);
}

int wait_first_feed(void)
{
    return k_sem_take(&first_feed_, K_MSEC(kFirstFeedWaitMs)) == 0 ? 0 : -ETIMEDOUT;
}

void set_baseline_point(bool ready)
{
    atomic_set(&baseline_point_, ready ? 1 : 0);
}

void set_l7_expected(bool expected)
{
    atomic_set(&l7_expected_, expected ? 1 : 0);
}

void long_operation_begin()
{
    atomic_set(&long_began_ms_, static_cast<atomic_val_t>(now_ms()));
    atomic_set(&long_operation_, 1);
}

void long_operation_end()
{
    atomic_set(&long_operation_, 0);
}

tomb::status read_record(tomb::record &out)
{
    return tomb::read(tombstone_region(), out);
}

void report_previous_stop()
{
    tomb::record r{};
    const tomb::status st{read_record(r)};
    if (st != tomb::status::valid) {
        /* Not a warning. A board that has never tripped the watchdog reads exactly like this, and
         * so does one whose battery was pulled, and neither is news. */
        LOG_INF("no retained watchdog record (%s)", tomb::status_name(st));
        return;
    }
    /* RETAINED, not necessarily the previous boot. Nothing clears this record after reporting it,
     * so it survives any number of clean boots until something overwrites it. Saying "previous
     * boot" would invite reading an old fault as a new one. */
    LOG_ERR("RETAINED WATCHDOG STOP RECORD: reason 0x%08x at %u ms", r.reason,
            static_cast<unsigned>(r.stopped_ms));
    LOG_ERR("  acq %u/%u  send_acq %u/%u  send_workq %u/%u", static_cast<unsigned>(r.acq_begun),
            static_cast<unsigned>(r.acq_ended), static_cast<unsigned>(r.send_acq_begun),
            static_cast<unsigned>(r.send_acq_ended), static_cast<unsigned>(r.send_workq_begun),
            static_cast<unsigned>(r.send_workq_ended));
    LOG_ERR("  last fed at %u ms, feed rc %d", static_cast<unsigned>(r.last_fed_ms),
            static_cast<int>(r.feed_rc));
    LOG_ERR("  health %u/%u  l7 %u/%u  zcan %u  long %u since %u ms",
            static_cast<unsigned>(r.health_begun), static_cast<unsigned>(r.health_ended),
            static_cast<unsigned>(r.l7_begun), static_cast<unsigned>(r.l7_ended),
            static_cast<unsigned>(r.zcan_loops), static_cast<unsigned>(r.long_active),
            static_cast<unsigned>(r.long_began_ms));
}

status current()
{
    status s{};
    s.phase = static_cast<uint32_t>(state_.current);
    s.why = state_.why;
    s.feeds = static_cast<uint32_t>(atomic_get(&feeds_));
    s.withheld = state_.current == wd::phase::stopped;
    return s;
}

}  // namespace lexxhard::tof_watchdog_feeder
