/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * See tof_watchdog_feeder.hpp. THIS FILE CONTAINS THE ONLY wdt_feed() CALL IN THE IMAGE.
 */

#include "tof_watchdog_feeder.hpp"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#if defined(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif
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

/* COOPERATIVE, AT THE TOP, AND THAT IS A CHOICE WITH A STATED LIMIT.
 *
 * An earlier value of 5 was justified against the acquisition thread (7) alone, and that was the
 * wrong comparison: in Zephyr a smaller number is HIGHER priority, and main.cpp starts thirteen
 * threads above 5 -- led, pgv and shutter at 1, actuator, actuator_service, adc, imu, uss, gpio and
 * tug at 2, bmu, can, board and runaway at 4 -- with zcan_main equal at 5. Any one of them spinning
 * starves the feeder, commit_tombstone() is never called, and the IWDG resets with no record: once
 * the wiring removes the unconditional timer feed, that is exactly the "first trip leaves nothing"
 * this layer exists to end.
 *
 * K_PRIO_COOP(0) is -CONFIG_NUM_COOP_PRIORITIES, which is -16 in this build: COOPERATIVE, not
 * preemptible. A previous comment here said "the highest preemptible priority" while using
 * K_HIGHEST_APPLICATION_THREAD_PRIO, which is the same negative value -- the description was simply
 * wrong. Cooperative is wanted because every application thread in main.cpp is preemptible, so none
 * of them can hold the CPU against this one, and this one yields every pass by sleeping, so it
 * cannot hold the CPU either.
 *
 * WHAT COOPERATIVE DOES NOT BUY, stated carefully because two earlier versions of this comment got
 * it wrong in two different directions. Zephyr's rule is that a cooperative thread, once it becomes
 * the current thread, REMAINS the current thread until it does something that makes itself unready
 * (doc/kernel/services/threads/index.rst). Priority decides who is selected among threads that are
 * READY; it does not let anyone interrupt a cooperative thread that is already running. So a
 * cooperative thread that spins without yielding starves this feeder whatever its priority is --
 * ABOVE OR BELOW. The system workqueue sits at CONFIG_SYSTEM_WORKQUEUE_PRIORITY = -1, numerically
 * below -16, and a work item there that loops without yielding starves the feeder anyway; the
 * assertions below cannot prevent that and are not claimed to. What they do is narrower and still
 * worth having: they keep the feeder the first cooperative thread SELECTED when several are ready,
 * and they fail the build if someone moves either priority, which is the kind of change that should
 * be noticed deliberately rather than measured later in resets.
 *
 * Nor does any priority cover a sustained interrupt storm or a region with interrupts locked,
 * because priority does not order a thread against an ISR. So the honest claim is narrow: this
 * makes a record LIKELY, not guaranteed, and the reset cause reported at boot is what remains when
 * it is not. */
constexpr int kPriority{K_PRIO_COOP(0)};

/* The top cooperative slot: nothing can be created above it without raising NUM_COOP_PRIORITIES,
 * which would move this one and should be noticed here. */
BUILD_ASSERT(kPriority == -CONFIG_NUM_COOP_PRIORITIES,
             "the feeder must hold the highest cooperative priority");
/* A smaller number is higher priority, so the workqueue must stay numerically greater. This does
 * NOT prevent a workqueue item from starving the feeder -- a running cooperative thread cannot be
 * preempted at all, see above -- it only keeps the feeder ahead of it in the selection order and
 * makes a change to either priority a deliberate one. */
BUILD_ASSERT(CONFIG_SYSTEM_WORKQUEUE_PRIORITY > kPriority,
             "the system workqueue would outrank the watchdog feeder");
constexpr size_t kStackSize{1024};

K_THREAD_STACK_DEFINE(stack_, kStackSize);
k_thread thread_{};

K_SEM_DEFINE(released_, 0, 1);
K_SEM_DEFINE(first_feed_, 0, 1);

const struct device *wdt_{nullptr};
int channel_{0};

atomic_t baseline_point_{};
atomic_t l7_expected_{};

/* DEFAULTS TO EXPECTED, and the default is the decision rather than an accident.
 *
 * If this defaulted to "not expected" a wiring that never calls the setter would leave the
 * production feeder permanently suspending the cycle reasons and send_acq -- a watchdog that is
 * silently blind to most of what it watches, which is the failure this layer exists to prevent and
 * the one nobody would ever find. Defaulting to expected makes the same omission produce a false
 * reset during the first commissioning pass: loud, reproducible on a bench, and found immediately.
 *
 * An earlier version of this file added the input and did not wire it at all, which is exactly the
 * first failure. */
atomic_t acquisition_expected_{ATOMIC_INIT(1)};
atomic_t long_operation_{};
atomic_t long_began_ms_{};
atomic_t feeds_{};
atomic_t last_fed_ms_{};

/* Consecutive refusals from the driver before the feeder gives up and records why. Three at the
 * 500 ms period is 1.5 s -- long enough that a transient does not end the boot, short enough that
 * the tombstone is committed well inside the 10 s window the refusals are burning. */
constexpr int kFeedFailureLimit{3};

/* Owned by the feeder thread and by nothing else, which is what keeps the decision single-threaded
 * while its inputs arrive from five others. NOTHING OUTSIDE THE FEEDER MAY READ THESE: current()
 * is called from the shell and from initialisation, and reading plain fields that the feeder is in
 * the middle of writing is a race whose most likely symptom is the worst one -- a phase and a
 * reason from either side of the same transition, so a reader is told the board stopped and shown
 * nothing that says why. */
wd::state state_{};
wd::bounds bounds_{};

/* THE DECISION, PUBLISHED IN ONE WORD, which is the point: phase and reason move together and a
 * reader takes both or neither. Two atomics would be no better than two plain fields here -- each
 * read would be clean and the pair still torn. The reason bits are a mask and the phase is a small
 * enum, so both fit in one 32-bit word with room to spare. */
constexpr unsigned kPhaseShift{24};
static_assert(static_cast<uint32_t>(wd::feed_api_failed) < (1U << kPhaseShift),
              "the reason mask has grown into the phase field");
atomic_t published_{};

uint32_t pack_state(wd::phase ph, uint32_t why)
{
    return (static_cast<uint32_t>(ph) << kPhaseShift) | (why & ((1U << kPhaseShift) - 1U));
}

/* Called by the feeder, and only by the feeder, after every decision. */
void publish_state()
{
    atomic_set(&published_, static_cast<atomic_val_t>(pack_state(state_.current, state_.why)));
}

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
/* Never the board: tof_watchdog_tombstone.cpp refuses to compile for the SCB without the
 * reservation, so reaching this branch means a host build, where a record that does not outlive the
 * process is exactly right. */
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
            in.acquisition_expected = atomic_get(&acquisition_expected_) != 0;
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

            const bool allowed{wd::feed_allowed(state_, bounds_, in)};
            /* Before the feed rather than after it. feed_allowed() has already moved the phase, and
             * a reader that arrives between the decision and the feed must not be shown the
             * previous one. */
            publish_state();
            if (allowed) {
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
                        publish_state();
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

void set_acquisition_expected(bool expected)
{
    atomic_set(&acquisition_expected_, expected ? 1 : 0);
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

namespace {

/* WHY THIS BOOT HAPPENED, so a watchdog reset with no record can be told from a clean start.
 *
 * It is the question the record cannot answer when the record is missing or stale: "no retained
 * record" reads identically on a board that has never tripped and on one that tripped and lost its
 * record. The reset cause distinguishes them, and it comes from the hardware rather than from
 * anything this subsystem wrote.
 *
 * Reported rather than acted on. Nothing here decides anything from it; a reset cause that drove
 * behaviour would be a second, weaker copy of the record. */
void report_reset_cause()
{
#if defined(CONFIG_HWINFO)
    uint32_t cause{0};
    if (hwinfo_get_reset_cause(&cause) != 0) {
        LOG_WRN("  reset cause unavailable");
        return;
    }
    if ((cause & RESET_WATCHDOG) != 0)
        LOG_ERR("  reset cause 0x%08x INCLUDES THE WATCHDOG", static_cast<unsigned>(cause));
    else
        LOG_INF("  reset cause 0x%08x (not the watchdog)", static_cast<unsigned>(cause));
#else
    /* Said once rather than left silent: a reader looking for the cause should learn that this
     * image cannot tell them, not that the cause was benign. */
    LOG_INF("  reset cause not available (CONFIG_HWINFO is off)");
#endif
}

}  // namespace

void report_previous_stop()
{
    tomb::record r{};
    const tomb::status st{read_record(r)};

    if (st == tomb::status::not_committed) {
        /* THE ONLY ONE THAT IS NOT NEWS. No commit magic means either nothing was ever written or a
         * reset landed mid-write: a board that has never tripped the watchdog reads exactly like
         * this, and so does one whose battery was pulled. */
        LOG_INF("no retained watchdog record (%s)", tomb::status_name(st));
        /* REPORTED HERE TOO, and this is the branch that needs it most: "no retained record" reads
         * identically on a board that has never tripped and on one that tripped and lost its
         * record. The cause is the only thing that tells those apart, and an earlier version
         * returned before reaching it. */
        report_reset_cause();
        return;
    }
    if (st != tomb::status::valid) {
        /* WRITTEN AND NOW UNREADABLE, WHICH IS A DIFFERENT THING. wrong_version, wrong_size,
         * bad_end_magic and bad_checksum are only reachable with the commit magic PRESENT, so they
         * all mean "a stop was recorded and cannot be read back": the overwrite the DTCM
         * reservation exists to prevent, or an MCUboot revert leaving an old image reading a newer
         * record. They used to go out as the same LOG_INF as not_committed, under a comment about
         * boards that never tripped -- which describes only that one case and buried these. */
        LOG_ERR("RETAINED WATCHDOG RECORD IS UNREADABLE (%s): a stop WAS recorded and cannot be "
                "read back", tomb::status_name(st));
        report_reset_cause();
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
    report_reset_cause();
    LOG_ERR("  health %u/%u  l7 %u/%u  zcan %u  long %u since %u ms",
            static_cast<unsigned>(r.health_begun), static_cast<unsigned>(r.health_ended),
            static_cast<unsigned>(r.l7_begun), static_cast<unsigned>(r.l7_ended),
            static_cast<unsigned>(r.zcan_loops), static_cast<unsigned>(r.long_active),
            static_cast<unsigned>(r.long_began_ms));
}

status current()
{
    /* ONE READ, so phase and reason cannot come from either side of a transition. `feeds` is a
     * separate counter and deliberately not part of the word: it moves on its own schedule, it
     * carries no consistency relationship with the decision, and a feed that lands between these
     * two reads is not a contradiction. */
    const uint32_t packed{static_cast<uint32_t>(atomic_get(&published_))};

    status s{};
    s.phase = packed >> kPhaseShift;
    s.why = packed & ((1U << kPhaseShift) - 1U);
    s.feeds = static_cast<uint32_t>(atomic_get(&feeds_));
    s.withheld = s.phase == static_cast<uint32_t>(wd::phase::stopped);
    return s;
}

}  // namespace lexxhard::tof_watchdog_feeder
