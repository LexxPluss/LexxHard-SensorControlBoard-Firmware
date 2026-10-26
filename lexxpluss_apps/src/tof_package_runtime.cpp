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

#include <errno.h>

#include "tof_package_runtime.hpp"

#if defined(ENABLE_TOF_PACKAGE_AUTO)

#if !defined(ENABLE_TOF_L7_ULD)
#error "ENABLE_TOF_PACKAGE_AUTO requires ENABLE_TOF_L7_ULD: the runtime exists to drive the L7."
#endif
#if defined(ENABLE_TOF_PACKAGE_BENCH)
#error "ENABLE_TOF_PACKAGE_AUTO and ENABLE_TOF_PACKAGE_BENCH claim the same device: pick one."
#endif

/* THE IDENTIFIERS COME FROM THE REGISTRY HEADER, not from this file and no longer from a build
 * flag. They were unregistered until 2026-10-26, and while that was true the build had to fail
 * without them: a literal written here would have become the allocation by being the only number
 * anybody could find. Now that tof_can_ids.hpp records the allocation, the build flags would be
 * the duplicate -- so they remain only as an override, for putting a candidate pair on the bus
 * without editing the registry. */
#if !defined(TOF_PACKAGE_AUTO_DATA_ID)
#define TOF_PACKAGE_AUTO_DATA_ID (lexxhard::tof_can_ids::PACKAGE_TOF_DATA_ID)
#endif
#if !defined(TOF_PACKAGE_AUTO_HEALTH_ID)
#define TOF_PACKAGE_AUTO_HEALTH_ID (lexxhard::tof_can_ids::PACKAGE_TOF_HEALTH_ID)
#endif
/* The acquisition rate still has no default, for the reason the identifiers no longer lack one:
 * the sustainable rate for this geometry has not been measured and accepted, so there is nothing
 * for a header to record. The build names it or the build fails. */
#if !defined(TOF_PACKAGE_AUTO_HZ)
#error "ENABLE_TOF_PACKAGE_AUTO needs TOF_PACKAGE_AUTO_HZ"
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "tof_can_ids.hpp"
#include "tof_package_auto.hpp"
#include "tof_package_can.hpp"
#include "tof_package_device.hpp"
#include "tof_package_l7.hpp"
#include "tof_package_stream.hpp"
#include "tof_watchdog_feeder.hpp"

LOG_MODULE_REGISTER(tof_package_runtime);

namespace lexxhard::tof_package_runtime {

k_thread thread;

namespace {

namespace pkg = lexxhard::tof_package;
namespace auto_policy = lexxhard::tof_package_auto;

/* HOW LONG TO LISTEN BEFORE TRANSMITTING. The identifiers are allocated, so this is no longer a
 * question of ownership; traffic on them now means a second transmitter configured for the same
 * stream, or an override pointing at somebody else's number, and either is worth catching before
 * a board starts transmitting by itself at every boot. Silence proves nothing in either
 * direction, as it never did. */
constexpr int32_t kListenWindowMs{1000};

/* HOW OFTEN THE LOOP WAKES. Shorter than one acquisition period on purpose: a poll that found
 * nothing is not an acquisition and publishes nothing, so the cost of polling early is a wakeup
 * and the cost of polling late is latency added to every grid. */
constexpr int32_t kPollIntervalMs{20};

/* Diagnostics have a 500 ms minimum interval, not one per 20 ms poll.
 * A grid is overdue after max(five nominal periods, 500 ms). This tolerance
 * comes from the observed ~195 ms cadence at configured 5 Hz; it is not a
 * sampling-freshness or sustainable-rate guarantee.
 */
constexpr int64_t kNominalPeriodMs{1000 / TOF_PACKAGE_AUTO_HZ};
constexpr int64_t kDiagnosticIntervalMs{500};
constexpr int64_t kGridOverdueMs{5 * kNominalPeriodMs > kDiagnosticIntervalMs ? 5 * kNominalPeriodMs
                                                                             : kDiagnosticIntervalMs};

/* The thread's own state, in .bss: see tof_package_device.hpp on why nothing from this path goes
 * on a stack. */
auto_policy::policy policy_;

/* Whether the transport is wired and the identifiers were silent. False means NOTHING is ever
 * sent: a collision has no diagnostic, because the diagnostic would go out on the identifier that
 * collided. */
bool transport_ready_{false};

/* WHEN A DIAGNOSTIC LAST WENT OUT, and when a grid was last in hand or last began to be expected.
 * Both are only ever read by what_to_send(), which is where every rule about them lives. */
int64_t last_diagnostic_ms_{0};
int64_t last_grid_ms_{0};

/* One place that turns the lifecycle into what the policy is allowed to see. The policy takes the
 * lifecycle's own discardable() answer rather than deriving one from the step, because that layer
 * is the one that knows which blocked steps a live session can be behind. */
auto_policy::observation look()
{
    const pkg::status s{tof_package_device::lifecycle().current()};
    auto_policy::observation o{};
    o.bound = tof_package_device::bound();
    o.current = s.current;
    o.failed_step = s.failed_step;
    o.discardable = tof_package_device::lifecycle().discardable();
    return o;
}

/* Sends what the cadence rule asked for, and nothing on say_nothing. Returns nothing: every
 * outcome is a counter the publisher already keeps, and a caller that branched on them would be
 * making the publication rule's decisions a second time.
 *
 * THE TWO CLOCKS ARE STAMPED HERE because this is where a frame actually goes out. Stamping them
 * at the decision instead would advance them for a pass that sent nothing. */
void send(auto_policy::report what, const tof_package_stream::input &in, int64_t now)
{
    if (what == auto_policy::report::say_nothing || !transport_ready_)
        return;

    tof_package_stream::outcome result{};
    const int rc{tof_package_device::publisher().publish(in, &result)};

    if (what == auto_policy::report::publish_grid)
        last_grid_ms_ = now;
    else
        last_diagnostic_ms_ = now;

    if (rc != 0 || result == tof_package_stream::outcome::grid_abandoned ||
        result == tof_package_stream::outcome::diagnostic_failed)
        LOG_WRN("package tof: publish said %s (%d)", tof_package_stream::outcome_name(result), rc);
}

auto_policy::cadence the_cadence()
{
    auto_policy::cadence c{};
    c.diagnostic_interval_ms = kDiagnosticIntervalMs;
    c.grid_overdue_ms = kGridOverdueMs;
    return c;
}

/* One round on the device, and the restart of the grid wait that a successful one earns.
 *
 * WHY THE WAIT RESTARTS HERE. `last_grid_ms_` means "when a grid was last in hand, or when the
 * runtime last began expecting one". A bring-up takes seconds -- the blob verification alone is
 * most of them -- so without this the first poll after a successful claim would find a grid
 * overdue by the whole duration of the claim and report a stall the instant the sensor started
 * working. A claim that FAILED earns nothing: the silence it leaves behind is real. */
void claim(bool by_takeover)
{
    const int rc{tof_package_device::begin_round(by_takeover)};
    if (rc != 0) {
        LOG_WRN("package tof: %s stopped at %s (%d)", by_takeover ? "takeover" : "bring-up",
                pkg::step_name(tof_package_device::lifecycle().current().failed_step), rc);
        return;
    }
    last_grid_ms_ = k_uptime_get();
}

/* One acquisition pass. What it owes the bus is what_to_send()'s answer, not this function's. */
void acquire()
{
    bool fresh{false};
    const int rc{tof_package_device::lifecycle().read_once(&fresh)};

    auto_policy::pass p{};
    p.read_rc = rc;
    p.fresh = fresh;
    p.now_ms = k_uptime_get();
    p.last_grid_ms = last_grid_ms_;
    p.last_diagnostic_ms = last_diagnostic_ms_;
    const auto_policy::report what{auto_policy::what_to_send(p, the_cadence())};

    tof_package_stream::input in{};
    if (what == auto_policy::report::publish_grid) {
        in.outcome = tof_package_health::acquisition::complete_grid;
        in.distance_mm = tof_package_device::last_sample().distance_mm;
        in.status = tof_package_device::last_sample().target_status;
        in.zone_count = tof_package_stream::kZones;
    } else {
        /* A failed read, or a grid overdue long enough to be a stall. The grid still sitting in
         * the sample may be perfectly good and is deliberately NOT passed: `failed` is what
         * happened to THIS acquisition, and the publication rule is what keeps a held grid off
         * the bus. */
        in.outcome = tof_package_health::acquisition::failed;
    }
    send(what, in, p.now_ms);
}

/* Says the device is not producing, on the diagnostic cadence. `never_started` rather than
 * `failed` whenever nothing has been brought up: the two are different claims about the device,
 * and the availability flag the consumer reads is what distinguishes them.
 *
 * This is not an acquisition, so it asks the same rule with a read that did not happen: a non-zero
 * read_rc is how a pass says it has no grid and no excuse, which is exactly the case here. */
void report(bool ever_claimed)
{
    auto_policy::pass p{};
    p.read_rc = -ENODEV;
    p.now_ms = k_uptime_get();
    p.last_grid_ms = last_grid_ms_;
    p.last_diagnostic_ms = last_diagnostic_ms_;
    const auto_policy::report what{auto_policy::what_to_send(p, the_cadence())};

    tof_package_stream::input in{};
    in.outcome = ever_claimed ? tof_package_health::acquisition::failed
                              : tof_package_health::acquisition::never_started;
    send(what, in, p.now_ms);
}

/* The transport, once, before the loop. A failure here is final for this boot: there is no
 * identifier to report it on. */
void wire_the_transport()
{
    tof_package_can::identifiers ids{};
    ids.data_id = TOF_PACKAGE_AUTO_DATA_ID;
    ids.health_id = TOF_PACKAGE_AUTO_HEALTH_ID;
    ids.extended = false;

    if (const int rc{tof_package_can::init()}; rc != 0) {
        LOG_ERR("package tof: can2 is not available (%d); nothing will be sent this boot", rc);
        return;
    }
    if (const int rc{tof_package_can::configure(ids)}; rc != 0) {
        LOG_ERR("package tof: identifiers 0x%x/0x%x refused (%d)", ids.data_id, ids.health_id, rc);
        return;
    }

    uint32_t heard{0};
    if (const int rc{tof_package_can::listen_for_traffic(kListenWindowMs, &heard)}; rc != 0) {
        LOG_ERR("package tof: could not listen on 0x%x/0x%x (%d); nothing will be sent this boot",
                ids.data_id, ids.health_id, rc);
        return;
    }
    if (heard != 0U) {
        LOG_ERR("package tof: %u frames on 0x%x/0x%x in %d ms. These identifiers are allocated to "
                "this stream, so another transmitter is misconfigured; this board will send "
                "nothing, including diagnostics.",
                heard, ids.data_id, ids.health_id, kListenWindowMs);
        return;
    }

    tof_package_stream::sequence_source source{};
    source.issue = tof_package_device::issue_boot_local;
    if (const int rc{tof_package_device::publisher().init(tof_package_can::sink(), source)};
        rc != 0) {
        LOG_ERR("package tof: the publisher refused the wiring (%d)", rc);
        return;
    }

    transport_ready_ = true;
    /* The wait for a first grid starts now rather than at uptime zero, so the listen window and
     * whatever boot took before it are not counted as a sensor that went quiet. */
    last_grid_ms_ = k_uptime_get();
    LOG_INF("package tof: data 0x%x, health 0x%x, 11-bit, boot-local sequence, quiet through a "
            "%d ms listen.", ids.data_id, ids.health_id, kListenWindowMs);
}

}  // namespace

void run(void *, void *, void *)
{
    auto_policy::limits l{};
    l.claims = TOF_PACKAGE_AUTO_CLAIMS;
    l.takeovers = TOF_PACKAGE_AUTO_TAKEOVERS;
    l.quiesce_first = TOF_PACKAGE_AUTO_QUIESCE_FIRST != 0;
    if (const int rc{policy_.init(l)}; rc != 0) {
        /* A configuration the policy refuses is a build mistake, and the thread ends rather than
         * running on a budget nobody chose. Nothing has been claimed and nothing is on the bus. */
        LOG_ERR("package tof: the runtime's budget is not usable (%d); this thread is doing "
                "nothing. claims=%u takeovers=%u",
                rc, l.claims, l.takeovers);
        return;
    }

    /* TELL THE FEEDER THERE IS A GRID SENSOR TO WATCH. Without this the L7 monitor never arms and
     * the bracketing in tof_package_device.cpp is inert -- feed_allowed() only starts watching when
     * l7_expected is set AND an operation has begun. Set after the budget check, because a thread
     * that is about to do nothing must not ask to be judged, and before the first bind, because the
     * probe inside it is already a bracketed operation. The chain's own caller is not compiled into
     * this image, so this is the only call. */
    tof_watchdog_feeder::set_l7_expected(true);

    wire_the_transport();

    while (true) {
        const auto_policy::observation seen{look()};
        const auto_policy::action what{policy_.next(seen)};

        switch (what) {
        case auto_policy::action::bind: {
            tof_package_device::bind_refusal why{tof_package_device::bind_refusal::none};
            if (const int rc{tof_package_device::bind(TOF_PACKAGE_AUTO_HZ, &why)}; rc != 0)
                LOG_ERR("package tof: bind refused: %s (%d)",
                        tof_package_device::bind_refusal_name(why), rc);
            break;
        }
        case auto_policy::action::rebind_to_retry: {
            /* The same call. A rebind IS a bind: what makes it a retry is that the lifecycle it
             * replaces was discardable, which the policy checked and this does not re-check. */
            tof_package_device::bind_refusal why{tof_package_device::bind_refusal::none};
            if (const int rc{tof_package_device::bind(TOF_PACKAGE_AUTO_HZ, &why)}; rc != 0)
                LOG_ERR("package tof: rebind refused: %s (%d)",
                        tof_package_device::bind_refusal_name(why), rc);
            break;
        }
        case auto_policy::action::claim_with_quiesce:
            /* Through the device layer's round entry, never the lifecycle directly: that is where
             * the step-record boundary is applied, and a caller that went round it would start a
             * round carrying the previous one's steps. */
            claim(false);
            break;
        case auto_policy::action::claim_by_takeover:
            /* The quiesce was refused, so this proceeds without it. `idle` says the software holds
             * no session, never that the sensor stopped: the full init that follows is then the
             * only thing that will have touched the device's state. */
            LOG_WRN("package tof: taking over a device whose state is unknown -- the stop "
                    "handshake was refused, and this skips it");
            claim(true);
            break;
        case auto_policy::action::pay_stop:
            if (const int rc{tof_package_device::lifecycle().stop()}; rc != 0)
                LOG_WRN("package tof: the stop this session owes was refused (%d)", rc);
            break;
        case auto_policy::action::acquire:
            acquire();
            break;
        case auto_policy::action::report_only:
            report(policy_.current().claims_used != 0U);
            break;
        }

        /* Every pass that was not an acquisition still owes the consumer a heartbeat: a claim that
         * is being retried looks, from the bus, exactly like a producer that died. */
        if (what != auto_policy::action::acquire && what != auto_policy::action::report_only)
            report(policy_.current().claims_used != 0U);

        k_msleep(kPollIntervalMs);
    }
}

}  // namespace lexxhard::tof_package_runtime

#endif  // ENABLE_TOF_PACKAGE_AUTO
