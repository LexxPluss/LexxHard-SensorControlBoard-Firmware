/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "tof_commission_bind.hpp"

#if defined(ENABLE_TOF_AUTO_COMMISSION)

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#include "tof_cliff_contract.h"
#include "tof_commission_entropy.hpp"
#include "tof_commission_runtime.hpp"
#include "tof_commissioning.hpp"

/* DECLARED RATHER THAN INCLUDED. tof_cliff_runtime.hpp pulls in the vendor ULD headers, and this
 * file needs exactly one function from it -- so including it would drag the sensor driver into every
 * build that wires the downlink, including the tests. A signature that stopped matching would fail
 * at link time, which is a hard failure rather than silent drift. */
namespace lexxhard::tof_cliff_runtime {
int start_acquisition();
}

namespace lexxhard::tof_commission_bind {

namespace rt = tof_commission_runtime;
namespace ctr = tof_cliff_contract;

namespace {

const struct device *can_{nullptr};
config cfg_{};

/* THE IDENTIFIERS, from the one place they are written down. A literal here would be a second
 * spelling of a value the contract owns, and the two would drift while both kept compiling. */
constexpr uint32_t kRequestId{ctr::kCommissionRequestId};
constexpr uint32_t kStatusId{ctr::kCommissionStatusId};
static_assert(kRequestId != kStatusId, "the request and status identifiers must differ");
static_assert(kRequestId <= CAN_STD_ID_MASK && kStatusId <= CAN_STD_ID_MASK,
              "this bus is CAN classic with standard identifiers");

void send_done(const struct device *, int, void *)
{
    /* Nothing. The runtime counts what it could not QUEUE; a frame that was queued and then lost on
     * the bus is the host's retransmission to notice, and counting it here would put a second
     * meaning on the same counter. */
}

int bind_send(void *, uint32_t id, const uint8_t *data, size_t len)
{
    if (can_ == nullptr || data == nullptr || len != 8)
        return -EINVAL;

    struct can_frame frame {};
    frame.id = id;   /* standard 11-bit: no CAN_FRAME_IDE */
    frame.dlc = static_cast<uint8_t>(len);
    memcpy(frame.data, data, len);

    /* THREAD CONTEXT ONLY. The runtime never calls this from on_frame(): it queues there and a work
     * item drains the queue, because this call takes a K_FOREVER mutex inside the driver no matter
     * what timeout it is given. The CALLBACK form even so -- the synchronous one waits for
     * transmission to complete, which is a thread held for a bus arbitration. */
    return can_send(can_, &frame, K_NO_WAIT, send_done, nullptr);
}

bool bind_permitted(void *)
{
    return cfg_.enumeration_permitted != nullptr && cfg_.enumeration_permitted(cfg_.ctx);
}

bool bind_installed(void *, uint8_t *epoch)
{
    return cfg_.installed_mapping != nullptr && cfg_.installed_mapping(cfg_.ctx, epoch);
}

int bind_prove(void *, uint32_t epoch, tof_commissioning::outcome *out)
{
    const tof_commissioning::outcome r{tof_commissioning::prove(epoch)};
    if (out != nullptr)
        *out = r;
    /* The transaction reports WHERE it stopped; `none` is the only success. Translating it to an
     * int here rather than in the runtime keeps the runtime free of the transaction's vocabulary. */
    return r.failed_at == tof_commissioning::stage::none ? 0 : -EIO;
}

int bind_start(void *)
{
    /* The real acquisition start, which refuses on its own unless the authority is PROVEN right now
     * under the epoch the descriptors were keyed with. Two independent checks of the same thing, on
     * purpose. */
    return tof_cliff_runtime::start_acquisition();
}

void on_can(const struct device *, struct can_frame *frame, void *)
{
    if (frame == nullptr)
        return;
    /* Classic CAN: the DLC is the byte count. */
    rt::on_frame(frame->id, frame->data, frame->dlc);
}

} // namespace

result start(const struct device *can_dev, const config &cfg)
{
    result out{};

    /* REFUSED BEFORE ANYTHING MOVES, and the order is the point.
     *
     * This function calls rt::init() unconditionally, and tof_commission_runtime.hpp states ONE
     * INIT PER BOOT: init() draws a new token, purges the queues and resets the attempt budgets,
     * and the worker is not stoppable, so a second init under a live worker moves the ground under
     * whatever that worker is doing. Nothing in the runtime enforces it -- the header says the
     * callers respect it -- so the enforcement belongs here, at the one call site that does the
     * init.
     *
     * A LIVE WORKER IS THE CONDITION, not "has this function been called before". A start that
     * drew no session never creates a worker, and its repeated init() has nothing to move the
     * ground under; refusing those would break the no-session cases for no gain. What must never
     * happen twice is an init while a worker is running.
     *
     * AND IT IS CHECKED HERE RATHER THAN REPORTED LATER. An earlier revision let the second call
     * run to completion and reported `answering_only` when rt::start() came back -EALREADY. By
     * then can_, cfg_ and the whole session layer had already been rewritten, and the state was
     * untrue besides: the previous worker was still running, so the board was commissioning, not
     * merely answering. The damage is the init, not the report. */
    if (rt::running()) {
        out.state = outcome::refused;
        out.rc = -EALREADY;
        return out;
    }

    if (cfg.enumeration_permitted == nullptr) {
        /* A board must not re-enumerate its chain because nobody said it should not. */
        out.state = outcome::refused;
        out.rc = -EINVAL;
        return out;
    }
    if (can_dev == nullptr || !device_is_ready(can_dev)) {
        out.state = outcome::refused;
        out.rc = -ENODEV;
        return out;
    }

    can_ = can_dev;
    cfg_ = cfg;

    rt::config rc{};
    rc.request_id = kRequestId;
    rc.status_id = kStatusId;
    rc.announce_period_ms = cfg.announce_period_ms;
    rc.poll_ms = cfg.poll_ms;
    rc.profile_enabled = cfg.profile_enabled;
    rc.max_proof_attempts = cfg.max_proof_attempts;
    rc.max_start_attempts = cfg.max_start_attempts;

    const rt::hooks hooks{bind_send,  bind_permitted, bind_prove,
                          bind_start, bind_installed, tof_commission_entropy::draw_token,
                          nullptr};
    out.rc = rt::init(rc, hooks);

    if (out.rc != 0 && out.rc != -ENODEV) {
        /* A configuration the runtime refused. Installing a filter now would take an identifier off
         * the bus with nothing behind it to answer on it. */
        out.state = outcome::refused;
        can_ = nullptr;
        return out;
    }

    /* THE FILTER GOES IN EITHER WAY, which is the whole reason -ENODEV is not lumped in above: a
     * board with no session still has to answer `no_session`, or a host holding a durable pending
     * request retransmits into silence for ever. */
    const struct can_filter filter {
        .id = kRequestId, .mask = CAN_STD_ID_MASK, .flags = 0,
    };
    out.filter_id = can_add_rx_filter(can_, on_can, nullptr, &filter);
    if (out.filter_id < 0) {
        out.state = outcome::refused;
        out.rc = out.filter_id;
        can_ = nullptr;
        return out;
    }
    out.filter_installed = true;

    if (out.rc == -ENODEV) {
        /* Answering, not commissioning. No worker: with no session every request is refused in the
         * receive path and there is nothing for one to do. */
        out.state = outcome::answering_only;
        return out;
    }

    /* `running` MEANS A WORKER IS RUNNING. It used to be set unconditionally, leaving
     * worker_started as the only trace, which is what let the suite's `worker_started ||
     * rt::running()` read the previous case's worker and agree.
     *
     * THE FAILURE BRANCH IS UNREACHABLE BEHIND THE GUARD ABOVE and is still reported rather than
     * papered over. rt::start() refuses with -EPERM only when the runtime is unconfigured or has
     * no session, which out.rc == 0 has just ruled out, and with -EALREADY only when a worker
     * exists, which rt::running() was asked about before anything moved. Reaching it means two
     * threads called this at once, and then the truthful answer is that THIS call delivered no
     * worker: the filter it just added is taken back out so the result describes one thing instead
     * of half of two. The filter and the worker the other caller installed are left alone, and so
     * is can_, which that worker sends through. */
    const int start_rc{rt::start()};
    out.worker_started = start_rc == 0;
    if (!out.worker_started) {
        can_remove_rx_filter(can_, out.filter_id);
        out.filter_installed = false;
        out.filter_id = -1;
        out.state = outcome::refused;
        out.rc = start_rc;
        return out;
    }
    out.state = outcome::running;
    return out;
}

} // namespace lexxhard::tof_commission_bind

#endif // ENABLE_TOF_AUTO_COMMISSION
