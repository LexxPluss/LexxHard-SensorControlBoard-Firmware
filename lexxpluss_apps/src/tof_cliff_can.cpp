/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tof_cliff_can.hpp"

#if defined(ENABLE_TOF_CHAIN) && defined(ENABLE_TOF_CLIFF_ULD)

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "tof_acquisition.hpp"
#include "tof_cliff_contract.h"
#include "tof_mapping_authority.hpp"

namespace lexxhard::tof_cliff_can {

LOG_MODULE_REGISTER(tof_cliff_can);

namespace {

namespace ctr = tof_cliff_contract;

const device *dev_{nullptr};

/* How long to wait for room in the transmit mailbox.
 *
 * Short, and deliberately not the 100 ms the other zcan modules use. A frame this layer
 * cannot place is dropped, never retried -- by the time the mailbox drains, the range it
 * carries is stale, and sending a stale range is the one thing the contract forbids
 * outright. So waiting longer cannot change the outcome for that frame; it can only delay
 * the caller. The flush runs after the chain lock is released, so a long block here would
 * not stall the I2C schedule, but it would still push the next cycle out and would sit in
 * front of the heartbeat.
 *
 * Not zero either: a momentary mailbox contention should not drop a frame that is still
 * fresh. One millisecond is well under any plausible cycle period and long enough for the
 * controller to finish a frame already in flight at 1 Mbit/s.
 *
 * WHAT THIS TIMEOUT ACTUALLY BOUNDS, because the obvious reading is wrong: it bounds only
 * "wait for a free TX mailbox". z_impl_can_send() implements the callback == NULL form as
 * api->send(...) followed by k_sem_take(&ctx.done, K_FOREVER) -- so waiting for the send to
 * COMPLETE is unconditional and this value does not constrain it at all.
 *
 * That was not academic. Before the bxCAN mailbox-overwrite backport
 * (patches/zephyr/0002-*), a second concurrent synchronous sender could have its completion
 * object overwritten and then never wake up; on DS20001 that left zcan_main pending forever
 * and took the whole legacy CAN telemetry plus the 0x20F control-frame consumer down with
 * it. The patch removes the overwrite. What remains bounded-but-lossy is mailbox exhaustion:
 * with all three busy, this 1 ms elapses and the frame is dropped with -EAGAIN, which the
 * publisher counts as a send failure and answers by withholding that cycle's health frame. */
constexpr k_timeout_t kSendTimeout{K_MSEC(1)};

int send(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (dev_ == nullptr)
        return -ENODEV;
    if (data == nullptr || dlc != ctr::kDlc)
        return -EINVAL;

    can_frame frame{};
    frame.id = can_id;   // standard 11-bit: no CAN_FRAME_IDE in flags
    frame.dlc = dlc;
    memcpy(frame.data, data, dlc);

    /* Blocking form rather than the callback form: the publisher has already left every lock,
     * and a completion callback would put the counter update in yet another context for no
     * gain. Note that "blocking" here is unbounded on the completion side -- see kSendTimeout
     * above for what the 1 ms does and does not cover. */
    return can_send(dev_, &frame, kSendTimeout, nullptr, nullptr);
}

} // namespace

int init()
{
    /* zcan_main owns can2 -- bitrate, mode and start. Configuring it again here would fight
     * whichever ran second. */
    dev_ = DEVICE_DT_GET(DT_NODELABEL(can2));
    if (!device_is_ready(dev_)) {
        LOG_ERR("can2 is not ready; cliff ToF frames will not be sent");
        dev_ = nullptr;
        return -ENODEV;
    }
    return 0;
}

struct tof_cliff_pub::can_sink sink()
{
    struct tof_cliff_pub::can_sink s{};
    s.send = send;
    return s;
}

struct tof_cliff_pub::authorisation production_authorisation()
{
    /* ONE read of the authority, then the clamp applied to that snapshot.
     *
     * Not effective_mapping_state() plus current().epoch: that reads the authority twice, and
     * a proof committing between the two reads yields a pair that never existed -- LOST with
     * the new epoch, say. The publisher latches this pair per cycle and re-checks it before
     * flushing, so a pair that never existed would be latched as though it had.
     *
     * The clamp still applies, and still lives in the acquisition layer. Taking the state
     * straight from the snapshot would bypass it, which is the shape of the safety backdoor
     * this project deleted once. */
    const tof_authority::snapshot now{tof_authority::current()};

    struct tof_cliff_pub::authorisation a{};
    a.state = tof_acq::clamp_mapping_state(now.state);
    a.epoch = now.epoch;
    a.enumerated_mask = now.enumerated_mask;
    a.model_verified_mask = now.model_verified_mask;
    a.chain_flags = now.chain_flags;
    a.failing_position = now.failing_position;
    return a;
}

} // namespace lexxhard::tof_cliff_can

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
