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

#include "tof_package_can.hpp"

#if defined(ENABLE_TOF_PACKAGE_BENCH) || defined(ENABLE_TOF_PACKAGE_AUTO)

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

namespace lexxhard::tof_package_can {

LOG_MODULE_REGISTER(tof_package_can);

namespace {

const device *dev_{nullptr};
identifiers ids_{};
bool configured_{false};

/* Wait up to 1 ms for a mailbox per frame. A non-null callback avoids the
 * synchronous can_send() path's K_FOREVER wait for ACK completion.
 * This is not a whole-call deadline: underlying driver locks may still wait.
 */
constexpr k_timeout_t kMailboxTimeout{K_MSEC(1)};

/* Completion failures reported by the driver since the last init(). An atomic because the callback
 * runs in the driver's context, not the caller's, and a counter is all it may do there. */
atomic_t transmit_errors_;

/* NON-BLOCKING BY OBLIGATION. It runs wherever the CAN driver finishes a frame, so it increments a
 * counter and returns; anything that could wait belongs on the caller's thread. */
void on_transmit_complete(const device *, int error, void *)
{
    if (error != 0)
        atomic_inc(&transmit_errors_);
}

/* Four slots, for a window whose only output is a count. The listen loop drains continuously, so
 * depth buys nothing except tolerance for a burst arriving between drains; a dropped frame would
 * understate a count that is already only an indication. */
K_MSGQ_DEFINE(listen_msgq, sizeof(can_frame), 4, 4);

uint32_t width_mask(bool extended)
{
    return extended ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK;
}

int send(uint32_t can_id, const uint8_t *frame_bytes, size_t frame_len)
{
    if (dev_ == nullptr || !configured_)
        return -EPERM;
    if (frame_bytes == nullptr || frame_len != tof_package_wire_n::kFrameBytes)
        return -EINVAL;

    can_frame frame{};
    frame.id = can_id;
    frame.dlc = static_cast<uint8_t>(frame_len);
    if (ids_.extended)
        frame.flags |= CAN_FRAME_IDE;
    memcpy(frame.data, frame_bytes, frame_len);

    /* SUCCESS HERE MEANS THE CONTROLLER TOOK THE FRAME, NOT THAT ANYBODY RECEIVED IT. Zero says a
     * mailbox accepted it within kMailboxTimeout. Whether it then won arbitration, was acknowledged
     * by another node, or was retransmitted until the controller gave up is reported later and
     * elsewhere -- on_transmit_complete() counts the failures, and transmit_errors() is how a
     * caller sees them. Waiting here for that answer is exactly what the null-callback form does,
     * and what this must not do. */
    return can_send(dev_, &frame, kMailboxTimeout, on_transmit_complete, nullptr);
}

int send_data(void *, const uint8_t *frame_bytes, size_t frame_len)
{
    return send(ids_.data_id, frame_bytes, frame_len);
}

int send_health(void *, const uint8_t *frame_bytes, size_t frame_len)
{
    return send(ids_.health_id, frame_bytes, frame_len);
}

}  // namespace

uint32_t transmit_errors()
{
    return static_cast<uint32_t>(atomic_get(&transmit_errors_));
}

int init()
{
    /* Cleared here so the count belongs to the run being started. A late completion from a previous
     * run can still land on it, which is the honest arrangement: the frame really did fail, and
     * attributing it to the run that is listening is better than dropping it. */
    atomic_clear(&transmit_errors_);

    dev_ = DEVICE_DT_GET(DT_NODELABEL(can2));
    if (!device_is_ready(dev_)) {
        LOG_ERR("can2 is not ready; no Package ToF frame can be sent");
        dev_ = nullptr;
        return -ENODEV;
    }
    return 0;
}

int configure(const identifiers &ids)
{
    /* Cleared first, so a refused pair leaves a transport that sends nothing rather than one still
     * holding whatever was accepted last. */
    configured_ = false;

    /* Zero means unset. Identifier zero is legal on a bus, and refusing it costs the stream one
     * number out of 2048 while removing the whole class of "the field was never filled in". */
    if (ids.data_id == 0U || ids.health_id == 0U)
        return -EINVAL;
    /* The two must be distinguishable, because the identifier is the ONLY thing that says whether
     * eight bytes are a zone measurement or a health verdict -- both are eight bytes with a
     * sequence in the first four. One identifier for both would make the stream undecodable. */
    if (ids.data_id == ids.health_id)
        return -EINVAL;
    if (ids.data_id > width_mask(ids.extended) || ids.health_id > width_mask(ids.extended))
        return -EINVAL;

    ids_ = ids;
    configured_ = true;
    return 0;
}

identifiers current()
{
    return ids_;
}

int listen_for_traffic(int32_t window_ms, uint32_t *frames_seen)
{
    if (dev_ == nullptr || !configured_)
        return -EPERM;
    if (frames_seen == nullptr || window_ms < 0)
        return -EINVAL;

    *frames_seen = 0;
    k_msgq_purge(&listen_msgq);

    const can_filter data_filter{
        .id = ids_.data_id,
        .mask = width_mask(ids_.extended),
        .flags = static_cast<uint8_t>(ids_.extended ? CAN_FILTER_IDE : 0U),
    };
    const can_filter health_filter{
        .id = ids_.health_id,
        .mask = width_mask(ids_.extended),
        .flags = static_cast<uint8_t>(ids_.extended ? CAN_FILTER_IDE : 0U),
    };

    const int data_id{can_add_rx_filter_msgq(dev_, &listen_msgq, &data_filter)};
    if (data_id < 0)
        return data_id;
    const int health_id{can_add_rx_filter_msgq(dev_, &listen_msgq, &health_filter)};
    if (health_id < 0) {
        /* The first filter is removed before returning: leaving it installed would keep feeding a
         * queue nobody drains, and the next window would start by counting old frames. */
        can_remove_rx_filter(dev_, data_id);
        return health_id;
    }

    /* Drained as we wait rather than once at the end, so a window longer than four frames' worth of
     * traffic still counts every frame instead of overflowing the queue. */
    const int64_t deadline{k_uptime_get() + window_ms};
    for (;;) {
        const int64_t remaining{deadline - k_uptime_get()};
        if (remaining <= 0)
            break;
        can_frame frame{};
        if (k_msgq_get(&listen_msgq, &frame, K_MSEC(remaining > 50 ? 50 : remaining)) == 0)
            ++*frames_seen;
    }
    /* One last drain, for frames that landed inside the final wait. */
    can_frame frame{};
    while (k_msgq_get(&listen_msgq, &frame, K_NO_WAIT) == 0)
        ++*frames_seen;

    can_remove_rx_filter(dev_, health_id);
    can_remove_rx_filter(dev_, data_id);
    return 0;
}

tof_package_stream::sink sink()
{
    tof_package_stream::sink s{};
    s.send_data = send_data;
    s.send_health = send_health;
    return s;
}

}  // namespace lexxhard::tof_package_can

#endif  // ENABLE_TOF_PACKAGE_BENCH || ENABLE_TOF_PACKAGE_AUTO
