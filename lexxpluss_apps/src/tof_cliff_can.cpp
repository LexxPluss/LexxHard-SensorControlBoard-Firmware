/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
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

#if defined(ENABLE_TOF_L7_ULD)
/* THE GRID PAIR SHARE THIS BUS AND THIS SENDER. Same device, same behaviour -- and deliberately
 * the same function, because two senders would be two places for the device handle and the timeout
 * to drift apart. The identifiers differ and that is the publisher's business, not this layer's.
 *
 * THE 1 ms BOUNDS THE WAIT FOR A FREE MAILBOX, NOT THE SEND. can_send()'s synchronous form returns
 * when transmission COMPLETES, and nothing here bounds that; what the timeout covers is how long a
 * caller waits when all three mailboxes are busy, after which the frame is dropped with -EAGAIN. */
struct tof_grid_pub::can_sink grid_sink()
{
    struct tof_grid_pub::can_sink s{};
    s.send = send;
    return s;
}

struct tof_grid_pub::authorisation grid_production_authorisation()
{
    /* ONE read of the authority, then the clamp applied to that snapshot -- the same rule and for
     * the same reason as the cliff gate above: reading twice can yield a pair that never existed,
     * and the publisher latches this per cycle. The single read is this line; everything derived
     * from it is derived from this one value.
     *
     * THE CLAMP IS WHY NOTHING IS PUBLISHED. clamp_mapping_state() makes PROVEN unreachable by
     * construction, the packer refuses anything that is not PROVEN, and so wiring this sender
     * changes what the board CAN do and not what it does. Lifting the clamp is a release decision
     * and is not made by connecting a function pointer. */
    return grid_authorisation_from(tof_authority::current());
}

struct tof_grid_pub::authorisation grid_authorisation_from(const tof_authority::snapshot &now)
{
    struct tof_grid_pub::authorisation a{};
    a.state = tof_acq::clamp_mapping_state(now.state);
    a.epoch = now.epoch;
    /* DIAGNOSTICS ONLY, and the authority does not carry it. There is no boards_detected in the
     * snapshot, so it is left at zero rather than derived from something that is not it: the
     * contract says this nibble is for diagnostics, and a plausible-looking wrong number is worse
     * for a diagnostic than an honest zero. Whatever eventually owns it is its own change. */
    a.boards_detected = 0;

    /* PER SOURCE, from THIS snapshot, so a grid is admitted by the permission read for the source
     * id it is labelled with as one step -- the obligation tof_grid_packer states on the producer.
     * Taking them from two reads is exactly what that rule forbids.
     *
     * FROM grid_source_mask AND NOT enumerated_mask. The latter is keyed by source_id_of(l4_role),
     * so its bits 0 and 1 are the front_left and rear_left CLIFF sensors; they agree with the grid
     * pair's permission only by coincidence of the current chain profile, and under the clamp that
     * coincidence would never have been noticed. grid_source_mask is the authority's statement
     * about grid sources, published in this same snapshot. */
    for (int i{0}; i < tof_grid_pub::kGridSources; ++i)
        a.source_allowed[i] = (now.grid_source_mask & (1U << i)) != 0U;

    /* Bit 2 is "the chain_position -> source_id binding cannot be trusted", which contract
     * 2026-08-02i deliberately separated from chain length. Nothing in this snapshot establishes
     * it, so it stays false rather than being derived from something that is not it. */
    a.binding_untrusted = false;
    /* kNoFailingPosition is 0xFF, not 0. Comparing against zero would have reported a failure on
     * every healthy chain -- the default IS the no-failure value. */
    a.other_position_enumeration_failed = now.failing_position != tof_authority::kNoFailingPosition;
    return a;
}
#endif

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
