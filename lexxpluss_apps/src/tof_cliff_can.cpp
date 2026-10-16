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

#include "runtime_progress.hpp"
#include "tof_acquisition.hpp"
#include "tof_cliff_contract.h"
#include "tof_mapping_authority.hpp"
#include "zcan_bounded_send.hpp"

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
 * "wait for a free TX mailbox". Waiting for the frame to COMPLETE on the wire is a separate
 * thing, and it used to be unbounded here -- z_impl_can_send() implements the
 * callback == NULL form as api->send(...) followed by k_sem_take(&ctx.done, K_FOREVER).
 * This layer now goes through zcan_bounded_send::send(), which passes a callback and so
 * returns once the frame is in a mailbox; see that header for why every sender in this
 * application was changed. With that, this 1 ms is the whole bound on the call.
 *
 * That unbounded completion was not academic. Before the bxCAN mailbox-overwrite backport
 * (patches/zephyr/0002-*), a second concurrent synchronous sender could have its completion
 * object overwritten and then never wake up; on DS20001 that left zcan_main pending forever
 * and took the whole legacy CAN telemetry plus the 0x20F control-frame consumer down with
 * it. The patch removed the overwrite; the bounded send removes the wait.
 *
 * What remains bounded-but-lossy is mailbox exhaustion: with all three busy, this 1 ms
 * elapses and the frame is dropped with -EAGAIN, which the publisher counts as a send
 * failure and answers by withholding that cycle's health frame. */
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

    /* WHICH SENDER THIS IS, decided from the calling thread rather than from the CAN id: the id
     * says what the frame is, the thread says who would be stuck in it. The acquisition slot
     * carries 0x214, 0x215, 0x216 and the cycle health frame; the system work queue carries the
     * 0x217 heartbeat. They wedge independently, so the watchdog watches them independently. */
    const runtime_progress::activity slot{runtime_progress::current_send_slot()};
    runtime_progress::begin(slot);

    /* THE BOUNDED SEND, not can_send() directly, and WHAT A ZERO FROM HERE NOW MEANS, because it
     * changed: the controller accepted the frame, not that a node acknowledged it. The publisher's
     * send_failed_measurement and send_failed_health therefore count transport REFUSALS -- no
     * mailbox, bus off, bus not started -- and no longer count frames that went out and were never
     * answered. On a bus with nobody listening the first three frames are accepted and only the
     * fourth is refused, so the withholding starts three frames later than it used to; the
     * alternative was the previous behaviour, where that bus blocked this thread forever and the
     * withholding never happened at all. Delivery evidence lives in zcan_bounded_send::snapshot(),
     * where a silent bus reads as `refused` rising while `queued` is frozen -- NOT as a rising
     * `failed`, since frames still being retransmitted into silence never complete at all. */
    const int rc{zcan_bounded_send::send(dev_, &frame, kSendTimeout)};

    /* ENDED ON RETURN WHATEVER THE RESULT, and a refusal is a return. A send that failed is a
     * sender that is alive; a sender that never returns is the thing being watched for, and it
     * never reaches this line. Ending only on a zero would leave `begun` permanently ahead of
     * `ended` once the bus went quiet, which is exactly the shape of a wedged sender -- so a host
     * that went away would be indistinguishable from the fault this pair exists to detect. */
    runtime_progress::end(slot);
    return rc;
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

    /* BIT 2 IS NOT SET FROM HERE, and there is no field for it to be set through. It says "the
     * chain_position -> source_id binding cannot be trusted", and authorisation carried a
     * chain-level binding_untrusted until #118 removed it: its only effect was to set the bit on
     * every source, which the packer reads as a contradiction against source_allowed and refuses,
     * taking a whole cycle down over one position. source_allowed[] above already says this per
     * source and says it as the enumerator's verdict. Nothing in this snapshot establishes bit 2
     * either way, so nothing here claims it; the packer's own check still guards the wire against
     * a non-conforming caller. */
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
