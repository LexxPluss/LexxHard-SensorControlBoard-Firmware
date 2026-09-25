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
#include "tof_mapping_proof.hpp"
#include "tof_diag_hang.hpp"
#include "tof_progress.hpp"

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
    /* WHICH SENDER THIS IS, decided from the calling thread rather than from the CAN id: the id
     * says what the frame is, the thread says who would be stuck in it. The acquisition slot carries
     * 0x214, 0x215, 0x216 and the cycle health frame; the system work queue carries the 0x217
     * heartbeat. They wedge independently, so the watchdog watches them independently. */
    const tof_progress::activity slot{tof_progress::current_send_slot()};
    tof_progress::begin(slot);
#if defined(TOF_DIAG_HANG)
    tof_diag::send_begin(can_id);
    const int rc{can_send(dev_, &frame, kSendTimeout, nullptr, nullptr)};
    tof_diag::send_end(rc);
#else
    const int rc{can_send(dev_, &frame, kSendTimeout, nullptr, nullptr)};
#endif
    /* Counted on return whatever the result. A send that failed is a sender that is alive; a sender
     * that never returns is the thing being watched for, and it never reaches this line. */
    tof_progress::end(slot);
    return rc;
}

#if defined(TOF_DIAG_HANG) && defined(ENABLE_TOF_L7_ULD)
/* DEV hang isolation, and THE one line that separates mode 1 from mode 3.
 *
 * Mode 1 keeps every grid frame off the bus -- the publisher, the packer and all of their
 * accounting still run, only the transmit is replaced -- so the real L7 path could be run without
 * adding a single frame to the shared CAN transmit path. That was the right isolation while the
 * question was which of the two suspects hung the board, and it is exactly why no real L7 distance
 * has ever reached the driver: the frames were built and then dropped here.
 *
 * Modes 2 and 3 send them, counted. Mode 2's are fabricated, mode 3's are the real sensors'. */
int send_grid(uint16_t can_id, const uint8_t *data, uint8_t dlc)
{
#if TOF_DIAG_SEND_GRID
    tof_diag::grid_sent();
    return send(can_id, data, dlc);
#else
    (void)can_id;
    (void)data;
    (void)dlc;
    tof_diag::grid_suppressed();
    return 0;
#endif
}
#endif

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
struct tof_grid_pub::can_sink grid_sink()
{
    struct tof_grid_pub::can_sink s{};
    /* The same send(). The grid frames are 0x214/0x215 and the cliff frames 0x216/0x217, all
     * DLC 8 on the same controller; a second sink would be a second copy of the same three
     * lines and one more place for the timeout to diverge. */
#if defined(TOF_DIAG_HANG)
    s.send = send_grid;
#else
    s.send = send;
#endif
    return s;
}

struct tof_grid_pub::authorisation grid_production_authorisation()
{
    /* ONE read, for the same reason as the cliff's. */
    const tof_authority::snapshot now{tof_authority::current()};

    struct tof_grid_pub::authorisation a{};
    a.state = now.state;
    a.epoch = now.epoch;
    /* The proven chain's length, and reported only when it is proven. A PROVEN mapping means
     * every position of the commissioning profile enumerated and verified, so the number is a
     * fact about that proof rather than a count taken here; outside PROVEN nothing is published
     * and zero is the honest answer. */
    a.boards_detected = now.state == tof_acq::mapping_state::proven
                            ? static_cast<uint8_t>(tof_proof::kCommissioningPositions)
                            : 0;
    /* Both false by construction -- see the header. Not "not implemented": there is no state in
     * which this firmware publishes a grid while either is true. */
    a.chain_length_unexpected = false;
    a.other_position_enumeration_failed = false;
    return a;
}
#endif

struct tof_cliff_pub::authorisation production_authorisation()
{
    /* ONE read of the authority, and every field below comes from that one snapshot.
     *
     * Not effective_mapping_state() plus current().epoch: that reads the authority twice, and
     * a proof committing between the two reads yields a pair that never existed -- LOST with
     * the new epoch, say. The publisher latches this pair per cycle and re-checks it before
     * flushing, so a pair that never existed would be latched as though it had. That rule is
     * unchanged by the clamp's removal and is the reason this function exists at all. */
    const tof_authority::snapshot now{tof_authority::current()};

    struct tof_cliff_pub::authorisation a{};
    a.state = now.state;
    a.epoch = now.epoch;
    a.enumerated_mask = now.enumerated_mask;
    a.model_verified_mask = now.model_verified_mask;
    a.chain_flags = now.chain_flags;
    a.failing_position = now.failing_position;
    return a;
}

} // namespace lexxhard::tof_cliff_can

#endif // ENABLE_TOF_CHAIN && ENABLE_TOF_CLIFF_ULD
