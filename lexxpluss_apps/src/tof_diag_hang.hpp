/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DEV ONLY: the instrumentation of the L7 hang-isolation images (TOF_DIAG_HANG = 1, 2 or 3).
 *
 * WHY IT EXISTS. The first long run of the L7 E2E image stopped every thread on the SCB after about
 * nine minutes -- no CAN frame, no shell -- while the IWDG, fed from a timer callback, kept the board
 * from resetting. Nothing was left to read: the console had gone quiet, and the on-site power cycle
 * erased RAM. Two images isolate the two suspects:
 *
 *   TOF_DIAG_HANG=1  the real L7 path -- ULD download, ranging, reads -- with the grid frames
 *                    NOT put on CAN. A hang here points at I2C / ULD / acquisition.
 *   TOF_DIAG_HANG=2  NO L7 bus access; a static grid is fed through the real grid publisher and
 *                    the real synchronous CAN send at the same 2 x 16 + health, 5 Hz cadence. A
 *                    hang here points at the shared synchronous CAN transmit path.
 *
 * Mode 1 answered the question: with the ISR backport the real L7 read path ran thirty minutes
 * armed without wedging. But mode 1 drops every grid frame at the last step, so no real L7 distance
 * has ever left the SCB, and the driver, the assembler and the detector above it have never seen
 * one. Hence a third mode:
 *
 *   TOF_DIAG_HANG=3  mode 1 with that one drop removed: the real L7 AND its grid frames on CAN.
 *                    It is the smallest change that lets the observe-only end-to-end run happen,
 *                    and it deliberately keeps everything mode 1 earned -- the ISR backport, the
 *                    DTCM record, the armed watchdog, `arm` before anything risky starts, and an
 *                    unconfirmed image that any reset reverts to E3. It is NOT a step towards
 *                    release: still DEV, still NO-FAT, still bypassed, still not driveable.
 *
 * Neither starts its risky path at boot: `tofdiag arm` does, once the console and a CAN capture are
 * running -- and `arm` itself refuses until this boot has shown a completed CAN send, health item,
 * acquisition cycle and zcan loop with nothing in flight, because until the robot PC is back on the
 * bus after the DFU power cycle every synchronous send legitimately blocks.
 *
 * THREE THINGS SURVIVE A HANG NOW:
 *   - a progress record in DTCM (see kRecordAddress), which no image, and not MCUboot, links
 *     anything into, and which a reset -- unlike a power cycle -- does not clear. After the reset
 *     MCUboot reverts this unconfirmed image and E3 runs; `devmem` from E3 reads the record back.
 *   - a watchdog that, ONCE ARMED, is fed only while the watched work keeps finishing, so a
 *     thread-level hang becomes an IWDG reset instead of a board that is silent until somebody
 *     pulls the power. Unarmed it judges only the clock, and hands the board back to the confirmed
 *     image if nobody armed it within kUnarmedRevertMs.
 *   - a fatal-error handler that writes reason, PC, LR and thread into the record before halting.
 */

#pragma once

#include <stdint.h>

#if defined(TOF_DIAG_HANG)

/* The mode number is two independent choices, and naming them separately is what stops a third
 * mode from being a third copy of everything. Reading the real sensors and putting grid frames on
 * CAN are unrelated: mode 1 does the first, mode 2 the second, mode 3 both. Every site that used
 * to ask "is this mode 1" asks one of these instead, so adding mode 3 changed no behaviour in the
 * two modes that were already measured. */
#define TOF_DIAG_REAL_L7   (TOF_DIAG_HANG == 1 || TOF_DIAG_HANG == 3)
#define TOF_DIAG_SEND_GRID (TOF_DIAG_HANG == 2 || TOF_DIAG_HANG == 3)

#define TOF_DIAG(stmt) stmt

namespace lexxhard::tof_diag {

/* The top 4 KiB of DTCM (0x20000000, 128 KiB). The application links nothing into DTCM (its .dtcm
 * sections are empty in every image) and MCUboot's RAM is sram0 at 0x20020000. */
inline constexpr uintptr_t kRecordAddress{0x2001F000};
inline constexpr uint32_t kMagic{0x31474E48};    // "HNG1"
inline constexpr uint32_t kMagicEnd{0x444E4548}; // "HEND"
inline constexpr int kRing{32};

/* Why the watchdog stopped being fed. A bit per watched activity. */
enum reason : uint32_t {
    stuck_cycle = 1U << 0,  // an acquisition cycle began and has not ended
    stuck_send = 1U << 1,   // a ToF CAN send began and has not returned
    stuck_health = 1U << 2, // a health work item began and has not ended
    stuck_zcan = 1U << 3,   // the zcan_main loop has stopped going round
    /* Not a hang: nobody armed this image. It is unconfirmed, so letting the watchdog go gives the
     * board back to E3 instead of leaving a diagnostic image running forever. */
    unarmed_timeout = 1U << 4,
};

/* Why `tofdiag arm` refused. The risky path may only start from a baseline where everything the
 * watchdog will judge has already been seen to work on this boot -- otherwise the first judgement
 * would be about the post-DFU boot, where every CAN send blocks until the robot PC is back on the
 * bus to acknowledge a frame. */
enum arm_block : uint32_t {
    arm_no_send = 1U << 0,       // no ToF CAN send has completed
    arm_no_health = 1U << 1,     // no health work item has completed
    arm_no_zcan = 1U << 2,       // the zcan_main loop has not gone round
    arm_no_cycle = 1U << 3,      // no acquisition cycle has completed (commissioning not done)
    arm_send_in_flight = 1U << 4,
    arm_health_in_flight = 1U << 5,
    arm_cycle_in_flight = 1U << 6,
    arm_slot_active = 1U << 7,   // a sender is inside a send right now
};

/* Everything is a 32-bit word so that `devmem` can read it back one word at a time. Offsets are part
 * of the reading procedure; append only. */
struct record {
    uint32_t magic;           // 0x00
    uint32_t mode;            // 0x04 1 or 2
    uint32_t armed;           // 0x08
    uint32_t armed_ms;        // 0x0c
    uint32_t eval_ms;         // 0x10 last watchdog evaluation (uptime)
    uint32_t acq_begin;       // 0x14 cycles begun
    uint32_t acq_end;         // 0x18 cycles ended (after the publisher's sends)
    uint32_t acq_begin_ms;    // 0x1c
    uint32_t acq_end_ms;      // 0x20
    uint32_t send_begin;      // 0x24 ToF CAN sends begun (cliff + grid + health)
    uint32_t send_end;        // 0x28 ToF CAN sends returned
    uint32_t send_id;         // 0x2c CAN id of the most recent send begun, ANY sender
    uint32_t send_rc;         // 0x30 rc of the most recent send returned, ANY sender
    uint32_t send_begin_ms;   // 0x34 (these three are last-writer-wins across senders: use slot[])
    uint32_t send_fail;       // 0x38 sends that returned non-zero
    uint32_t grid_suppressed; // 0x3c mode 1: grid frames the publisher handed over and we dropped
    uint32_t grid_sent;       // 0x40 grid frames (0x214/0x215) sent
    uint32_t health_begin;    // 0x44
    uint32_t health_end;      // 0x48
    uint32_t zcan_loops;      // 0x4c
    uint32_t l7_open_begin[2]; // 0x50
    uint32_t l7_open_end[2];   // 0x58
    uint32_t l7_open_rc[2];    // 0x60
    uint32_t l7_read_begin[2]; // 0x68
    uint32_t l7_read_end[2];   // 0x70
    uint32_t l7_fresh[2];      // 0x78
    uint32_t l7_read_rc[2];    // 0x80
    uint32_t wdt_feeds;       // 0x88
    uint32_t wdt_withheld;    // 0x8c 1 once the watchdog is no longer fed (latched)
    uint32_t wdt_reason;      // 0x90 reason bits
    uint32_t wdt_withheld_ms; // 0x94
    uint32_t fatal_reason;    // 0x98 0xffffffff = none
    uint32_t fatal_pc;        // 0x9c
    uint32_t fatal_lr;        // 0xa0
    uint32_t fatal_ms;        // 0xa4
    char fatal_thread[16];    // 0xa8
    uint32_t ring_next;       // 0xb8
    struct {
        uint32_t ms;
        uint32_t event; // code << 24 | arg (24 bits)
    } ring[kRing];            // 0xbc
    /* One slot per sender, because the three fields above are overwritten by whichever sender went
     * last: with one sender stuck, the other can replace its id. slot[0] is the acquisition thread
     * -- grid 0x214/0x215, cliff measurements 0x216, AND the per-cycle health 0x217 that
     * on_cycle_complete sends; slot[1] is the system work queue, which sends only the heartbeat
     * 0x217. So a 0x217 in slot[0] is a cycle health frame and in slot[1] a heartbeat. */
    struct {
        uint32_t active;   // 1 between begin and end
        uint32_t id;       // CAN id of this sender's current or last send
        uint32_t begin_ms; // when it began
        uint32_t rc;       // rc of this sender's last completed send
        uint32_t begin;    // sends begun by this sender
        uint32_t end;      // sends returned to this sender
    } slot[2];                // 0x1bc
    uint32_t arm_blockers;    // 0x1ec what the last refused `arm` was missing (0 = none refused)
    uint32_t magic_end;       // 0x1f0
};

/* Ring event codes. */
enum event : uint32_t {
    ev_boot = 1,
    ev_armed = 2,
    ev_open_begin = 3,  // arg: index
    ev_open_end = 4,    // arg: index << 16 | (rc & 0xffff)
    ev_withheld = 5,    // arg: reason bits
    ev_fatal = 6,       // arg: reason
    ev_send_fail = 7,   // arg: can id << 8 | (rc & 0xff)
    ev_first_fresh = 8, // arg: index
};

/* THE WATCHDOG DECISION, as a pure function so it can be tested on a host.
 *
 * "Begun and not ended" rather than "has not happened lately": acquisition is legitimately idle for
 * seconds while commissioning holds the chain, and a rule about recency would reset a board that is
 * behaving exactly as asked. A cycle, a send or a health item that has STARTED and not finished for
 * longer than its bound is not idle, it is stuck. The zcan loop is the exception: it never idles, so
 * for it recency is the right question.
 *
 * ARM IS THE BOUNDARY, and it is there because of what the first GATE 0 run showed: after a CAN DFU
 * the SCB boots about 90 s before the robot PC, and until the PC is on the bus to acknowledge a
 * frame every synchronous can_send blocks. Judging threads then is judging the boot, not the image,
 * and it reset a healthy board. So nothing is judged while the image is unarmed; `arm` refuses
 * unless every watched activity has already completed at least once and nothing is in flight; and
 * the moment it succeeds the baseline is retaken and EVERYTHING is watched at once -- including the
 * very first L7 open, the very first grid send, and the first cycle after them.
 *
 * The one thing judged while unarmed is time itself: an image nobody armed hands the board back to
 * the confirmed E3 after kUnarmedRevertMs rather than sitting there forever. */
struct watch_state {
    uint32_t last_acq_end{0}, acq_end_seen_ms{0};
    uint32_t last_send_end{0}, send_end_seen_ms{0};
    uint32_t last_health_end{0}, health_end_seen_ms{0};
    uint32_t last_zcan{0}, zcan_seen_ms{0};
    bool initialised{false};
};

struct watch_input {
    uint32_t now_ms;
    uint32_t acq_begin, acq_end;
    uint32_t send_begin, send_end;
    uint32_t health_begin, health_end;
    uint32_t zcan_loops;
    bool armed{false};
    uint32_t armed_ms{0};
};

/* What `arm` needs to see before it will let the risky path start. */
struct arm_input {
    uint32_t acq_begin, acq_end;
    uint32_t send_begin, send_end;
    uint32_t health_begin, health_end;
    uint32_t zcan_loops;
    bool slot_active[2];
};

/* Measured from the moment `arm` succeeded, not from boot. */
inline constexpr uint32_t kGraceMs{60'000};
/* An image nobody armed gives the board back to E3 after this. Long enough for the post-DFU boot,
 * the PC's return and an unhurried set of pre-arm checks. */
inline constexpr uint32_t kUnarmedRevertMs{600'000};
/* A cycle may contain both ULD downloads after `arm` (about 2 s each at 400 kHz). */
inline constexpr uint32_t kCycleBoundMs{20'000};
/* One CAN frame is 130 us at 1 Mbit/s. Two seconds is four orders of magnitude of slack. */
inline constexpr uint32_t kSendBoundMs{2'000};
inline constexpr uint32_t kHealthBoundMs{3'000};
inline constexpr uint32_t kZcanBoundMs{5'000};

int init_record_at_boot();

/* Returns the reason bits of everything stuck now; 0 means feed. Updates st. */
uint32_t evaluate(watch_state &st, const watch_input &in);

/* Returns the arm_block bits of everything missing; 0 means `arm` may proceed. */
uint32_t arm_blockers(const arm_input &in);

/* Called once per second from the board controller's timer callback; true = feed the IWDG. ISR-safe.
 * Once it has returned false it keeps returning false: a board that hung once is not trusted to
 * have recovered. */
bool feed_allowed();

bool armed();
/* 0 on success, otherwise the arm_block bits that refused it. */
uint32_t arm();

void cycle_begin();
void cycle_end();
void send_begin(uint16_t can_id);
void send_end(int rc);
void health_begin();
void health_end();
void zcan_loop();
/* Mode 1: a grid frame the publisher handed over and this image did not transmit. */
void grid_suppressed();
void grid_sent();

} // namespace lexxhard::tof_diag

#else

#define TOF_DIAG(stmt)

#endif // TOF_DIAG_HANG
