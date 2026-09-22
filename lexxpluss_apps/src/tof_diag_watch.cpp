/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The watchdog decision of the L7 hang-isolation images, on its own so a host suite can test it
 * without a board. See tof_diag_hang.hpp.
 */

#include "tof_diag_hang.hpp"

#if defined(TOF_DIAG_HANG)

namespace lexxhard::tof_diag {

uint32_t evaluate(watch_state &st, const watch_input &in)
{
    if (!st.initialised) {
        st = watch_state{};
        st.last_acq_end = in.acq_end;
        st.acq_end_seen_ms = in.now_ms;
        st.last_send_end = in.send_end;
        st.send_end_seen_ms = in.now_ms;
        st.last_health_end = in.health_end;
        st.health_end_seen_ms = in.now_ms;
        st.last_zcan = in.zcan_loops;
        st.zcan_seen_ms = in.now_ms;
        st.initialised = true;
    }
    /* "Seen" is when an END was last observed to move. */
    if (in.acq_end != st.last_acq_end) {
        st.last_acq_end = in.acq_end;
        st.acq_end_seen_ms = in.now_ms;
    }
    if (in.send_end != st.last_send_end) {
        st.last_send_end = in.send_end;
        st.send_end_seen_ms = in.now_ms;
    }
    if (in.health_end != st.last_health_end) {
        st.last_health_end = in.health_end;
        st.health_end_seen_ms = in.now_ms;
    }
    if (in.zcan_loops != st.last_zcan) {
        st.last_zcan = in.zcan_loops;
        st.zcan_seen_ms = in.now_ms;
    }

    if (in.now_ms < kGraceMs)
        return 0;

    uint32_t why{0};
    /* Begun != ended: something is inside. Stuck if the end count has not moved for the bound. */
    if (in.acq_begin != in.acq_end && in.now_ms - st.acq_end_seen_ms > kCycleBoundMs)
        why |= stuck_cycle;
    if (in.send_begin != in.send_end && in.now_ms - st.send_end_seen_ms > kSendBoundMs)
        why |= stuck_send;
    if (in.health_begin != in.health_end && in.now_ms - st.health_end_seen_ms > kHealthBoundMs)
        why |= stuck_health;
    if (in.now_ms - st.zcan_seen_ms > kZcanBoundMs)
        why |= stuck_zcan;
    return why;
}

} // namespace lexxhard::tof_diag

#endif // TOF_DIAG_HANG
