/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A hang-isolation mode is two independent choices -- read the real L7, and put grid frames on
 * CAN -- and this pins the truth table against the production header.
 *
 * It exists because of what mode 3 is. Mode 1 ran thirty minutes armed on dasher2 without wedging
 * and proved the firmware can read the real L7; it then dropped every grid frame at the last step,
 * so no real L7 distance has ever left the board. Mode 3 removes that one drop. The risk in adding
 * it is not that mode 3 is wrong -- the hardware run will answer that -- but that adding it quietly
 * changed one of the two modes whose hardware results are already being relied on. So every mode is
 * built here, not only the new one.
 */

#include <zephyr/ztest.h>

#include <stdio.h>
#include <string.h>

#include "tof_diag_hang.hpp"

namespace
{

/* The axes, as the production header defines them. Written out rather than derived, because a
 * table that recomputes the thing it checks agrees with any mistake in it. */
#if TOF_DIAG_HANG == 1
constexpr bool kExpectRealL7{true};
constexpr bool kExpectSendGrid{false};
#elif TOF_DIAG_HANG == 2
constexpr bool kExpectRealL7{false};
constexpr bool kExpectSendGrid{true};
#elif TOF_DIAG_HANG == 3
constexpr bool kExpectRealL7{true};
constexpr bool kExpectSendGrid{true};
#else
#error "this test covers modes 1, 2 and 3"
#endif

bool read_source(char *buf, size_t cap)
{
    FILE *f{fopen(TOF_CLIFF_CAN_SOURCE, "rb")};
    if (f == nullptr)
        return false;
    const size_t n{fread(buf, 1, cap - 1, f)};
    fclose(f);
    buf[n] = '\0';
    return n > 0;
}

} // namespace

ZTEST(tof_diag_mode, test_the_two_axes_are_what_this_mode_claims)
{
    zassert_equal(static_cast<bool>(TOF_DIAG_REAL_L7), kExpectRealL7,
                  "mode %d reads the real L7: expected %d", TOF_DIAG_HANG, kExpectRealL7);
    zassert_equal(static_cast<bool>(TOF_DIAG_SEND_GRID), kExpectSendGrid,
                  "mode %d puts grid frames on CAN: expected %d", TOF_DIAG_HANG, kExpectSendGrid);
}

ZTEST(tof_diag_mode, test_only_mode_one_keeps_grid_frames_off_the_bus)
{
    /* The whole reason mode 3 exists, stated as an assertion: it is the mode that reads the real
     * sensors AND lets their frames out. Mode 1 reads them and drops the frames; mode 2 sends
     * frames it made up. */
    const bool real_and_on_the_bus{TOF_DIAG_REAL_L7 && TOF_DIAG_SEND_GRID};
    zassert_equal(real_and_on_the_bus, TOF_DIAG_HANG == 3,
                  "only mode 3 may carry a real L7 distance off the board");
}

ZTEST(tof_diag_mode, test_the_tx_seam_is_keyed_on_the_axis_not_on_a_mode_number)
{
    /* Read as text, because the branch is chosen by the preprocessor in a translation unit that
     * cannot be linked here. If someone re-hardcodes "mode 1" in send_grid, mode 3 silently goes
     * back to dropping every frame and every other assertion in this file still passes -- the
     * macros would be right and the code would ignore them. */
    static char src[262144];
    zassert_true(read_source(src, sizeof src), "cannot read %s", TOF_CLIFF_CAN_SOURCE);

    const char *seam{strstr(src, "int send_grid(")};
    zassert_not_null(seam, "send_grid is gone; this test is now checking nothing");

    const char *guard{strstr(seam, "#if TOF_DIAG_SEND_GRID")};
    zassert_not_null(guard, "send_grid no longer selects its branch with TOF_DIAG_SEND_GRID");

    const char *end{strstr(seam, "\n}")};
    zassert_not_null(end, "cannot find the end of send_grid");
    zassert_true(guard < end, "the TOF_DIAG_SEND_GRID guard is not inside send_grid");

    /* And nothing inside it asks the mode number directly. */
    const char *hard{strstr(seam, "TOF_DIAG_HANG ==")};
    zassert_true(hard == nullptr || hard > end, "send_grid branches on a mode number again");
}

ZTEST_SUITE(tof_diag_mode, NULL, NULL, NULL, NULL, NULL);
