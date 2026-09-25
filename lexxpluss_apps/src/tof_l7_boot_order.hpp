/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The one part of boot whose ORDER is the whole content, kept where a host suite can link it.
 *
 * The L7 recovery pass can only talk to a survivor while the survivor is still enabled. The enable
 * chain is a row of D flip-flops powered from the rail an SCB reset does not drop, so after a
 * software, watchdog or post-DFU reset a survivor is still enabled and still answering at its
 * programmed address at the moment the application starts. The first gpio_pin_configure_dt on the
 * data line ends that: the chain goes to a known state, the L7 goes silent, and it keeps every bit
 * of the state that made recovery necessary while losing the only channel that could clear it.
 *
 * So "recovery happens before the first control-line change" is not a preference about tidiness. It
 * is the difference between a recovery that can run and one that cannot, and it is exactly the kind
 * of constraint that survives in a comment for a while and then quietly stops being true. It lives
 * in a function instead.
 *
 * WHAT IS AND IS NOT FATAL. The two pin configurations are: a chain whose control lines cannot be
 * configured has nothing to enumerate, and that is a real refusal. Nothing on the recovery side is.
 * A bus speed that will not set, a readback that disagrees, a survivor that will not stop -- each is
 * recorded and the boot carries on to bring the chain up, because enumeration owns the verdict on
 * whether the chain is usable and a second opinion from here would be the same mistake this project
 * has already paid for more than once.
 *
 * WHY THE SPEED IS SET AND READ BACK. The recovery traffic is the only traffic on this bus before
 * enumeration, and it goes to a device in an unknown state. It is issued at the slower proof speed,
 * and the speed is read back rather than assumed because a configure that silently did nothing
 * would leave the recovery running at whatever the devicetree left behind -- which is the product
 * speed, chosen for a schedule rather than for robustness. The product speed is restored afterwards
 * so that everything downstream sees the bus the devicetree promised it.
 */

#pragma once

#include <stdint.h>

namespace lexxhard::tof_l7_boot_order {

enum class step : uint8_t {
    none,
    set_recovery_speed,
    read_back_speed,
    recover,
    restore_product_speed,
    configure_data_pin,
    configure_clock_pin,
};

/* Every step the caller supplies. A null step is skipped rather than refused, because an image
 * built without the L7 ULD has no recovery to run and must still reach its pins. */
struct steps {
    int (*set_recovery_speed)(void *ctx){nullptr};
    /* Sets *matches to whether the bus is now at the speed that was asked for. A non-zero return is
     * a failed readback, which is treated the same as a mismatch. */
    int (*read_back_speed)(void *ctx, bool *matches){nullptr};
    int (*recover_survivors)(void *ctx){nullptr};
    int (*restore_product_speed)(void *ctx){nullptr};
    int (*configure_data_pin)(void *ctx){nullptr};
    int (*configure_clock_pin)(void *ctx){nullptr};
    void *ctx{nullptr};
};

struct report {
    /* Non-zero only for a pin that would not configure. Everything before the pins is advisory. */
    int rc{0};
    step failed_at{step::none};
    bool speed_set{false};
    bool speed_readback_ok{false};
    bool recovery_ran{false};
    int recovery_rc{0};
    bool product_speed_restored{false};
    bool pins_configured{false};
};

/* Runs the fixed order and returns what happened. The pins are configured on every path that is not
 * itself a pin failure -- including the paths where recovery was skipped or failed. */
report run(const steps &s);

const char *step_name(step v);

}  // namespace lexxhard::tof_l7_boot_order
