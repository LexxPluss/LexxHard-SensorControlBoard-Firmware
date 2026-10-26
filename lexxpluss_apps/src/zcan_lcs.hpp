/*
 * Copyright (c) 2026, LexxPluss Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <cstring>
#include "lcs_can_packer.hpp"
#include "lcs_controller.hpp"
#include "zcan_bounded_send.hpp"
#include "zcan_poll_budget.hpp"

namespace lexxhard::zcan_lcs {

LOG_MODULE_REGISTER(zcan_lcs);

class zcan_lcs {
public:
    void init()
    {
        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)) {
            LOG_ERR("CAN_2 is not ready");
            return;
        }
    }

    void poll()
    {
        lcs::CanFrame message;
        for (int n{0}; n < zcan_poll_budget::kTxPerPass &&
                      k_msgq_get(&lcs_controller::msgq, &message, K_NO_WAIT) == 0; ++n)
        {
            can_frame frame{
                .id = message.id,
                .dlc = message.dlc
            };
            std::memcpy(frame.data, message.data.data(), message.data.size());
            zcan_bounded_send::send(dev, &frame, zcan_poll_budget::kMailboxWait);
        }
    }

private:
    const device *dev{nullptr};
};

}

// vim: set expandtab shiftwidth=4:
