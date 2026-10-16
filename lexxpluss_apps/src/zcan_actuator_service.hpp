/*
 * Copyright (c) 2022, LexxPluss Inc.
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

#pragma once

#include <algorithm>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "actuator_service_controller.hpp"
#include "zcan_bounded_send.hpp"
#include "zcan_poll_budget.hpp"

#define CAN_ID_ACTUATOR_SERVICE_REQUEST 0x20b // based on CAN ID assignment
#define CAN_ID_ACTUATOR_SERVICE_RESPONSE 0x213 // based on CAN ID assignment
#define CAN_DATALENGTH_ACTUATOR_SERVICE_REQUEST 8
#define CAN_DATALENGTH_ACTUATOR_SERVICE_RESPONSE 8

namespace lexxhard::zcan_actuator_service {

LOG_MODULE_REGISTER(zcan_actuator_service);

char __aligned(4) msgq_can_actuator_service_request_buffer[8 * sizeof (struct can_frame)];
char __aligned(4) msgq_can_actuator_service_response_buffer[8 * sizeof (struct can_frame)];

CAN_MSGQ_DEFINE(msgq_can_actuator_service_request, 16);

class zcan_actuator_service {
public:
    void init() {
        // can device bind
        k_msgq_init(&msgq_can_actuator_service_request, msgq_can_actuator_service_request_buffer, sizeof(struct can_frame), 8);

        dev = DEVICE_DT_GET(DT_NODELABEL(can2));
        if (!device_is_ready(dev)){
            LOG_ERR("CAN_2 is not ready");
            return;
        }

        // setup can filter
        static const can_filter filter_actuator_service_request{
            .id{CAN_ID_ACTUATOR_SERVICE_REQUEST},
            .mask{0x7ff}
        };

        can_add_rx_filter_msgq(dev, &msgq_can_actuator_service_request, &filter_actuator_service_request);
        return;
    }
    void poll() {
        handle_request();
        handle_response();
    }

private:
    void handle_request() {
        struct can_frame can_frame;
        for (int n{0}; n < zcan_poll_budget::kRxPerPass &&
                      k_msgq_get(&msgq_can_actuator_service_request, &can_frame, K_NO_WAIT) == 0; ++n) {
            auto const msg{actuator_service_controller::msg_request::from(can_frame.data)};
            while (k_msgq_put(&actuator_service_controller::msgq_request, &msg, K_NO_WAIT) != 0) {
                k_msgq_purge(&actuator_service_controller::msgq_request);
            }
        }
    }

    /* Held until accepted, not sent once and forgotten: this is a reply to a request the host is
     * waiting on, and the controller produces it exactly once. A reply dropped because the
     * mailboxes were busy leaves the requester waiting forever, so a refused send keeps
     * `pending_response_` for a later pass. Periodic telemetry does the opposite and drops. */
    void handle_response() {
        if (!pending_response_valid_ &&
            k_msgq_get(&actuator_service_controller::msgq_response, &pending_response_,
                       K_NO_WAIT) == 0) {
            pending_response_valid_ = true;
        }
        if (!pending_response_valid_)
            return;
        struct can_frame can_frame{
            .id = CAN_ID_ACTUATOR_SERVICE_RESPONSE,
            .dlc = CAN_DATALENGTH_ACTUATOR_SERVICE_RESPONSE,
        };
        pending_response_.into(can_frame.data);
        if (zcan_bounded_send::send(dev, &can_frame, zcan_poll_budget::kMailboxWait) == 0)
            pending_response_valid_ = false;
    }

    const device *dev{nullptr};
    actuator_service_controller::msg_response pending_response_{};
    bool pending_response_valid_{false};
};

}

