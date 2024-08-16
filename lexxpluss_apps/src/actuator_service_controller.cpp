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

#include <optional>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "actuator_controller.hpp"
#include "actuator_service_controller.hpp"

namespace lexxhard::actuator_service_controller {

LOG_MODULE_REGISTER(actuator_service);

char __aligned(4) msgq_request_buffer[8 * sizeof (msg_request)];
char __aligned(4) msgq_response_buffer[8 * sizeof (msg_response)];

class actuators_service_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_request, msgq_request_buffer, sizeof (msg_request), 8);
        k_msgq_init(&msgq_response, msgq_response_buffer, sizeof (msg_response), 8);

        return 0;
    }

    void run() {
        while (true) {
            struct msg_request msg_req;
            if (k_msgq_get(&msgq_request, &msg_req, K_NO_WAIT) == 0) {
                if(auto const ret(handle_request(msg_req)); ret.has_value()) {
                    struct msg_response msg_resp{*ret};
                    while(k_msgq_put(&msgq_response, &msg_resp, K_NO_WAIT) != 0)  {
                        k_msgq_purge(&msgq_response);
                    }
                }
            }
            k_msleep(10);
        }
    }

private:
    std::optional<struct msg_response> handle_request(struct  msg_request const& msg_req)
    {
        if(msg_req.mode == service_mode::INIT_DOWN || msg_req.mode == service_mode::INIT_UP) {
            return do_init_service(msg_req);
        } else if(msg_req.mode == service_mode::LOCATION) {
            return do_location_service(msg_req);
        }

        LOG_ERR("Invalid service mode: %d", static_cast<int8_t>(msg_req.mode));
        return std::nullopt;
    };

    struct msg_response do_init_service(struct msg_request const& msg_req)
    {
        bool const ret{actuator_controller::init_location() == 0};
        return {
            .mode = msg_req.mode,
            .success = ret,
            .left = {.detail = 0},
            .center = {.detail = 0},
            .right = {.detail = 0},
            .counter = msg_req.counter
        };
    }

    struct msg_response do_location_service(struct msg_request const& msg_req)
    {
        uint8_t const location[3]{
            msg_req.center.location,
            msg_req.left.location,
            msg_req.right.location
        };
        uint8_t const power[3]{
            msg_req.center.power,
            msg_req.left.power,
            msg_req.right.power
        };
        uint8_t detail[3]{0, 0, 0};
        bool const ret{actuator_controller::to_location(location, power, detail) == 0};

        return {
            .mode = msg_req.mode,
            .success = ret,
            .left = {.detail = detail[1]},
            .center = {.detail = detail[0]},
            .right = {.detail = detail[2]},
            .counter = msg_req.counter
        };
    }
} impl;

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
k_msgq msgq_request, msgq_response;
}