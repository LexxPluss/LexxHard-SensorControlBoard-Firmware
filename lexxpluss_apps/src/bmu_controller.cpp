/*
 * Copyright (c) 2024, LexxPluss Inc.
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


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "bmu_controller.hpp"
#include "board_controller.hpp"

namespace lexxhard::bmu_controller { 

LOG_MODULE_REGISTER(bmu);

// Generous upper bound for a single format_bmu_info_line() line (longest line is
// well under 100 bytes); this is printed and reused per line, not sized for the
// whole ~12-line message, to keep the shell thread's stack usage small.
constexpr size_t BMU_INFO_LINE_BUFFER_SIZE{160};

char __aligned(4) msgq_rawframe_bmu_buffer[8 * sizeof (can_frame)];

CAN_MSGQ_DEFINE(msgq_can_recv_bmu, 32);

class bmu_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_rawframe_bmu, msgq_rawframe_bmu_buffer, sizeof (can_frame), 8); // For IPC as path through from CAN_1 to CAN_2

        dev_can_bmu = DEVICE_DT_GET(DT_NODELABEL(can1));
        if (!device_is_ready(dev_can_bmu))
            return -1;

        static const can_filter filter_bmu{
            .id{0x100},
            .mask{0x7c0} // This mask ranged from 0x100 to 0x13F
        };

        can_add_rx_filter_msgq(dev_can_bmu, &msgq_can_recv_bmu, &filter_bmu);

        can_set_bitrate(dev_can_bmu, 500000);
        can_set_mode(dev_can_bmu, CAN_MODE_NORMAL);
        can_start(dev_can_bmu);

        return 0;
    }

    void run() {
        if (!device_is_ready(dev_can_bmu)){
            LOG_ERR("CAN_1 is not ready");
            return;
        }
            
        while (true) {
            can_frame frame;
            if (k_msgq_get(&msgq_can_recv_bmu, &frame, K_NO_WAIT) == 0) {
                // -> raw packet to can_bmu
                while (k_msgq_put(&msgq_rawframe_bmu, &frame, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_rawframe_bmu);
                // -> can frame to board_controller
                while (k_msgq_put(&board_controller::msgq_can_bmu_pb, &frame, K_NO_WAIT) != 0)
                    k_msgq_purge(&board_controller::msgq_can_bmu_pb);

                // -> parsed info to show in bmu info
                handler_bmu(frame);
            } else {
                k_msleep(1);
            }
        }
    }
    uint32_t get_rsoc() const {
        return msg.f100.rsoc_min;
    }

    void bmu_info(const shell *shell) const {
        char buf[BMU_INFO_LINE_BUFFER_SIZE];
        for (size_t line = 0; line < bmu_lipy041::BMU_INFO_LINE_COUNT; ++line) {
            bmu_lipy041::format_bmu_info_line(msg, line, buf, sizeof buf);
            shell_print(shell, "%s", buf);
        }
    }

private:
    void handler_bmu(can_frame &frame) {
        if (!bmu_lipy041::decode_frame_bmu_info(frame.id, frame.data, frame.dlc, msg)) {
            LOG_WRN("bmu decode failed: id=0x%03x dlc=%u", frame.id, frame.dlc);
        }
    }

    msg_bmu msg{};
    msg_can_bmu can_msg{0};

    const device *dev_can_bmu{nullptr};

} impl;

int bmu_info(const shell *shell, size_t argc, char **argv)
{
    impl.bmu_info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_bmu,
    SHELL_CMD(info, NULL, "BMU information", bmu_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(bmu, &sub_bmu, "BMU commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

uint32_t get_rsoc()
{
    return impl.get_rsoc();
}

k_thread thread;
k_msgq msgq_parsed_bmu, msgq_rawframe_bmu;

}

// vim: set expandtab shiftwidth=4:
