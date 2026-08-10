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
        shell_print(shell,
                    "FailStatus1:0x%02x/0x%02x LeaderBMStatus:0x%02x\n"
                    "ASOCmin:%u RSOCmin:%u SOHmin:%u\n"
                    "MaxFETTemp:%d AvgCurrent:%d MaxChgCurrent:%u\n"
                    "BMVoltageMax:%u Capacity(design):%u Capacity(FCCmin):%u Capacity(RCmin):%u FETStatus:0x%02x\n"
                    "Max Voltage:%u/%u Min Voltage:%u/%u\n"
                    "Max Temp:%d/%u Min Temp:%d/%u\n"
                    "Max Current:%d/%u Min Current:%d/%u\n"
                    "FWVer:0x%02x DataVer:0x%02x ConnectedBMNum:0x%02x\n"
                    "LeaderAlarm1:0x%02x LeaderAlarm2:0x%02x FailStatus3:0x%02x\n"
                    "Max Cell Voltage:%u/%u Min Cell Voltage:%u/%u\n"
                    "Manufacture:%u Inspection:%u Serial:%u\n"
                    "AccumulatedCapacity:%u\n",
                    msg.f100.fail_status1, msg.f101.fail_status2, msg.f100.leader_battery_status,
                    msg.f100.asoc_min, msg.f100.rsoc_min, msg.f100.soh_min,
                    msg.f100.max_fet_temp, msg.f101.average_current, msg.f101.max_charging_current,
                    msg.f101.bm_voltage_max, msg.f103.design_capacity, msg.f103.fcc_min, msg.f103.rc_min, msg.f103.fet_status,
                    msg.f110.max_voltage.value, msg.f110.max_voltage.id, msg.f110.min_voltage.value, msg.f110.min_voltage.id,
                    msg.f111.max_temp.value, msg.f111.max_temp.id, msg.f111.min_temp.value, msg.f111.min_temp.id,
                    msg.f112.max_current.value, msg.f112.max_current.id, msg.f112.min_current.value, msg.f112.min_current.id,
                    msg.f113.fw_ver, msg.f113.data_ver, msg.f113.connected_bm_count,
                    msg.f113.leader_alarm1, msg.f113.leader_alarm2, msg.f113.fail_status3,
                    msg.f120.max_cell_voltage.value, msg.f120.max_cell_voltage.id, msg.f120.min_cell_voltage.value, msg.f120.min_cell_voltage.id,
                    msg.f130.manufacturing, msg.f130.inspection, msg.f130.serial,
                    msg.f131.accumulated_capacity);
    }

private:
    void handler_bmu(can_frame &frame) {
        if (frame.id == 0x100) {
            bmu_lipy041::decode_0x100(frame.data, msg.f100);
        } else if (frame.id == 0x101) {
            bmu_lipy041::decode_0x101(frame.data, msg.f101);
        } else if (frame.id == 0x103) {
            bmu_lipy041::decode_0x103(frame.data, msg.f103);
        } else if (frame.id == 0x110) {
            bmu_lipy041::decode_0x110(frame.data, msg.f110);
        } else if (frame.id == 0x111) {
            bmu_lipy041::decode_0x111(frame.data, msg.f111);
        } else if (frame.id == 0x112) {
            bmu_lipy041::decode_0x112(frame.data, msg.f112);
        } else if (frame.id == 0x113) {
            bmu_lipy041::decode_0x113(frame.data, msg.f113);
        } else if (frame.id == 0x120) {
            bmu_lipy041::decode_0x120(frame.data, msg.f120);
        } else if (frame.id == 0x130) {
            bmu_lipy041::decode_0x130(frame.data, msg.f130);
        } else if (frame.id == 0x131) {
            bmu_lipy041::decode_0x131(frame.data, msg.f131);
        }
        return;
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
