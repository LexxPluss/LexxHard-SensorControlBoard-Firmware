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


#include <zephyr.h>
#include <device.h>
#include <drivers/can.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "bmu_controller.hpp"

namespace lexxhard::bmu_controller { 

LOG_MODULE_REGISTER(bmu);

char __aligned(4) msgq_bmu_buffer[8 * sizeof (msg_bmu)];

CAN_DEFINE_MSGQ(msgq_can_bmu, 16);

class bmu_controller_impl {
public:
    int init() {
        //k_msgq_init(&msgq_bmu, msgq_bmu_buffer, sizeof (msg_bmu), 8);
        dev_can_bmu = device_get_binding("CAN_1");
        if (!device_is_ready(dev_can_bmu))
            return -1;
        dev_can_ipc = device_get_binding("CAN_2");
        if (!device_is_ready(dev_can_ipc))
            return -1;
        return 0;
    }

    void run() {
        if (!device_is_ready(dev_can_bmu))
            return;
        if (!device_is_ready(dev_can_ipc))
            return;
        static const zcan_filter filter_bmu{
            .id{0x100},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7c0},
            .rtr_mask{1}
        };
        can_attach_msgq(dev_can_bmu, &msgq_can_bmu, &filter_bmu);

        //setup_can_ipc_frame();

        while (true) {
            zcan_frame frame;
            if (k_msgq_get(&msgq_can_bmu, &frame, K_NO_WAIT) == 0) {
                // if (handler_bmu(frame)) {
                //     while (k_msgq_put(&msgq_bmu, &msg, K_NO_WAIT) != 0)
                //         k_msgq_purge(&msgq_bmu);
                // }
                can_send(dev_can_ipc, &frame, K_MSEC(100), nullptr, nullptr); //path thru from BMU to ipc
            }
            else
            {
                k_msleep(1);
            }
        }
    }
    uint32_t get_rsoc() const {
        return msg.rsoc;
    }

    void bmu_info(const shell *shell) const {
        shell_print(shell,
                    "MOD:0x%02x/%02x BMU:0x%02x\n"
                    "ASOC:%u RSOC:%u SOH:%u\n"
                    "FET:%d Current:%d ChargeCurrent:%u\n"
                    "Voltage:%u Capacity(design):%u Capacity(full):%u Capacity(remain):%u\n"
                    "Max Voltage:%u/%u Min Voltage:%u/%u\n"
                    "Max Temp:%d/%u Min Temp:%d/%u\n"
                    "Max Current:%d/%u Min Current:%d/%u\n"
                    "BMUFW:0x%02x MODFW:0x%02x SER:0x%02x PAR:0x%02x\n"
                    "ALM1:0x%02x ALM2:0x%02x\n"
                    "Max Cell Voltage:%u/%u Min Cell Voltage:%u/%u\n"
                    "Manufacture:%u Inspection:%u Serial:%u\n",
                    msg.mod_status1, msg.mod_status2, msg.bmu_status,
                    msg.asoc, msg.rsoc, msg.soh,
                    msg.fet_temp, msg.pack_current, msg.charging_current,
                    msg.pack_voltage, msg.design_capacity, msg.full_charge_capacity, msg.remain_capacity,
                    msg.max_voltage.value, msg.max_voltage.id, msg.min_voltage.value, msg.min_voltage.id,
                    msg.max_temp.value, msg.max_temp.id, msg.min_temp.value, msg.min_temp.id,
                    msg.max_current.value, msg.max_current.id, msg.min_current.value, msg.min_current.id,
                    msg.bmu_fw_ver, msg.mod_fw_ver, msg.serial_config, msg.parallel_config,
                    msg.bmu_alarm1, msg.bmu_alarm2,
                    msg.max_cell_voltage.value, msg.max_cell_voltage.id, msg.min_cell_voltage.value, msg.min_cell_voltage.id,
                    msg.manufacturing, msg.inspection, msg.serial);
    }

private:
    bool handler_bmu(zcan_frame &frame) {
        bool result{false};
        if (frame.id == 0x100) {
            msg.mod_status1 = frame.data[0];
            msg.bmu_status = frame.data[1];
            msg.asoc = frame.data[2];
            msg.rsoc = frame.data[3];
            msg.soh = frame.data[4];
            msg.fet_temp = (frame.data[5] << 8) | frame.data[6];
        } else if (frame.id == 0x101) {
            msg.pack_current = (frame.data[0] << 8) | frame.data[1];
            msg.charging_current = (frame.data[2] << 8) | frame.data[3];
            msg.pack_voltage = (frame.data[4] << 8) | frame.data[5];
            msg.mod_status2 = frame.data[6];
        } else if (frame.id == 0x103) {
            msg.design_capacity = (frame.data[0] << 8) | frame.data[1];
            msg.full_charge_capacity = (frame.data[2] << 8) | frame.data[3];
            msg.remain_capacity = (frame.data[4] << 8) | frame.data[5];
        } else if (frame.id == 0x110) {
            msg.max_voltage.value = (frame.data[0] << 8) | frame.data[1];
            msg.max_voltage.id = frame.data[2];
            msg.min_voltage.value = (frame.data[4] << 8) | frame.data[5];
            msg.min_voltage.id = frame.data[6];
        } else if (frame.id == 0x111) {
            msg.max_temp.value = (frame.data[0] << 8) | frame.data[1];
            msg.max_temp.id = frame.data[2];
            msg.min_temp.value = (frame.data[4] << 8) | frame.data[5];
            msg.min_temp.id = frame.data[6];
        } else if (frame.id == 0x112) {
            msg.max_current.value = (frame.data[0] << 8) | frame.data[1];
            msg.max_current.id = frame.data[2];
            msg.min_current.value = (frame.data[4] << 8) | frame.data[5];
            msg.min_current.id = frame.data[6];
        } else if (frame.id == 0x113) {
            msg.bmu_fw_ver = frame.data[0];
            msg.mod_fw_ver = frame.data[1];
            msg.serial_config = frame.data[2];
            msg.parallel_config = frame.data[3];
            msg.bmu_alarm1 = frame.data[4];
            msg.bmu_alarm2 = frame.data[5];
        } else if (frame.id == 0x120) {
            msg.min_cell_voltage.value = (frame.data[0] << 8) | frame.data[1];
            msg.min_cell_voltage.id = frame.data[2];
            msg.max_cell_voltage.value = (frame.data[4] << 8) | frame.data[5];
            msg.max_cell_voltage.id = frame.data[6];
        } else if (frame.id == 0x130) {
            msg.manufacturing = (frame.data[0] << 8) | frame.data[1];
            msg.inspection = (frame.data[2] << 8) | frame.data[3];
            msg.serial = (frame.data[4] << 8) | frame.data[5];
            result = true;
        }
        return result;
    }

    msg_bmu msg{0};

    const device *dev_can_bmu{nullptr};
    const device *dev_can_ipc{nullptr};

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
k_msgq msgq_bmu;

}

// vim: set expandtab shiftwidth=4:
