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
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/i2c.h>
#include "tug_encoder_controller.hpp"

namespace lexxhard::tug_encoder_controller { 

LOG_MODULE_REGISTER(tug_encoder);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

class tug_encoder_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);

        dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
        if (!device_is_ready(dev)) {
            LOG_ERR("TUG Encoder device not found");
            return -1;
        }

        return 0;
    }

    void run() {
        if (!device_is_ready(dev)) {
            LOG_ERR("TUG Encoder device not found");
            return;
        }

        while (true) {
            fetch_angle();
            while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq);
            k_msleep(1);
        }
    }

    void tug_encoder_info(const shell *shell) const {
        float const angle_deg = message.angle * 360.0f / 4096.0f;
        shell_print(shell, "Angle: %f[deg]\n", angle_deg);
    }

private:
    void fetch_angle() {
        uint8_t angle_h;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ANGLE_H, &angle_h)) {
            LOG_ERR("Failed to read angle high byte\n");
            return;
        }
        uint8_t angle_l;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ANGLE_L, &angle_l)) {
            LOG_ERR("Failed to read angle low byte\n");
            return;
        }
    
        message.angle = (static_cast<uint16_t>(angle_h) << 8) | angle_l;
    }
    msg message;
    const device *dev{nullptr};

    static constexpr uint8_t AS5600_ADDR{0x36};
    static constexpr uint8_t AS5600_REG_ANGLE_H{0x0E};
    static constexpr uint8_t AS5600_REG_ANGLE_L{0x0F};
} impl;

int tug_encoder_info(const shell *shell, size_t argc, char **argv)
{
    impl.tug_encoder_info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tug_encoder,
    SHELL_CMD(info, NULL, "TUG Encoder information", tug_encoder_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(tug_encoder, &sub_tug_encoder, "TUG Encoder commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
k_msgq msgq;

}