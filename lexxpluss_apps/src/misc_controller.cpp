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

#include <device.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "misc_controller.hpp"

namespace lexxhard::misc_controller {

LOG_MODULE_REGISTER(misc);

class impl {
public:
    int init() {
        dev = device_get_binding("I2C_1");
        if (!device_is_ready(dev))
            return -1;
        for (int i{0}; i < TEMPERATURE_NUM; ++i) {
            uint8_t wbuf[1]{0x0b}, rbuf[2];
            if (i2c_write_read(dev, ADDR + i, wbuf, sizeof wbuf, rbuf, sizeof rbuf) == 0 &&
                (rbuf[0] & 0b11111000) == 0b11001000) {
                static constexpr uint8_t initbuf[2]{0x03, 0b10000000};
                i2c_write(dev, initbuf, sizeof initbuf, ADDR + i);
            }
        }
        return 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        while (true) {
            for (int i{0}; i < TEMPERATURE_NUM; ++i) {
                uint8_t wbuf[1]{0x00}, rbuf[2];
                if (i2c_write_read(dev, ADDR + i, wbuf, sizeof wbuf, rbuf, sizeof rbuf) == 0) {
                    int16_t value{static_cast<int16_t>((rbuf[0] << 8) | rbuf[1])};
                    temperature[i] = temperature[i] * 0.5f + value / 128.0f * 0.5f;
                }
            }
            k_msleep(100);
        }
    }
    void info(const shell *shell) const {
        shell_print(shell,
                    "main: %fdeg\n"
                    "actuator: %fdeg %fdeg %fdeg",
                    get_main_board_temp(),
                    get_actuator_board_temp(0),
                    get_actuator_board_temp(1),
                    get_actuator_board_temp(2));
    }
    float get_main_board_temp() const {return temperature[0];}
    float get_actuator_board_temp(int index) const {
        ++index;
        return index > 0 && index < TEMPERATURE_NUM ? temperature[index] : 0.0f;
    }
private:
    const device *dev{nullptr};
    static constexpr int TEMPERATURE_NUM{4};
    float temperature[TEMPERATURE_NUM]{0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr uint8_t ADDR{0b1001000};
} impl;

int info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(info, NULL, "Misc information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(misc, &sub, "Misc commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

float get_main_board_temp()
{
    return impl.get_main_board_temp();
}

float get_actuator_board_temp(int index)
{
    return impl.get_actuator_board_temp(index);
}

k_thread thread;

}

// vim: set expandtab shiftwidth=4:
