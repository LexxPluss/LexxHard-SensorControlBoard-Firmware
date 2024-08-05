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
#include <array>
#include <cstdlib>
#include <optional>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>
#include "gpio_controller.hpp"
#include "common.hpp"

namespace lexxhard::gpio_controller { 

LOG_MODULE_REGISTER(gpio);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];
char __aligned(4) msgq_control_buffer[8 * sizeof (msg_control)];

class gpio_controller_impl {
public:
    static constexpr int32_t POLL_INTERVAL_MS = 100;

    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        k_msgq_init(&msgq_control, msgq_control_buffer, sizeof (msg_control), 8);

        if (!check_devices()) {
            LOG_ERR("gpio_is_ready_dt Failed");
            return -1;
        }

        if(!update_status()) {
            LOG_ERR("Failed to update status");
            return -1;
        }
        return 0;
    }

    void run() {
        if (!check_devices()) {
            LOG_ERR("gpio_is_ready_dt Failed");
            return;
        }

        while (true) {
            if(!update_status()) {
                LOG_ERR("Failed to update status");
                return;
            }

            if (msg_control msg_control; k_msgq_get(&msgq_control, &msg_control, K_NO_WAIT) == 0) {
                handle_msg_control(msg_control);
            }

            msg msg = create_msg();
            while (k_msgq_put(&msgq, &msg, K_NO_WAIT) != 0) {
                k_msgq_purge(&msgq);
            }
            k_msleep(POLL_INTERVAL_MS);
        }
    }

    void info(const shell *shell) const {
        constexpr auto get_status_char = [](bool status) {
            return status ? 'H' : 'L';
        };
        shell_print(shell, "INPUT 0:%c, 1:%c, 2:%c, 3:%c", get_status_char(gpio_in_status[0]), get_status_char(gpio_in_status[1]), get_status_char(gpio_in_status[2]), get_status_char(gpio_in_status[3]));
        shell_print(shell, "OUTPUT 0:%c, 1:%c, 2:%c, 3:%c", get_status_char(gpio_out_status[0]), get_status_char(gpio_out_status[1]), get_status_char(gpio_out_status[2]), get_status_char(gpio_out_status[3]));
    }

    void set_gpio_out(size_t n, bool value) {
        if(gpio_out_devs.size() <= n) {
            LOG_ERR("N is out of range");
            return;
        }

        const gpio_dt_spec *gpio_dev = &gpio_out_devs[n];
        set_gpio(gpio_dev, value);
    }

private:
    void set_gpio(const gpio_dt_spec *gpio_dev, bool value) {
        if (!gpio_is_ready_dt(gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed");
            return;
        }
        gpio_pin_set_dt(gpio_dev, value);
    }

    std::optional<bool> get_gpio(const gpio_dt_spec *gpio_dev) {
        if (!gpio_is_ready_dt(gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed");
            return std::nullopt;
        }
        return gpio_pin_get_dt(gpio_dev);
    }

    bool update_status() {
        for(size_t i = 0; i < 4; i++) {
            auto status = get_gpio(&gpio_out_devs[i]);
            if(!status.has_value()) {
                return false;
            }
            gpio_out_status[i] = status.value();

            status = get_gpio(&gpio_in_devs[i]);
            if(!status.has_value()) {
                return false;
            }
            gpio_in_status[i] = status.value();
        }

        return true;
    }

    void handle_msg_control(const msg_control &msg) {
        set_gpio_out(0, msg.ros_gpio_out_0);
        set_gpio_out(1, msg.ros_gpio_out_1);
        set_gpio_out(2, msg.ros_gpio_out_2);
        set_gpio_out(3, msg.ros_gpio_out_3);
    }

    msg create_msg() const {
        return msg{
            .gpio_in_0 = gpio_in_status[0],
            .gpio_in_1 = gpio_in_status[1],
            .gpio_in_2 = gpio_in_status[2],
            .gpio_in_3 = gpio_in_status[3],
        };
    }

    bool check_devices(){
        for(size_t i = 0; i < 4; i++) {
            if (!gpio_is_ready_dt(&gpio_out_devs[i])) {
                return false;
            }
        }

        for(size_t i = 0; i < 4; i++) {
            if (!gpio_is_ready_dt(&gpio_in_devs[i])) {
                return false;
            }
        }

        return true;
    }

    const std::array<const gpio_dt_spec, 4> gpio_out_devs{{
        GET_GPIO(spare_gpio_6),
        GET_GPIO(spare_gpio_7),
        GET_GPIO(spare_gpio_8),
        GET_GPIO(spare_gpio_9)
    }};
    std::array<bool, 4> gpio_out_status;
    const std::array<const gpio_dt_spec, 4> gpio_in_devs{{
        GET_GPIO(spare_gpio_10),
        GET_GPIO(spare_gpio_11),
        GET_GPIO(spare_gpio_12),
        GET_GPIO(spare_gpio_13)
    }};
    std::array<bool, 4> gpio_in_status;
} impl;

int info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

int set(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "[notice] parameter order [index] [status (0 or 1)]");
    if (argc != 3) {
        shell_error(shell, "Usage: %s %s <index> <status>", argv[-1], argv[0]);
        return 1;
    }

    auto const ind = atoi(argv[1]);
    auto const status = atoi(argv[2]);
    if(status != 0 && status != 1) {
        shell_error(shell, "status must be 0 or 1");
        return 1;
    }

    impl.set_gpio_out(ind, status);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpio,
    SHELL_CMD(info, NULL, "GPIO information", info),
    SHELL_CMD(set, NULL, "GPIO set command", set),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(gpio, &sub_gpio, "GPIO commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
k_msgq msgq, msgq_control;

}
