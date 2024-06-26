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

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include "firmware_updater.hpp"

namespace lexxhard::firmware_updater {

LOG_MODULE_REGISTER(dfu);

class {
public:
    void init() {
        k_msgq_init(&msgq_command, msgq_command_buffer, sizeof (command_packet), 2);
        k_msgq_init(&msgq_response, msgq_response_buffer, sizeof (response_packet), 2);
    }
    void run() {
        while (true) {
            handle_command();
            check_timeout();
        }
    }
private:
    enum class CMD {
        START  = 0,
        DATA   = 1,
        END    = 2,
        REBOOT = 3,
    };
    enum class RESP {
        OK    = 0,
        ERROR = 1,
        AGAIN = 2,
    };
    void handle_command() {
        if (command_packet command; k_msgq_get(&msgq_command, &command, K_MSEC(1'000)) == 0) {
            switch (static_cast<CMD>(command.command)) {
            case CMD::START:
                cmd_start(command);
                break;
            case CMD::DATA:
                cmd_data(command);
                break;
            case CMD::END:
                cmd_end(command);
                break;
            case CMD::REBOOT:
                cmd_reboot(command);
                break;
            }
        }
    }
    void check_timeout() {
        if (flash != nullptr && dfu_start_cycle != 0) {
            if (auto elapsed{k_cyc_to_ms_near32(k_cycle_get_32() - dfu_start_cycle)}; elapsed > 90'000) {
                flash_close();
                dfu_start_cycle = 0;
            }
        }
        if (dfu_reboot_cycle != 0) {
            if (auto elapsed{k_cyc_to_ms_near32(k_cycle_get_32() - dfu_reboot_cycle)}; elapsed > 2'000) {
                dfu_reboot_cycle = 0;
                sys_reboot(SYS_REBOOT_COLD);
            }
        }
    }
    void cmd_start(const command_packet &command) {
        LOG_INF("firmware update start");
        if (flash != nullptr)
            flash_close();
        dfu_start_cycle = k_cycle_get_32();
        if (flash_area_open(FIXED_PARTITION_ID(slot1_partition), &flash) != 0) {
            respond(command, RESP::ERROR);
            LOG_ERR("cannot open flash area");
            return;
        }
        static constexpr uint32_t LENGTH{0x00040000};
        for (uint32_t current_offset{0}; current_offset < LENGTH; current_offset += 4) {
            uint32_t data{0};
            flash_area_read(flash, current_offset, &data, sizeof data);
            if (data != 0xffffffff) {
                LOG_INF("need to erase flash area");
                if (flash_area_erase(flash, 0, LENGTH) != 0) {
                    flash_close();
                    respond(command, RESP::ERROR);
                    LOG_ERR("cannot erase flash area");
                    return;
                }
                LOG_INF("erased flash area");
            }
        }
        respond(command, RESP::OK);
    }
    void cmd_data(const command_packet &command) {
        if (flash == nullptr) {
            respond(command, RESP::ERROR);
            return;
        }
        uint32_t address{((command.address[0] << 8u) | command.address[1]) * 4u};
        if (flash_area_write(flash, address, command.data, command.length) != 0) {
            flash_close();
            respond(command, RESP::ERROR);
            LOG_ERR("cannot write to flash");
            return;
        }
        respond(command, RESP::OK);
    }
    void cmd_end(const command_packet &command) {
        if (flash == nullptr) {
            respond(command, RESP::ERROR);
            LOG_WRN("firmware update end without start");
            return;
        }
        LOG_INF("firmware update end");
        flash_close();
        respond(command, RESP::OK);
    }
    void cmd_reboot(const command_packet &command) {
        if (flash != nullptr) {
            flash_close();
            respond(command, RESP::ERROR);
            LOG_ERR("reboot during firmware update");
            return;
        }
        LOG_INF("reboot...");
        dfu_reboot_cycle = k_cycle_get_32();
        respond(command, RESP::OK);
    }
    void flash_close() {
        if (flash != nullptr) {
            flash_area_close(flash);
            flash = nullptr;
        }
    }
    void respond(const command_packet &command, RESP resp) {
        response_packet response;
        response.address[0] = command.address[0];
        response.address[1] = command.address[1];
        response.command = command.command;
        response.response = static_cast<uint8_t>(resp);
        while (k_msgq_put(&msgq_response, &response, K_MSEC(2'000)) != 0)
            k_msgq_purge(&msgq_response);
    }
    char __aligned(4) msgq_command_buffer[4 * sizeof (command_packet)];
    char __aligned(4) msgq_response_buffer[4 * sizeof (response_packet)];
    const struct flash_area *flash{nullptr};
    uint32_t dfu_start_cycle{0}, dfu_reboot_cycle{0};
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
k_msgq msgq_command, msgq_response;

}

// vim: set expandtab shiftwidth=4:
