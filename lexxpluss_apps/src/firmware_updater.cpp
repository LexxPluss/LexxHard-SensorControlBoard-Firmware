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

#include <zephyr.h>
#include <devicetree.h>
#include <storage/flash_map.h>
#include <sys/reboot.h>
#include "firmware_updater.hpp"

namespace lexxhard::firmware_updater {

class {
public:
    void init() {
        k_msgq_init(&msgq_data, msgq_data_buffer, sizeof (packet_array), 2);
        k_msgq_init(&msgq_response, msgq_response_buffer, sizeof (response_array), 2);
    }
    void run() {
        while (true) {
            cmd();
            failsafe();
            reboot();
        }
    }
private:
    enum class RESP {
        OK                = 0,
        COMPLETE          = 1,
        COMPLETE_RESET    = 2,
        ERR_PARTITION     = 3,
        ERR_FLASH_AREA    = 4,
        ERR_FLASH_ERASE   = 5,
        ERR_FLASH_PROGRAM = 6,
        ERR_CHECKSUM      = 7,
        ERR_TIMEOUT       = 8,
        ERR_POINTER       = 9,
    };
    enum class CMD {
        START           = 0,
        DATA            = 1,
        RESET           = 2,
        RESET_NO_REBOOT = 3,
    };
    void cmd() {
        if (k_msgq_get(&msgq_data, packet.data, K_MSEC(1000)) == 0) {
            switch (static_cast<CMD>(packet.data[2])) {
            case CMD::START:
                cmd_start(packet.data);
                break;
            case CMD::DATA:
                cmd_data(packet.data);
                break;
            case CMD::RESET:
                cmd_reset(reboot_option_enabled);
                break;
            case CMD::RESET_NO_REBOOT:
                cmd_reset(!reboot_option_enabled);
                break;
            }
        }
    }
    void failsafe() {
        uint32_t current_cycle{k_cycle_get_32()};
        if (fa != nullptr && dfu_start_cycle != 0) {
            uint32_t dfu_ms_elapsed{k_cyc_to_ms_near32(current_cycle - dfu_start_cycle)};
            if (dfu_ms_elapsed > 90000) {
                flash_area_reset();
                respond(RESP::ERR_TIMEOUT);
            }
        }
    }
    void reboot() {
        uint32_t current_cycle{k_cycle_get_32()};
        if (ready_to_reboot && k_cyc_to_ms_near32(current_cycle - dfu_completion_cycle) > 2000) {
            ready_to_reboot = false;
            sys_reboot(SYS_REBOOT_COLD);
        }
    }
    void cmd_start(const uint8_t *data) {
        if (fa != nullptr)
            flash_area_reset();
        dfu_start_cycle = k_cycle_get_32();
        if (!DT_NODE_EXISTS(DT_NODELABEL(image_0_secondary_partition))) {
            respond(RESP::ERR_PARTITION);
            return;
        }
        if (flash_area_open(FLASH_AREA_ID(image_1), &fa) != 0) {
            respond(RESP::ERR_FLASH_AREA);
            return;
        }
        static constexpr uint32_t LENGTH{0x00040000};
        for (uint32_t current_offset{0}; current_offset < LENGTH; current_offset += 4) {
            uint32_t buf{0};
            flash_area_read(fa, current_offset, &buf, sizeof buf);
            if (buf != 0xffffffff) {
                if (flash_area_erase(fa, 0, LENGTH) != 0) {
                    flash_area_reset();
                    respond(RESP::ERR_FLASH_ERASE);
                    return;
                }
            }
        }
        cmd_data(data);
    }
    void cmd_data(const uint8_t *data) {
        if (fa == nullptr) {
            flash_area_reset();
            respond(RESP::ERR_POINTER);
            return;
        }
        uint8_t checksum{0x0};
        for (int i{3}; i < 259; i++)
            checksum += data[i];
        if (checksum != data[259]) {
            flash_area_reset();
            respond(RESP::ERR_CHECKSUM);
            return;
        }
        response.data[1] = data[0] + (data[1] << 8);
        uint32_t current_offset{static_cast<uint32_t>(response.data[1]) * 256U};
        if (flash_area_write(fa, current_offset, &data[3], 256) != 0) {
            flash_area_reset();
            respond(RESP::ERR_FLASH_PROGRAM);
            return;
        }
        respond(RESP::OK);
    }
    void cmd_reset(bool reboot_option_enabled) {
        if (fa == nullptr) {
            respond(RESP::ERR_POINTER);
            return;
        }
        flash_area_reset();
        dfu_completion_cycle = k_cycle_get_32();
        ready_to_reboot = reboot_option_enabled;
        respond(reboot_option_enabled ? RESP::COMPLETE_RESET : RESP::COMPLETE);
    }
    void flash_area_reset() {
        if (fa != nullptr) {
            flash_area_close(fa);
            fa = nullptr;
        }
    }
    void respond(RESP resp) {
        response.data[0] = static_cast<uint16_t>(resp);
        k_msgq_put(&msgq_response, response.data, K_MSEC(2000));
    }
    char __aligned(4) msgq_data_buffer[2 * sizeof (packet_array)];
    char __aligned(4) msgq_response_buffer[2 * sizeof (response_array)];
    response_array response;
    packet_array packet;
    const struct flash_area *fa{nullptr};
    uint32_t dfu_start_cycle{0};
    uint32_t dfu_completion_cycle{0};
    bool ready_to_reboot{false};
    const bool reboot_option_enabled{true};
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
k_msgq msgq_data, msgq_response;

}

// vim: set expandtab shiftwidth=4:
