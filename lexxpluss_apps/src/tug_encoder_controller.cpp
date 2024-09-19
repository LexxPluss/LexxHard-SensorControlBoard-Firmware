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

#include <optional>
#include <string>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/i2c.h>
#include "tug_encoder_controller.hpp"

namespace {
    std::optional<std::string> active_token{std::nullopt};
}

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

        if (!is_tug_connected()) {
            LOG_INF("TUG Encoder not found");
            return;
        }

        while (true) {
            if (!fetch_encoder_value()) {
                LOG_ERR("Failed to fetch encoder value");
            }

            while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq);
            k_msleep(1);
        }
    }

    void tug_encoder_info(const shell *shell) {
        float const angle_deg{message.angle * 360.0f / 4096.0f};
        int const burn_count{get_burn_count().value_or(-1)};
        int const raw_angle{get_raw_angle().value_or(-1)};
        int const zpos{get_zero_angle().value_or(-1)};
        int const mpos{get_max_angle().value_or(-1)};
        shell_print(shell, "Angle: %f[deg]", static_cast<double>(angle_deg));
        shell_print(shell, "TUG connected: %d", is_tug_connected());
        shell_print(shell, "Magnet detected: %d", is_magnet_detected());
        shell_print(shell, "Raw Angle: %d", raw_angle);
        shell_print(shell, "Burn count: %d", burn_count);
        shell_print(shell, "ZPOS: %d", zpos);
        shell_print(shell, "MPOS: %d", mpos);

        if (burn_count == 0) {
            shell_print(shell, "");
            shell_print(shell, "***********************************************");
            shell_print(shell, " CAUTION!! This tug encoder is not caliblated. ");
            shell_print(shell, "***********************************************");
            shell_print(shell, "");
        }
    }

    bool burn_angle()  {
        if (!is_tug_connected()) {
            LOG_ERR("TUG Encoder not found");
            return false;
        }

        if (!is_magnet_detected()) {
            LOG_ERR("Magnet not detected");
            return false;
        }

        auto const raw_angle{get_raw_angle()};
        if (!raw_angle.has_value()) {
            LOG_ERR("Failed to read raw angle");
            return false;
        }

        if (!write_zero_angle(raw_angle.value())) {
            LOG_ERR("Failed to write angle to AS5600_REG_ZPOS");
            return false;
        }
        k_msleep(3);

        if (!write_max_angle(raw_angle.value())) {
            LOG_ERR("Failed to write angle to AS5600_REG_MPOS");
            return false;
        }
        k_msleep(3);

        if (!write_burn_sequence()) {
            LOG_ERR("Failed to write burn sequence");
            return false;
        }

        return true;
    }

    bool is_tug_connected() {
        if (!is_tug_connected_cache.has_value()) {
            is_tug_connected_cache = detect_tug();
        }

        return is_tug_connected_cache.value();
    }

private:
    bool detect_tug() const {
        for (uint32_t i = 0; i < DETECTION_RETRY_COUNT; ++i) {
            uint8_t buf;
            if(i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ANGLE_H, &buf) == 0) {
                return true;
            }

            k_msleep(100);
        }

        return false;
    }

    bool fetch_encoder_value() {
        uint8_t angle_h;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ANGLE_H, &angle_h)) {
            LOG_WRN("Failed to read angle high byte");
            return false;
        }
        uint8_t angle_l;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ANGLE_L, &angle_l)) {
            LOG_WRN("Failed to read angle low byte");
            return false;
        }

        message.angle = (static_cast<uint16_t>(angle_h) << 8) | angle_l;
        return true;
    }

    bool is_magnet_detected() const{
        uint8_t status;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_STATUS, &status)) {
            LOG_WRN("Failed to read status");
            return false;
        }

        return (status & 0x20) != 0;
    }

    std::optional<uint8_t> get_burn_count() const {
        uint8_t burn_count;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ZMCO, &burn_count)) {
            LOG_WRN("Failed to read ZMCO");
            return std::nullopt;
        }

        return burn_count;
    }

    std::optional<uint16_t> get_zero_angle() const {
        uint8_t angle_h;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ZPOS_H, &angle_h)) {
            LOG_WRN("Failed to read AS5600_REG_ZPOS_H");
            return 0;
        }

        uint8_t angle_l;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_ZPOS_L, &angle_l)) {
            LOG_WRN("Failed to read AS5600_REG_ZPOS_L");
            return 0;
        }

        return (angle_h <<8) | angle_l;
    }

    std::optional<uint16_t> get_max_angle() const {
        uint8_t angle_h;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_MPOS_H, &angle_h)) {
            LOG_WRN("Failed to read AS5600_REG_MPOS_H");
            return 0;
        }

        uint8_t angle_l;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_MPOS_L, &angle_l)) {
            LOG_WRN("Failed to read AS5600_REG_MPOS_L");
            return 0;
        }

        return (angle_h <<8) | angle_l;
    }

    std::optional<uint16_t> get_raw_angle() const {
        uint8_t angle_h;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_RAW_ANGLE_H, &angle_h)) {
            LOG_WRN("Failed to read AS5600_REG_RAW_ANGLE_H");
            return 0;
        }

        uint8_t angle_l;
        if (i2c_reg_read_byte(dev, AS5600_ADDR, AS5600_REG_RAW_ANGLE_L, &angle_l)) {
            LOG_WRN("Failed to read AS5600_REG_RAW_ANGLE_L");
            return 0;
        }

        return (angle_h <<8) | angle_l;
    }

    bool write_zero_angle(uint16_t angle) {
        uint8_t angle_h = (angle >> 8) & 0xFF;
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_ZPOS_H, angle_h)) {
            LOG_WRN("Failed to write angle to AS5600_REG_ZPOS_H");
            return false;
        }
        uint8_t angle_l = angle & 0xFF;
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_ZPOS_L, angle_l)) {
            LOG_WRN("Failed to write angle to AS5600_REG_ZPOS_L");
            return false;
        }

        return true;
    }

    bool write_max_angle(uint16_t angle) {
        uint8_t angle_h = (angle >> 8) & 0xFF;
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_MPOS_H, angle_h)) {
            LOG_WRN("Failed to write angle to AS5600_REG_MPOS_H");
            return false;
        }

        uint8_t angle_l = angle & 0xFF;
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_MPOS_L, angle_l)) {
            LOG_WRN("Failed to write angle to AS5600_REG_MPOS_L");
            return false;
        }

        return true;
    }

    bool write_burn_sequence() {
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_BURN, 0x80)) {
            LOG_WRN("Failed to write 0x01 to AS5600_REG_BURN");
            return false;
        }
        k_msleep(5);

        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_BURN, 0x01)) {
            LOG_WRN("Failed to write 0x01 to AS5600_REG_BURN");
            return false;
        }
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_BURN, 0x11)) {
            LOG_WRN("Failed to write 0x11 to AS5600_REG_BURN");
            return false;
        }
        if (i2c_reg_write_byte(dev, AS5600_ADDR, AS5600_REG_BURN, 0x10)) {
            LOG_WRN("Failed to write 0x10 to AS5600_REG_BURN");
            return false;
        }
        k_msleep(5);

        return true;
    }

    msg message;
    const device *dev{nullptr};
    std::optional<bool> is_tug_connected_cache{false};

    static constexpr uint8_t AS5600_ADDR{0x36};
    static constexpr uint8_t AS5600_REG_ZMCO{0x00};
    static constexpr uint8_t AS5600_REG_ZPOS_H{0x01};
    static constexpr uint8_t AS5600_REG_ZPOS_L{0x02};
    static constexpr uint8_t AS5600_REG_MPOS_H{0x03};
    static constexpr uint8_t AS5600_REG_MPOS_L{0x04};
    static constexpr uint8_t AS5600_REG_STATUS{0x0B};
    static constexpr uint8_t AS5600_REG_RAW_ANGLE_H{0x0C};
    static constexpr uint8_t AS5600_REG_RAW_ANGLE_L{0x0D};
    static constexpr uint8_t AS5600_REG_ANGLE_H{0x0E};
    static constexpr uint8_t AS5600_REG_ANGLE_L{0x0F};
    static constexpr uint8_t AS5600_REG_BURN{0xFF};
    static constexpr uint32_t DETECTION_RETRY_COUNT{10};
} impl;

int tug_encoder_info(const shell *shell, size_t argc, char **argv)
{
    impl.tug_encoder_info(shell);
    return 0;
}

int tug_encoder_token(const shell *shell, size_t argc, char **argv)
{
    static constexpr const char* hex_ascii_table{"0123456789abcdef"};

    // generate token by current cycle of lower 16 bits
    uint32_t const cur_cycle{k_cycle_get_32()};
    active_token.emplace({
        hex_ascii_table[(cur_cycle >> 0) & 0xF],
        hex_ascii_table[(cur_cycle >> 4) & 0xF],
        hex_ascii_table[(cur_cycle >> 8) & 0xF],
        hex_ascii_table[(cur_cycle >> 12) & 0xF],
    });
    shell_print(shell, "Token: %s", active_token.value().c_str());
    return 0;
}

int tug_encoder_burn(const shell *shell, size_t argc, char **argv)
{
    if (!active_token.has_value()) {
        shell_print(shell, "No token available");
        return 0;
    }

    if (argc != 2) {
        shell_print(shell, "Usage: burn <token>");
        return 0;
    }

    if (active_token.value() != argv[1]) {
        shell_print(shell, "Invalid token");
        return 0;
    }
    active_token.reset();
    shell_print(shell, "Accept valid token and invalidate it.");

    auto const ret{impl.burn_angle()};
    if (!ret) {
        shell_print(shell, "Failed to burn angle");
        return 0;
    }

    shell_print(shell, "Completed to burn angle");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tug_encoder,
    SHELL_CMD(info, NULL, "TUG Encoder information", tug_encoder_info),
    SHELL_CMD(token, NULL, "Generate new token for burning angle", tug_encoder_token),
    SHELL_CMD(burn, NULL, "Burn angle", tug_encoder_burn),
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

bool is_tug_connected() {
    return impl.is_tug_connected();
}

k_thread thread;
k_msgq msgq;

}

