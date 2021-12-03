#include <device.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "misc_controller.hpp"
#include "rosdiagnostic.hpp"

namespace {

LOG_MODULE_REGISTER(misc);

class misc_controller_impl {
public:
    int init() {
        dev = device_get_binding("I2C_1");
        if (dev != nullptr) {
            for (int i{0}; i < TEMPERATURE_NUM; ++i) {
                uint8_t wbuf[1]{0x0b}, rbuf[2];
                if (i2c_write_read(dev, ADDR + i, wbuf, sizeof wbuf, rbuf, sizeof rbuf) == 0 &&
                    (rbuf[0] & 0b11111000) == 0b11001000) {
                    static constexpr uint8_t initbuf[2]{0x03, 0b10000000};
                    i2c_write(dev, initbuf, sizeof initbuf, ADDR + i);
                }
            }
        }
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        while (true) {
            for (int i{0}; i < TEMPERATURE_NUM; ++i) {
                uint8_t wbuf[1]{0x00}, rbuf[2];
                if (i2c_write_read(dev, ADDR + i, wbuf, sizeof wbuf, rbuf, sizeof rbuf) == 0) {
                    int16_t value = (rbuf[0] << 8) | rbuf[1];
                    temperature[i] = value / 128.0f;
                }
            }
            k_msleep(100);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "misc", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
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
    float get_actuator_board_temp(int index) const {return temperature[index];}
private:
    const device *dev{nullptr};
    static constexpr int TEMPERATURE_NUM{4};
    float temperature[TEMPERATURE_NUM]{0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr uint8_t ADDR{0b1001000};
} impl;

static int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_misc,
    SHELL_CMD(info, NULL, "Misc information", cmd_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(misc, &sub_misc, "Misc commands", NULL);

}

void misc_controller::init()
{
    impl.init();
}

void misc_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

float misc_controller::get_main_board_temp()
{
    return impl.get_main_board_temp();
}

float misc_controller::get_actuator_board_temp(int index)
{
    return impl.get_actuator_board_temp(index);
}

k_thread misc_controller::thread;

// vim: set expandtab shiftwidth=4:
