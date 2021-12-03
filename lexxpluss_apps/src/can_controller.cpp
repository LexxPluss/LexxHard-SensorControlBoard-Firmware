#include <zephyr.h>
#include <device.h>
#include <drivers/can.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include "led_controller.hpp"
#include "misc_controller.hpp"
#include "can_controller.hpp"
#include "rosdiagnostic.hpp"

k_msgq msgq_bmu2ros;
k_msgq msgq_board2ros;
k_msgq msgq_ros2board;

namespace {

LOG_MODULE_REGISTER(can);

char __aligned(4) msgq_bmu2ros_buffer[8 * sizeof (msg_bmu2ros)];
char __aligned(4) msgq_board2ros_buffer[8 * sizeof (msg_board2ros)];
char __aligned(4) msgq_ros2board_buffer[8 * sizeof (msg_ros2board)];

CAN_DEFINE_MSGQ(msgq_can_bmu, 16);
CAN_DEFINE_MSGQ(msgq_can_board, 4);
CAN_DEFINE_MSGQ(msgq_can_log, 8);

class log_printer {
public:
    void putc(char c) {
        if (c != '\n')
            buffer[index++] = c;
        if (c == '\n' || index >= 80) {
            buffer[index] = '\0';
            LOG_INF("%s", log_strdup(buffer));
            index = 0;
        }
    }
private:
    uint32_t index{0};
    char buffer[128];
};

class can_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_bmu2ros, msgq_bmu2ros_buffer, sizeof (msg_bmu2ros), 8);
        k_msgq_init(&msgq_board2ros, msgq_board2ros_buffer, sizeof (msg_board2ros), 8);
        k_msgq_init(&msgq_ros2board, msgq_ros2board_buffer, sizeof (msg_ros2board), 8);
        dev = device_get_binding("CAN_1");
        if (dev != nullptr)
            can_configure(dev, CAN_NORMAL_MODE, 500000);
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        const device *gpiok = device_get_binding("GPIOK");
        if (gpiok != nullptr)
            gpio_pin_configure(gpiok, 3, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        setup_can_filter();
        int heartbeat_led{1};
        while (true) {
            bool handled{false};
            zcan_frame frame;
            if (k_msgq_get(&msgq_can_bmu, &frame, K_NO_WAIT) == 0) {
                if (handler_bmu(frame)) {
                    while (k_msgq_put(&msgq_bmu2ros, &bmu2ros, K_NO_WAIT) != 0)
                        k_msgq_purge(&msgq_bmu2ros);
                }
                handled = true;
            }
            if (k_msgq_get(&msgq_can_board, &frame, K_NO_WAIT) == 0) {
                handler_board(frame);
                while (k_msgq_put(&msgq_board2ros, &board2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_board2ros);
                handled = true;
            }
            if (k_msgq_get(&msgq_can_log, &frame, K_NO_WAIT) == 0)
                handler_log(frame);
            if (k_msgq_get(&msgq_ros2board, &ros2board, K_NO_WAIT) == 0) {
                prev_cycle_ros = k_cycle_get_32();
                handled = true;
            }
            uint32_t now_cycle{k_cycle_get_32()};
            if (prev_cycle_ros != 0) {
                uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle_ros)};
                heartbeat_timeout = dt_ms > 1000;
            }
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle_send)};
            if (dt_ms > 100) {
                prev_cycle_send = now_cycle;
                send_message();
                if (gpiok != nullptr) {
                    gpio_pin_set(gpiok, 3, heartbeat_led);
                    heartbeat_led = !heartbeat_led;
                }
            }
            if (!handled)
                k_msleep(1);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "can", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
        }
    }
    uint32_t get_rsoc() const {
        return bmu2ros.rsoc;
    }
private:
    void setup_can_filter() const {
        static const zcan_filter filter_bmu{
            .id{0x100},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7c0},
            .rtr_mask{1}
        };
        static const zcan_filter filter_board{
            .id{0x200},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7fc},
            .rtr_mask{1}
        };
        static const zcan_filter filter_log{
            .id{0x300},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{CAN_STD_ID_MASK},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can_bmu, &filter_bmu);
        can_attach_msgq(dev, &msgq_can_board, &filter_board);
        can_attach_msgq(dev, &msgq_can_log, &filter_log);
    }
    bool handler_bmu(zcan_frame &frame) {
        bool result{false};
        if (frame.id == 0x100) {
            bmu2ros.mod_status1 = frame.data[0];
            bmu2ros.bmu_status = frame.data[1];
            bmu2ros.asoc = frame.data[2];
            bmu2ros.rsoc = frame.data[3];
            bmu2ros.soh = frame.data[4];
            bmu2ros.fet_temp = (frame.data[5] << 8) | frame.data[6];
        } else if (frame.id == 0x101) {
            bmu2ros.pack_current = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.charging_current = (frame.data[2] << 8) | frame.data[3];
            bmu2ros.pack_voltage = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.mod_status2 = frame.data[6];
        } else if (frame.id == 0x103) {
            bmu2ros.design_capacity = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.full_charge_capacity = (frame.data[2] << 8) | frame.data[3];
            bmu2ros.remain_capacity = (frame.data[4] << 8) | frame.data[5];
        } else if (frame.id == 0x110) {
            bmu2ros.max_voltage.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.max_voltage.id = frame.data[2];
            bmu2ros.min_voltage.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.min_voltage.id = frame.data[6];
        } else if (frame.id == 0x111) {
            bmu2ros.max_temp.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.max_temp.id = frame.data[2];
            bmu2ros.min_temp.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.min_temp.id = frame.data[6];
        } else if (frame.id == 0x112) {
            bmu2ros.max_current.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.max_current.id = frame.data[2];
            bmu2ros.min_current.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.min_current.id = frame.data[6];
        } else if (frame.id == 0x113) {
            bmu2ros.bmu_fw_ver = frame.data[0];
            bmu2ros.mod_fw_ver = frame.data[1];
            bmu2ros.serial_config = frame.data[2];
            bmu2ros.parallel_config = frame.data[3];
            bmu2ros.bmu_alarm1 = frame.data[4];
            bmu2ros.bmu_alarm2 = frame.data[5];
        } else if (frame.id == 0x120) {
            bmu2ros.min_cell_voltage.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.min_cell_voltage.id = frame.data[2];
            bmu2ros.max_cell_voltage.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.max_cell_voltage.id = frame.data[6];
        } else if (frame.id == 0x130) {
            bmu2ros.manufacturing = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.inspection = (frame.data[2] << 8) | frame.data[3];
            bmu2ros.serial = (frame.data[4] << 8) | frame.data[5];
            result = true;
        }
        return result;
    }
    void handler_board(zcan_frame &frame) {
        if (frame.id == 0x200) {
            board2ros.bumper_switch[0] = (frame.data[0] & 0b00001000) != 0;
            board2ros.bumper_switch[1] = (frame.data[0] & 0b00010000) != 0;
            board2ros.emergency_switch[0] = (frame.data[0] & 0b00000010) != 0;
            board2ros.emergency_switch[1] = (frame.data[0] & 0b00000100) != 0;
            board2ros.power_switch = (frame.data[0] & 0b00000001) != 0;
            board2ros.wait_shutdown = (frame.data[1] & 0b10000000) != 0;
            board2ros.auto_charging = (frame.data[1] & 0b00000010) != 0;
            board2ros.manual_charging = (frame.data[1] & 0b00000001) != 0;
            board2ros.c_fet = (frame.data[2] & 0b00010000) != 0;
            board2ros.d_fet = (frame.data[2] & 0b00100000) != 0;
            board2ros.p_dsg = (frame.data[2] & 0b01000000) != 0;
            board2ros.v5_fail = (frame.data[2] & 0b00000001) != 0;
            board2ros.v16_fail = (frame.data[2] & 0b00000010) != 0;
            board2ros.wheel_disable[0] = (frame.data[3] & 0b00000001) != 0;
            board2ros.wheel_disable[1] = (frame.data[3] & 0b00000010) != 0;
            board2ros.fan_duty = frame.data[4];
            board2ros.charge_connector_temp[0] = frame.data[5];
            board2ros.charge_connector_temp[1] = frame.data[6];
            board2ros.power_board_temp = frame.data[7];
            board2ros.main_board_temp = misc_controller::get_main_board_temp();
            for (auto i{0}; i < 3; ++i)
                board2ros.actuator_board_temp[i] = misc_controller::get_actuator_board_temp(i);
        } else if (frame.id == 0x202) {
            if (frame.data[0] == 1) {
                msg_ros2led message{msg_ros2led::CHARGE_LEVEL, 2000};
                while (k_msgq_put(&msgq_ros2led, &message, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_ros2led);
            }
        }
    }
    void handler_log(zcan_frame &frame) {
        for (uint32_t i{0}; i < frame.dlc; ++i) {
            uint8_t data{frame.data[i]};
            if (data == 0)
                break;
            log.putc(data);
        }
    }
    void send_message() const {
        zcan_frame frame{
            .id{0x201},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .dlc{3},
            .data{ros2board.emergency_stop, ros2board.power_off, heartbeat_timeout}
        };
        can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
    }
    msg_bmu2ros bmu2ros{0};
    msg_board2ros board2ros{0};
    msg_ros2board ros2board{true, false};
    log_printer log;
    uint32_t prev_cycle_ros{0}, prev_cycle_send{0};
    const device *dev{nullptr};
    bool heartbeat_timeout{true};
} impl;

}

void can_controller::init()
{
    impl.init();
}

void can_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

uint32_t can_controller::get_rsoc()
{
    return impl.get_rsoc();
}

k_thread can_controller::thread;

// vim: set expandtab shiftwidth=4:
