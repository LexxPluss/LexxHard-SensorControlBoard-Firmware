#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "imu_controller.hpp"
#include "rosdiagnostic.hpp"

k_msgq msgq_imu2ros;

namespace {

LOG_MODULE_REGISTER(imu);

char __aligned(4) msgq_imu2ros_buffer[8 * sizeof (msg_imu2ros)];

class imu_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_imu2ros, msgq_imu2ros_buffer, sizeof (msg_imu2ros), 8);
        dev = device_get_binding("ADIS16470");
        if (dev == nullptr)
            return -1;
        for (int i{0}; i < 3; ++i) {
            message.accel[i] = 0;
            message.gyro[i] = 0;
            message.delta_ang[i] = 0;
            message.delta_vel[i] = 0;
        }
        message.temp = 0;
        LOG_INF("IMU controller for LexxPluss board. (%p)", dev);
        return 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        while (true) {
            if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
                message.accel[0] = get_sensor_value_as_float(SENSOR_CHAN_ACCEL_X);
                message.accel[1] = get_sensor_value_as_float(SENSOR_CHAN_ACCEL_Y);
                message.accel[2] = get_sensor_value_as_float(SENSOR_CHAN_ACCEL_Z);
                message.gyro[0] = get_sensor_value_as_float(SENSOR_CHAN_GYRO_X);
                message.gyro[1] = get_sensor_value_as_float(SENSOR_CHAN_GYRO_Y);
                message.gyro[2] = get_sensor_value_as_float(SENSOR_CHAN_GYRO_Z);
                message.temp = get_sensor_value_as_float(SENSOR_CHAN_DIE_TEMP);
                message.delta_ang[0] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START);
                message.delta_ang[1] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 1);
                message.delta_ang[2] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 2);
                message.delta_vel[0] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 3);
                message.delta_vel[1] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 4);
                message.delta_vel[2] = get_sensor_value_as_float(SENSOR_CHAN_PRIV_START, 5);
                while (k_msgq_put(&msgq_imu2ros, &message, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_imu2ros);
            }
            k_msleep(1);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "imu", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
        }
    }
    void info(const shell *shell) {
        msg_imu2ros m{message};
        shell_print(shell,
                    "accel: %f %f %f (m/s/s)\n"
                    "gyro: %f %f %f (deg/s)\n"
                    "vel: %f %f %f (m/s)\n"
                    "ang: %f %f %f (deg)\n"
                    "temp: %fdeg",
                    m.accel[0], m.accel[1], m.accel[2],
                    m.gyro[0], m.gyro[1], m.gyro[2],
                    m.delta_vel[0], m.delta_vel[1], m.delta_vel[2],
                    m.delta_ang[0], m.delta_ang[1], m.delta_ang[2],
                    m.temp);
    }
private:
    float get_sensor_value_as_float(enum sensor_channel chan, uint32_t offset) const {
        chan = static_cast<enum sensor_channel>(static_cast<uint32_t>(chan) + offset);
        return get_sensor_value_as_float(chan);
    }
    float get_sensor_value_as_float(enum sensor_channel chan) const {
        sensor_value v;
        sensor_channel_get(dev, chan, &v);
        return static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
    }
    const device *dev{nullptr};
    msg_imu2ros message;
} impl;

static int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu,
    SHELL_CMD(info, NULL, "IMU information", cmd_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(imu, &sub_imu, "IMU commands", NULL);

}

void imu_controller::init()
{
    impl.init();
}

void imu_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

k_thread imu_controller::thread;

// vim: set expandtab shiftwidth=4:
