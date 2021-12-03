#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "rosdiagnostic.hpp"
#include "uss_controller.hpp"

k_msgq msgq_uss2ros;

namespace {

LOG_MODULE_REGISTER(uss);

char __aligned(4) msgq_uss2ros_buffer[8 * sizeof (msg_uss2ros)];

class uss_fetcher {
public:
    int init(const char *label0, const char *label1) {
        dev[0] = device_get_binding(label0);
        if (dev[0] == nullptr)
            return -1;
        if (label1 != nullptr) {
            dev[1] = device_get_binding(label1);
            if (dev[1] == nullptr)
                return -1;
        }
        return 0;
    }
    void get_distance(uint32_t (&distance)[2]) const {
        distance[0] = this->distance[0];
        distance[1] = this->distance[1];
    }
    static void runner(void *p1, void *p2, void *p3) {
        uss_fetcher *self{static_cast<uss_fetcher*>(p1)};
        self->run();
        self->run_error();
    }
    k_thread thread;
private:
    void run() {
        if (!device_is_ready(dev[0]))
            return;
        while (true) {
            if (sensor_sample_fetch_chan(dev[0], SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev[0], SENSOR_CHAN_DISTANCE, &v);
                distance[0] = v.val1 * 1000 + v.val2 / 1000;
            }
            if (dev[1] != nullptr) {
                if (sensor_sample_fetch_chan(dev[1], SENSOR_CHAN_ALL) == 0) {
                    sensor_value v;
                    sensor_channel_get(dev[1], SENSOR_CHAN_DISTANCE, &v);
                    distance[1] = v.val1 * 1000 + v.val2 / 1000;
                }
            }
            k_msleep(1);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "uss", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
        }
    }
    const device *dev[2]{nullptr, nullptr};
    uint32_t distance[2]{0, 0};
} fetcher[4];

K_THREAD_STACK_DEFINE(fetcher_stack_0, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_1, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_2, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_3, 2048);

#define RUN(x) \
    k_thread_create(&fetcher[x].thread, fetcher_stack_##x, K_THREAD_STACK_SIZEOF(fetcher_stack_##x), \
                    &uss_fetcher::runner, &fetcher[x], nullptr, nullptr, 3, K_FP_REGS, K_NO_WAIT);

class uss_controller_impl {
public:
    int init() const {
        k_msgq_init(&msgq_uss2ros, msgq_uss2ros_buffer, sizeof (msg_uss2ros), 8);
        fetcher[0].init("MB1604_0", "MB1604_1");
        fetcher[1].init("MB1604_2", nullptr);
        fetcher[2].init("MB1604_3", nullptr);
        fetcher[3].init("MB1604_4", nullptr);
        return 0;
    }
    void run() const {
        RUN(0);
        RUN(1);
        RUN(2);
        RUN(3);
        while (true) {
            msg_uss2ros message;
            uint32_t distance[2];
            fetcher[0].get_distance(distance);
            message.front_left = distance[0];
            message.front_right = distance[1];
            fetcher[1].get_distance(distance);
            message.left = distance[0];
            fetcher[2].get_distance(distance);
            message.right = distance[0];
            fetcher[3].get_distance(distance);
            message.back = distance[0];
            while (k_msgq_put(&msgq_uss2ros, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_uss2ros);
            k_msleep(100);
        }
    }
    void info(const shell *shell) const {
        uint32_t front[2], left[2], right[2], back[2];
        fetcher[0].get_distance(front);
        fetcher[1].get_distance(left);
        fetcher[2].get_distance(right);
        fetcher[3].get_distance(back);
        shell_print(shell, "FL:%umm FR:%umm L:%umm R:%umm B:%umm\n",
                    front[0], front[1],
                    left[0], right[0], back[0]);
    }
} impl;

static int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uss,
    SHELL_CMD(info, NULL, "USS information", cmd_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(uss, &sub_uss, "USS commands", NULL);

}

void uss_controller::init()
{
    impl.init();
}

void uss_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread uss_controller::thread;

// vim: set expandtab shiftwidth=4:
