#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "uss_controller.hpp"

namespace lexxfirm::uss_controller {

LOG_MODULE_REGISTER(uss);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

class uss_fetcher {
public:
    int init(const char *label) {
        dev = device_get_binding(label);
        if (!device_is_ready(dev))
            return -1;
        return 0;
    }
    uint32_t get_distance() const {
        return distance;
    }
    static void runner(void *p1, void *p2, void *p3) {
        uss_fetcher *self{static_cast<uss_fetcher*>(p1)};
        self->run();
    }
    k_thread thread;
private:
    void run() {
        if (!device_is_ready(dev))
            return;
        while (true) {
            if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &v);
                int32_t value{v.val1 * 1000 + v.val2 / 1000};
                distance = value;
            }
            k_msleep(1);
        }
    }
    const device *dev{nullptr};
    uint32_t distance{0};
} fetcher[5];

K_THREAD_STACK_DEFINE(fetcher_stack_0, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_1, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_2, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_3, 2048);
K_THREAD_STACK_DEFINE(fetcher_stack_4, 2048);

#define RUN(x) \
    k_thread_create(&fetcher[x].thread, fetcher_stack_##x, K_THREAD_STACK_SIZEOF(fetcher_stack_##x), \
                    &uss_fetcher::runner, &fetcher[x], nullptr, nullptr, 3, K_FP_REGS, K_NO_WAIT);

int info(const shell *shell, size_t argc, char **argv)
{
    uint32_t front_l, front_r, left, right, back;
    front_l = fetcher[0].get_distance();
    front_r = fetcher[1].get_distance();
    left = fetcher[2].get_distance();
    right = fetcher[3].get_distance();
    back = fetcher[4].get_distance();
    shell_print(shell, "FL:%umm FR:%umm L:%umm R:%umm B:%umm\n",
                front_l, front_r, left, right, back);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(info, NULL, "USS information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(uss, &sub, "USS commands", NULL);

void init()
{
    k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
    fetcher[0].init("MB1604_0");
    fetcher[1].init("MB1604_1");
    fetcher[2].init("MB1604_2");
    fetcher[3].init("MB1604_3");
    fetcher[4].init("MB1604_4");
}

void run(void *p1, void *p2, void *p3)
{
    RUN(0);
    RUN(1);
    RUN(2);
    RUN(3);
    RUN(4);
    while (true) {
        msg message;
        message.front_left  = fetcher[0].get_distance();
        message.front_right = fetcher[1].get_distance();
        message.left        = fetcher[2].get_distance();
        message.right       = fetcher[3].get_distance();
        message.back        = fetcher[4].get_distance();
        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq);
        k_msleep(100);
    }
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
