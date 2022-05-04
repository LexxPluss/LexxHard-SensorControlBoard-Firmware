#include <zephyr.h>
#include <logging/log.h>
#include <shell/shell.h>
#include "adc_reader.hpp"
#include "tof_controller.hpp"

namespace lexxhard::tof_controller {

LOG_MODULE_REGISTER(tof);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

int info(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "L:%dmV R:%dmV",
                adc_reader::get(adc_reader::DOWNWARD_L),
                adc_reader::get(adc_reader::DOWNWARD_R));
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(info, NULL, "ToF information", info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(tof, &sub, "ToF commands", NULL);

void init()
{
    k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
}

void run(void *p1, void *p2, void *p3)
{
    while (true) {
        msg message;
        message.left = adc_reader::get(adc_reader::DOWNWARD_L);
        message.right = adc_reader::get(adc_reader::DOWNWARD_R);
        while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq);
        k_msleep(20);
    }
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
