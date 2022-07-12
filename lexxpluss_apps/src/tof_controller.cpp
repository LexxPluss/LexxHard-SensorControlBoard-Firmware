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
