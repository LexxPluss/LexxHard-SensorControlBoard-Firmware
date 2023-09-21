/*
 * Copyright (c) 2022-2023, LexxPluss Inc.
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

#define DT_DRV_COMPAT maxbotix_maxbotix

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAXBOTIX, CONFIG_SENSOR_LOG_LEVEL);

enum maxbotix_state {
    MAXBOTIX_STATE_IDLE,
    MAXBOTIX_STATE_RISING_EDGE,
    MAXBOTIX_STATE_FALLING_EDGE,
    MAXBOTIX_STATE_FINISHED,
};

struct maxbotix_callback_data {
    struct gpio_callback gpio_cb;
    struct k_sem semaphore;
    enum maxbotix_state state;
    uint32_t start_time, end_time;
};

struct maxbotix_data {
    struct sensor_value sensor_value;
    struct maxbotix_callback_data cb_data;
};

struct maxbotix_cfg {
    struct gpio_dt_spec gpio_trig, gpio_echo;
};

static void input_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct maxbotix_callback_data *cb_data = (struct maxbotix_callback_data*)cb;
    switch (cb_data->state) {
    case MAXBOTIX_STATE_RISING_EDGE:
        cb_data->start_time = k_cycle_get_32();
        cb_data->state = MAXBOTIX_STATE_FALLING_EDGE;
        break;
    case MAXBOTIX_STATE_FALLING_EDGE:
        cb_data->end_time = k_cycle_get_32();
        cb_data->state = MAXBOTIX_STATE_FINISHED;
        gpio_remove_callback(dev, cb);
        k_sem_give(&cb_data->semaphore);
        break;
    default:
        gpio_remove_callback(dev, cb);
        cb_data->state = MAXBOTIX_STATE_IDLE;
        break;
    }
}

static int maxbotix_init(const struct device *dev)
{
    struct maxbotix_data *data = dev->data;
    const struct maxbotix_cfg *cfg  = dev->config;
    if (!cfg->gpio_echo.port || !cfg->gpio_echo.port) {
        LOG_ERR("no gpio supplied");
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&cfg->gpio_trig, (GPIO_OUTPUT | cfg->gpio_trig.dt_flags));
    if (err != 0)
        return err;
    err = gpio_pin_configure_dt(&cfg->gpio_echo, (GPIO_INPUT | cfg->gpio_echo.dt_flags));
    if (err != 0)
        return err;
    err = gpio_pin_interrupt_configure_dt(&cfg->gpio_echo, GPIO_INT_EDGE_BOTH);
    if (err != 0)
        return err;
    gpio_init_callback(&data->cb_data.gpio_cb, input_changed, BIT(cfg->gpio_echo.pin));
    err = k_sem_init(&data->cb_data.semaphore, 0, 1);
    if (0 != err)
        return err;
    gpio_pin_set_dt(&cfg->gpio_echo, 0);
    data->sensor_value.val1 = 0;
    data->sensor_value.val2 = 0;
    data->cb_data.state = MAXBOTIX_STATE_IDLE;
    LOG_INF("MaxBotix started.");
    return 0;
}

static int maxbotix_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct maxbotix_data *data = dev->data;
    const struct maxbotix_cfg *cfg  = dev->config;
    if (unlikely((SENSOR_CHAN_ALL != chan) && (SENSOR_CHAN_DISTANCE != chan)))
        return -ENOTSUP;
    gpio_add_callback(cfg->gpio_echo.port, &data->cb_data.gpio_cb);
    data->cb_data.state = MAXBOTIX_STATE_RISING_EDGE;
    gpio_pin_set_dt(&cfg->gpio_trig, 1);
    k_busy_wait(20);
    gpio_pin_set_dt(&cfg->gpio_trig, 0);
    if (k_sem_take(&data->cb_data.semaphore, K_MSEC(200)) || data->cb_data.state != MAXBOTIX_STATE_FINISHED) {
        LOG_DBG("No response from MAXBOTIX");
        gpio_remove_callback(cfg->gpio_echo.port, &data->cb_data.gpio_cb);
        return -EIO;
    }
    int result = 0;
    uint32_t count = data->cb_data.end_time - data->cb_data.start_time;
    count = k_cyc_to_us_near32(count);
    if (count < 20 || count > 5000) {
        data->sensor_value.val1 = 0;
        data->sensor_value.val2 = 0;
        result = -ENODATA;
    } else {
        uint32_t micrometer = count * 1000;
        data->sensor_value.val1 = (micrometer / 1000000);
        data->sensor_value.val2 = (micrometer % 1000000);
    }
    static const uint32_t period_us = 98000, measure_us = 85000, count_max_us = period_us - measure_us;
    if (count < count_max_us) {
        uint32_t sleep_to_next = (period_us - (measure_us + count)) / 1000;
        k_msleep(sleep_to_next);
    }
    return result;
}

static int maxbotix_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    const struct maxbotix_data *data = dev->data;
    switch (chan) {
    case SENSOR_CHAN_DISTANCE:
        val->val1 = data->sensor_value.val1;
        val->val2 = data->sensor_value.val2;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api maxbotix_driver_api = {
    .sample_fetch = maxbotix_sample_fetch,
    .channel_get  = maxbotix_channel_get,
};

#define INST(num) DT_INST(num, maxbotix_maxbotix)

#define MAXBOTIX_INIT(inst)                                                          \
    static struct maxbotix_data maxbotix_driver_##inst;                              \
    static const struct maxbotix_cfg maxbotix_cfg_##inst = {                         \
        .gpio_trig = GPIO_DT_SPEC_INST_GET(inst, trig_gpios),                        \
        .gpio_echo = GPIO_DT_SPEC_INST_GET(inst, echo_gpios)                         \
    };                                                                               \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, maxbotix_init, NULL, &maxbotix_driver_##inst, \
        &maxbotix_cfg_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,              \
        &maxbotix_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAXBOTIX_INIT)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MAXBOTIX driver enabled without any devices"
#endif
