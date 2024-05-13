/*
 * Copyright (c) 2022-2024, LexxPluss Inc.
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

#define DT_DRV_COMPAT adi_adis16470

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ADIS16470, CONFIG_SENSOR_LOG_LEVEL);

#define ADIS16470_REG_DIAG_STAT     0x02
#define ADIS16470_REG_X_GYRO_LOW    0x04
#define ADIS16470_REG_X_GYRO_OUT    0x06
#define ADIS16470_REG_Y_GYRO_LOW    0x08
#define ADIS16470_REG_Y_GYRO_OUT    0x0a
#define ADIS16470_REG_Z_GYRO_LOW    0x0c
#define ADIS16470_REG_Z_GYRO_OUT    0x0e
#define ADIS16470_REG_X_ACCL_LOW    0x10
#define ADIS16470_REG_X_ACCL_OUT    0x12
#define ADIS16470_REG_Y_ACCL_LOW    0x14
#define ADIS16470_REG_Y_ACCL_OUT    0x16
#define ADIS16470_REG_Z_ACCL_LOW    0x18
#define ADIS16470_REG_Z_ACCL_OUT    0x1a
#define ADIS16470_REG_TEMP_OUT      0x1c
#define ADIS16470_REG_X_DELTANG_LOW 0x24
#define ADIS16470_REG_X_DELTANG_OUT 0x26
#define ADIS16470_REG_Y_DELTANG_LOW 0x28
#define ADIS16470_REG_Y_DELTANG_OUT 0x2a
#define ADIS16470_REG_Z_DELTANG_LOW 0x2c
#define ADIS16470_REG_Z_DELTANG_OUT 0x2e
#define ADIS16470_REG_X_DELTVEL_LOW 0x30
#define ADIS16470_REG_X_DELTVEL_OUT 0x32
#define ADIS16470_REG_Y_DELTVEL_LOW 0x34
#define ADIS16470_REG_Y_DELTVEL_OUT 0x36
#define ADIS16470_REG_Z_DELTVEL_LOW 0x38
#define ADIS16470_REG_Z_DELTVEL_OUT 0x3a
#define ADIS16470_REG_MSC_CTRL      0x60
#define ADIS16470_REG_DEC_RATE      0x64
#define ADIS16470_REG_GLOB_CMD      0x68
#define ADIS16470_REG_PROD_ID       0x72

struct adis16470_cb_data {
    struct gpio_callback gpio_cb;
    struct k_sem sem;
};

struct adis16470_data {
    struct adis16470_cb_data cb_data;
    int64_t gyro[3]; // micro rad/s
    int64_t accl[3]; // micro m/s/s
    int64_t temp;    // micro degc
    int64_t delta_ang[3]; // micro rad
    int64_t delta_vel[3]; // micro m/s
};

struct adis16470_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec gpio_int;
};

static int adis16470_reg_read(const struct spi_dt_spec *bus, uint8_t reg, uint16_t *value)
{
    /*
     * For unknown reasons, received data is stored with an 8-bit offset.
     */
    uint8_t buffer_tx[2] = {reg & 0x7f, 0};
    uint8_t buffer_rx[4] = {0xff, 0xff, 0xff, 0xff};
    struct spi_buf spibuf_tx[2] = {
        {.buf = buffer_tx, .len = 2},
        {.buf = NULL,      .len = 2},
    };
    struct spi_buf spibuf_rx[2] = {
        {.buf = NULL,      .len = 2},
        {.buf = buffer_rx, .len = 4},
    };
    struct spi_buf_set bufset_tx = {.buffers = spibuf_tx, .count = 2};
    struct spi_buf_set bufset_rx = {.buffers = spibuf_rx, .count = 2};
    int status = spi_transceive_dt(bus, &bufset_tx, &bufset_rx);
    if (status) {
        LOG_ERR("SPI transceive error %d", status);
        return -1;
    }
    k_usleep(20);
    *value = (buffer_rx[1] << 8) | buffer_rx[2];
    return 0;
}

static int adis16470_reg_write(const struct spi_dt_spec *bus, uint8_t reg, uint16_t value)
{
    uint8_t buffer0[2] = {0x80 | (reg       & 0x7f),  value       & 0xff};
    uint8_t buffer1[2] = {0x80 | ((reg + 1) & 0x7f), (value >> 8) & 0xff};
    struct spi_buf spibuf[2] = {
        {.buf = buffer0, .len = 2},
        {.buf = buffer1, .len = 2},
    };
    struct spi_buf_set bufset = {.buffers = spibuf, .count = 2};
    int status = spi_write_dt(bus, &bufset);
    if (status) {
        LOG_ERR("SPI write error %d", status);
        return -1;
    }
    k_usleep(20);
    return 0;
}

static int adis16470_reg_read_pair(const struct spi_dt_spec *bus, uint8_t reg[2], uint32_t *value)
{
    uint16_t v[2];
    int status = adis16470_reg_read(bus, reg[0], &v[0]);
    if (status < 0)
        return status;
    status = adis16470_reg_read(bus, reg[1], &v[1]);
    if (status < 0)
        return status;
    *value = ((uint32_t)v[1] << 16) | (uint32_t)v[0];
    return 0;
}

static void adis16470_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct adis16470_cb_data *cb_data = (struct adis16470_cb_data*)cb;
    k_sem_give(&(cb_data->sem));
}

static int adis16470_trigger_init(const struct device *dev)
{
    struct adis16470_data *data = dev->data;
    const struct adis16470_config *cfg = dev->config;
    if (!cfg->gpio_int.port) {
        LOG_ERR("trigger enabled but no interrupt gpio supplied");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->gpio_int.port)) {
        LOG_ERR("trigger gpio device is not ready");
        return -ENODEV;
    }
    k_sem_init(&data->cb_data.sem, 0, 1);
    gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    gpio_init_callback(&data->cb_data.gpio_cb, adis16470_gpio_callback, BIT(cfg->gpio_int.pin));
    int err = gpio_add_callback(cfg->gpio_int.port, &data->cb_data.gpio_cb);
    if (err < 0) {
        LOG_ERR("Failed to add callback (err %d)", err);
        return err;
    }
    return gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_RISING);
}

static int adis16470_software_reset(const struct device *dev)
{
    const struct adis16470_config *cfg = dev->config;
    int result = adis16470_reg_write(&cfg->spi, ADIS16470_REG_GLOB_CMD, 0x0080);
    k_msleep(300);
    return result;
}

static int adis16470_self_test(const struct device *dev)
{
    const struct adis16470_config *cfg = dev->config;
    int result = adis16470_reg_write(&cfg->spi, ADIS16470_REG_GLOB_CMD, 0x0004);
    if (result != 0)
        return result;
    k_msleep(14);
    uint16_t value;
    result = adis16470_reg_read(&cfg->spi, ADIS16470_REG_DIAG_STAT, &value);
    if (result != 0)
        return result;
    if (value != 0) {
        LOG_ERR("DIAG_STAT error %u", value);
        return -1;
    }
    result = adis16470_reg_read(&cfg->spi, ADIS16470_REG_PROD_ID, &value);
    if (result != 0)
        return result;
    if (value != 0x4056) {
        LOG_ERR("PROD_ID error %04x", value);
        return -1;
    }
    return 0;
}

static int adis16470_configure(const struct device *dev)
{
    const struct adis16470_config *cfg = dev->config;
    uint16_t value = 39;
    int result = adis16470_reg_write(&cfg->spi, ADIS16470_REG_DEC_RATE, value);
    if (result != 0)
        return result;
    return 0;
}

static int adis16470_init(const struct device *dev)
{
    k_msleep(300);
    const struct adis16470_config *cfg = dev->config;
    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI device is not ready");
        return -EINVAL;
    }
    if (adis16470_trigger_init(dev) != 0) {
        LOG_ERR("Failed to initialize interrupts");
        return -EINVAL;
    }
    int err = adis16470_software_reset(dev);
    if (err) {
        LOG_ERR("adis16470_software_reset failed, error %d\n", err);
        return -ENODEV;
    }
    err = adis16470_self_test(dev);
    if (err) {
        LOG_ERR("adis16470_self_test failed, error %d\n", err);
        return -ENODEV;
    }
    err = adis16470_configure(dev);
    if (err) {
        LOG_ERR("adis16470_configure failed, error %d\n", err);
        return -ENODEV;
    }
    return 0;
}

static int adis16470_sample_fetch_gyro(const struct device *dev, uint32_t reg, int64_t *value)
{
    const struct adis16470_config *cfg = dev->config;
    uint8_t r[2] = {reg - 2, reg};
    int32_t v;
    int status = adis16470_reg_read_pair(&cfg->spi, r, &v);
    if (status < 0)
        return status;
    *value = (int64_t)v * SENSOR_PI / 180LL / 10LL / 65536LL;
    return 0;
}

static int adis16470_sample_fetch_accel(const struct device *dev, uint32_t reg, int64_t *value)
{
    const struct adis16470_config *cfg = dev->config;
    uint8_t r[2] = {reg - 2, reg};
    int32_t v;
    int status = adis16470_reg_read_pair(&cfg->spi, r, &v);
    if (status < 0)
        return status;
    *value = (int64_t)v * SENSOR_G / 800LL / 65536LL;
    return 0;
}

static int adis16470_sample_fetch_delta_ang(const struct device *dev, uint32_t reg, int64_t *value)
{
    const struct adis16470_config *cfg = dev->config;
    uint8_t r[2] = {reg - 2, reg};
    int32_t v;
    int status = adis16470_reg_read_pair(&cfg->spi, r, &v);
    if (status < 0)
        return status;
    *value = (int64_t)v * SENSOR_PI * 2160LL / 2147483648LL / 180LL;
    return 0;
}

static int adis16470_sample_fetch_delta_vel(const struct device *dev, uint32_t reg, int64_t *value)
{
    const struct adis16470_config *cfg = dev->config;
    uint8_t r[2] = {reg - 2, reg};
    int32_t v;
    int status = adis16470_reg_read_pair(&cfg->spi, r, &v);
    if (status < 0)
        return status;
    *value = (int64_t)v * 400LL * 1000000LL / 2147483648LL;
    return 0;
}

static int adis16470_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct adis16470_data *data = dev->data;
    if (k_sem_take(&data->cb_data.sem, K_MSEC(100)) != 0)
        return -1;
    static const uint8_t gyro_reg[3] = {ADIS16470_REG_X_GYRO_OUT, ADIS16470_REG_Y_GYRO_OUT, ADIS16470_REG_Z_GYRO_OUT};
    static const uint8_t accl_reg[3] = {ADIS16470_REG_X_ACCL_OUT, ADIS16470_REG_Y_ACCL_OUT, ADIS16470_REG_Z_ACCL_OUT};
    static const uint8_t ang_reg[3] = {ADIS16470_REG_X_DELTANG_OUT, ADIS16470_REG_Y_DELTANG_OUT, ADIS16470_REG_Z_DELTANG_OUT};
    static const uint8_t vel_reg[3] = {ADIS16470_REG_X_DELTVEL_OUT, ADIS16470_REG_Y_DELTVEL_OUT, ADIS16470_REG_Z_DELTVEL_OUT};
    for (int i = 0; i < 3; ++i) {
        int status = adis16470_sample_fetch_gyro(dev, gyro_reg[i], &data->gyro[i]);
        if (status == 0)
            status = adis16470_sample_fetch_accel(dev, accl_reg[i], &data->accl[i]);
        if (status == 0)
            status = adis16470_sample_fetch_delta_ang(dev, ang_reg[i], &data->delta_ang[i]);
        if (status == 0)
            status = adis16470_sample_fetch_delta_vel(dev, vel_reg[i], &data->delta_vel[i]);
        if (status < 0)
            return status;
    }
    return 0;
}

static void store_value(struct sensor_value *val, int64_t data)
{
    val->val1 = data / 1000000;
    val->val2 = data % 1000000;
}

static int adis16470_channel_get(const struct device *dev,
        enum sensor_channel chan,
        struct sensor_value *val)
{
    struct adis16470_data *data = dev->data;
    switch (chan) {
    case SENSOR_CHAN_GYRO_X:   store_value(val, data->gyro[0]); break;
    case SENSOR_CHAN_GYRO_Y:   store_value(val, data->gyro[1]); break;
    case SENSOR_CHAN_GYRO_Z:   store_value(val, data->gyro[2]); break;
    case SENSOR_CHAN_ACCEL_X:  store_value(val, data->accl[0]); break;
    case SENSOR_CHAN_ACCEL_Y:  store_value(val, data->accl[1]); break;
    case SENSOR_CHAN_ACCEL_Z:  store_value(val, data->accl[2]); break;
    case SENSOR_CHAN_DIE_TEMP: store_value(val, data->temp);    break;
    default:
        if      (chan == SENSOR_CHAN_PRIV_START)     store_value(val, data->delta_ang[0]);
        else if (chan == SENSOR_CHAN_PRIV_START + 1) store_value(val, data->delta_ang[1]);
        else if (chan == SENSOR_CHAN_PRIV_START + 2) store_value(val, data->delta_ang[2]);
        else if (chan == SENSOR_CHAN_PRIV_START + 3) store_value(val, data->delta_vel[0]);
        else if (chan == SENSOR_CHAN_PRIV_START + 4) store_value(val, data->delta_vel[1]);
        else if (chan == SENSOR_CHAN_PRIV_START + 5) store_value(val, data->delta_vel[2]);
        else                                         return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api adis16470_driver_api = {
    .sample_fetch = adis16470_sample_fetch,
    .channel_get = adis16470_channel_get,
};

#define ADIS16470_SPI_CFG \
    (SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define ADIS16470_INIT(inst) \
    static struct adis16470_data adis16470_driver_##inst; \
    static const struct adis16470_config adis16470_cfg_##inst = { \
        .spi = SPI_DT_SPEC_INST_GET(inst, ADIS16470_SPI_CFG, 0U), \
        .gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), \
    }; \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, adis16470_init, NULL, &adis16470_driver_##inst, \
        &adis16470_cfg_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
        &adis16470_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADIS16470_INIT)

/* vim: set expandtab shiftwidth=4: */
