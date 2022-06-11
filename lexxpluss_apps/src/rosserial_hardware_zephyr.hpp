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

#pragma once

#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include "ros/node_handle.h"

namespace {

class rosserial_hardware_zephyr {
public:
    void init(const char *name) {
        ring_buf_init(&ringbuf.rx, sizeof ringbuf.rbuf, ringbuf.rbuf);
        ring_buf_init(&ringbuf.tx, sizeof ringbuf.tbuf, ringbuf.tbuf);
        uart_dev = device_get_binding(name);
        if (device_is_ready(uart_dev)) {
            uart_config config{
                .baudrate{baudrate},
                .parity{UART_CFG_PARITY_NONE},
                .stop_bits{UART_CFG_STOP_BITS_1},
                .data_bits{UART_CFG_DATA_BITS_8},
                .flow_ctrl{UART_CFG_FLOW_CTRL_RTS_CTS}
            };
            uart_configure(uart_dev, &config);
            uart_irq_rx_disable(uart_dev);
            uart_irq_tx_disable(uart_dev);
            static const auto uart_isr_trampoline{[](const device* dev, void* user_data){
                rosserial_hardware_zephyr* self{static_cast<rosserial_hardware_zephyr*>(user_data)};
                self->uart_isr();
            }};
            uart_irq_callback_user_data_set(uart_dev, uart_isr_trampoline, this);
            uart_irq_rx_enable(uart_dev);
        }
    }
    void set_baudrate(uint32_t baudrate) {
        this->baudrate = baudrate;
    }
    int read() {
        uint8_t c;
        uint32_t n{ring_buf_get(&ringbuf.rx, &c, sizeof c)};
        return n > 0 ? c : -1;
    }
    void write(uint8_t* data, int length) {
        if (device_is_ready(uart_dev)) {
            while (length > 0) {
                uint32_t n{ring_buf_put(&ringbuf.tx, data, length)};
                uart_irq_tx_enable(uart_dev);
                data += n;
                length -= n;
            }
        }
    }
    unsigned long time() {
        return k_uptime_get_32();
    }
private:
    void uart_isr() {
        while (uart_irq_update(uart_dev) && uart_irq_is_pending(uart_dev)) {
            uint8_t buf[64];
            if (uart_irq_rx_ready(uart_dev)) {
                if (int n{uart_fifo_read(uart_dev, buf, sizeof buf)}; n > 0)
                    ring_buf_put(&ringbuf.rx, buf, n);
            }
            if (uart_irq_tx_ready(uart_dev)) {
                if (uint32_t n{ring_buf_get(&ringbuf.tx, buf, 1)}; n > 0)
                    uart_fifo_fill(uart_dev, buf, n);
            }
            if (uart_irq_tx_complete(uart_dev))
                uart_irq_tx_disable(uart_dev);
        }
    }
    struct {
        ring_buf rx, tx;
        uint8_t rbuf[1024], tbuf[1024];
    } ringbuf;
    uint32_t baudrate{57600};
    const device* uart_dev{nullptr};
};

}

namespace ros {

typedef NodeHandle_<rosserial_hardware_zephyr> NodeHandle;

}

// vim: set expandtab shiftwidth=4:
