#pragma once

#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include "ros/node_handle.h"

class rosserial_hardware_zephyr {
public:
    void init(const char *name) {
        ring_buf_init(&ringbuf.rx, sizeof ringbuf.rbuf, ringbuf.rbuf);
        ring_buf_init(&ringbuf.tx, sizeof ringbuf.tbuf, ringbuf.tbuf);
        uart_dev = device_get_binding(name);
        if (uart_dev != nullptr) {
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
        if (uart_dev != nullptr) {
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
                int n{uart_fifo_read(uart_dev, buf, sizeof buf)};
                if (n > 0)
                    ring_buf_put(&ringbuf.rx, buf, n);
            }
            if (uart_irq_tx_ready(uart_dev)) {
                uint32_t n{ring_buf_get(&ringbuf.tx, buf, 1)};
                if (n > 0)
                    uart_fifo_fill(uart_dev, buf, n);
            }
            if (uart_irq_tx_complete(uart_dev))
                uart_irq_tx_disable(uart_dev);
        }
    }
    static void uart_isr_trampoline(const device* dev, void* user_data) {
        rosserial_hardware_zephyr* self{static_cast<rosserial_hardware_zephyr*>(user_data)};
        self->uart_isr();
    }
    struct {
        ring_buf rx, tx;
        uint8_t rbuf[1024], tbuf[1024];
    } ringbuf;
    uint32_t baudrate{57600};
    const device* uart_dev{nullptr};
};

namespace ros {

typedef NodeHandle_<rosserial_hardware_zephyr> NodeHandle;

}

// vim: set expandtab shiftwidth=4:
