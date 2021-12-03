#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <sys/ring_buffer.h>
#include "pgv_controller.hpp"
#include "rosdiagnostic.hpp"

k_msgq msgq_pgv2ros;
k_msgq msgq_ros2pgv;

namespace {

LOG_MODULE_REGISTER(pgv);

char __aligned(4) msgq_pgv2ros_buffer[8 * sizeof (msg_pgv2ros)];
char __aligned(4) msgq_ros2pgv_buffer[8 * sizeof (msg_ros2pgv)];

class pgv_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_pgv2ros, msgq_pgv2ros_buffer, sizeof (msg_pgv2ros), 8);
        k_msgq_init(&msgq_ros2pgv, msgq_ros2pgv_buffer, sizeof (msg_ros2pgv), 8);
        ring_buf_init(&rxbuf.rb, sizeof rxbuf.buf, rxbuf.buf);
        ring_buf_init(&txbuf.rb, sizeof txbuf.buf, txbuf.buf);
        dev_485 = device_get_binding("UART_6");
        if (dev_485 != nullptr) {
            uart_config config{
                .baudrate{115200},
                .parity{UART_CFG_PARITY_EVEN},
                .stop_bits{UART_CFG_STOP_BITS_1},
                .data_bits{UART_CFG_DATA_BITS_8},
                .flow_ctrl{UART_CFG_FLOW_CTRL_NONE}
            };
            uart_configure(dev_485, &config);
            uart_irq_rx_disable(dev_485);
            uart_irq_tx_disable(dev_485);
            uart_irq_callback_user_data_set(dev_485, uart_isr_trampoline, this);
            uart_irq_rx_enable(dev_485);
        }
        dev_en = device_get_binding("GPIOG");
        if (dev_en != nullptr) {
            gpio_pin_configure(dev_en, 10, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
            gpio_pin_configure(dev_en, 11, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
            gpio_pin_configure(dev_en, 13, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
            gpio_pin_set(dev_en, 10, 0);
        }
        k_sem_init(&sem, 0, 1);
        return dev_485 == nullptr || dev_en == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev_485) || !device_is_ready(dev_en))
            return;
        for (int i{0}; i < 30; ++i) {
            ring_buf_reset(&rxbuf.rb);
            set_direction_decision(DIR::STRAIGHT);
            if (wait_data(5))
                break;
            uint8_t buf[8];
            recv(buf, sizeof buf);
        }
        int heartbeat_led{1};
        while (true) {
            gpio_pin_set(dev_en, 13, heartbeat_led);
            heartbeat_led = !heartbeat_led;
            if (get_position(pgv2ros)) {
                while (k_msgq_put(&msgq_pgv2ros, &pgv2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_pgv2ros);
            }
            msg_ros2pgv ros2pgv;
            if (k_msgq_get(&msgq_ros2pgv, &ros2pgv, K_NO_WAIT) == 0) {
                switch (ros2pgv.dir_command) {
                    case 0: set_direction_decision(DIR::NOLANE);   break;
                    case 1: set_direction_decision(DIR::RIGHT);    break;
                    case 2: set_direction_decision(DIR::LEFT);     break;
                    default:
                    case 3: set_direction_decision(DIR::STRAIGHT); break;
                }
                wait_data(5);
                uint8_t buf[8];
                recv(buf, sizeof buf);
            }
            k_msleep(10);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "pgv", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
        }
    }
    void info(const shell *shell) const {
        msg_pgv2ros m{pgv2ros};
        shell_print(shell,
                    "x: %umm %dmm\n"
                    "y: %dmm\n"
                    "ang: %udeg", m.xp, m.xps, m.yps, m.ang);
    }
private:
    enum class DIR {
        NOLANE,
        RIGHT,
        LEFT,
        STRAIGHT
    };
    bool get_position(msg_pgv2ros &data) {
        ring_buf_reset(&rxbuf.rb);
        uint8_t req[2];
        req[0] = 0xc8;
        req[1] = ~req[0];
        send(req, sizeof req);
        wait_data(23);
        uint8_t buf[64];
        int n{recv(buf, sizeof buf)};
        if (n < 23 || !validate(buf + 2, 21))
            return false;
        decode(buf + 2, data);
        return true;
    }
    void set_direction_decision(DIR dir) {
        uint8_t req[2];
        switch (dir) {
        case DIR::NOLANE:   req[0] = 0xe0; break;
        case DIR::RIGHT:    req[0] = 0xe4; break;
        case DIR::LEFT:     req[0] = 0xe8; break;
        case DIR::STRAIGHT: req[0] = 0xec; break;
        }
        req[1] = ~req[0];
        send(req, sizeof req);
    }
    void decode(const uint8_t *buf, msg_pgv2ros &data) const {
        data.f.cc2 =  (buf[ 0] & 0x40) != 0;
        data.addr  =  (buf[ 0] & 0x30) >> 4;
        data.f.cc1 =  (buf[ 0] & 0x08) != 0;
        data.f.wrn =  (buf[ 0] & 0x04) != 0;
        data.f.np  =  (buf[ 0] & 0x02) != 0;
        data.f.err =  (buf[ 0] & 0x01) != 0;
        data.f.tag =  (buf[ 1] & 0x40) != 0;
        data.lane  =  (buf[ 1] & 0x30) >> 4;
        data.f.rp  =  (buf[ 1] & 0x08) != 0;
        data.f.nl  =  (buf[ 1] & 0x04) != 0;
        data.f.ll  =  (buf[ 1] & 0x02) != 0;
        data.f.rl  =  (buf[ 1] & 0x01) != 0;
        data.yps   =  (buf[ 6] & 0x40 ? 0xc000 : 0) |
                     ((buf[ 6] & 0x7f) << 7) |
                      (buf[ 7] & 0x7f);
        data.ang   = ((buf[10] & 0x7f) << 7) |
                      (buf[11] & 0x7f);
        data.wrn   = ((buf[18] & 0x7f) << 7) |
                      (buf[19] & 0x7f);
        if (data.f.tag) { // data matrix tag
            data.xp  = 0;
            data.o1  = 0;
            data.s1  = 0;
            data.cc1 = 0;
            data.o2  = 0;
            data.s2  = 0;
            data.cc2 = 0;
            data.xps =  (buf[ 2] & 0x04 ? 0xff00000000 : 0) |
                       ((buf[ 2] & 0x07) << 21) |
                       ((buf[ 3] & 0x7f) << 14) |
                       ((buf[ 4] & 0x7f) <<  7) |
                        (buf[ 5] & 0x7f);
            data.tag = ((buf[14] & 0x7f) << 21) |
                       ((buf[15] & 0x7f) << 14) |
                       ((buf[16] & 0x7f) <<  7) |
                        (buf[17] & 0x7f);
        } else { // lane tracking
            data.xp  = ((buf[ 2] & 0x07) << 21) |
                       ((buf[ 3] & 0x7f) << 14) |
                       ((buf[ 4] & 0x7f) <<  7) |
                        (buf[ 5] & 0x7f);
            data.o1  =  (buf[14] & 0x60) >> 5;
            data.s1  =  (buf[14] & 0x18) >> 3;
            data.cc1 = ((buf[14] & 0x07) << 7) |
                        (buf[15] & 0x7f);
            data.o2  =  (buf[16] & 0x60) >> 5;
            data.s2  =  (buf[16] & 0x18) >> 3;
            data.cc2 = ((buf[16] & 0x07) << 7) |
                        (buf[17] & 0x7f);
            data.xps = 0;
            data.tag = 0;
        }
    }
    bool validate(const uint8_t *buf, uint32_t length) const {
        uint32_t tail{length - 1};
        uint8_t check{buf[0]};
        for (uint32_t i{1}; i < tail; ++i)
            check ^= buf[i];
        return check == buf[tail];
    }
    uint32_t rb_count(const ring_buf *rb) const {
        return rb->tail - rb->head;
    }
    void send(const uint8_t *buf, uint32_t length) {
        if (dev_485 != nullptr) {
            gpio_pin_set(dev_en, 11, 1);
            k_busy_wait(100);
            while (length > 0) {
                uint32_t n{ring_buf_put(&txbuf.rb, buf, length)};
                uart_irq_tx_enable(dev_485);
                buf += n;
                length -= n;
            }
            k_sem_take(&sem, K_USEC(500));
            k_busy_wait(280); // 180 - 380, At 115200 bps, the transmission of one octet requires 96us.
            gpio_pin_set(dev_en, 11, 0);
        }
    }
    bool wait_data(uint32_t length) const {
        for (int i{0}; i < 100; ++i) {
            if (rb_count(&rxbuf.rb) >= length)
                return true;
            k_msleep(10);
        }
        return false;
    }
    int recv(uint8_t *buf, uint32_t length) {
        return ring_buf_get(&rxbuf.rb, buf, length);
    }
    void uart_isr() {
        while (uart_irq_update(dev_485) && uart_irq_is_pending(dev_485)) {
            uint8_t buf[64];
            if (uart_irq_rx_ready(dev_485)) {
                int n{uart_fifo_read(dev_485, buf, sizeof buf)};
                if (n > 0)
                    ring_buf_put(&rxbuf.rb, buf, n);
            }
            if (uart_irq_tx_ready(dev_485)) {
                uint32_t n{ring_buf_get(&txbuf.rb, buf, 1)};
                if (n > 0)
                    uart_fifo_fill(dev_485, buf, 1);
            }
            if (uart_irq_tx_complete(dev_485)) {
                uart_irq_tx_disable(dev_485);
                k_sem_give(&sem);
            }
        }
    }
    static void uart_isr_trampoline(const device *dev, void *user_data) {
        pgv_controller_impl *self{static_cast<pgv_controller_impl*>(user_data)};
        self->uart_isr();
    }
    struct {
        ring_buf rb;
        uint32_t buf[256 / sizeof (uint32_t)];
    } txbuf, rxbuf;
    const device *dev_485{nullptr}, *dev_en{nullptr};
    msg_pgv2ros pgv2ros;
    k_sem sem;
} impl;

static int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pgv,
    SHELL_CMD(info, NULL, "PGV information", cmd_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pgv, &sub_pgv, "PGV commands", NULL);

}

void pgv_controller::init()
{
    impl.init();
}

void pgv_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

k_thread pgv_controller::thread;

// vim: set expandtab shiftwidth=4:
