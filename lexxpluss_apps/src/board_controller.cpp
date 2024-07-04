/*
 * Copyright (c) 2024, LexxPluss Inc.
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

#include <zephyr/kernel.h>
#include <cmath>
#include <functional>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include "adc_reader.hpp"
#include "board_controller.hpp"
#include "can_controller.hpp"
#include "common.hpp"
#include "led_controller.hpp"

namespace lexxhard::board_controller {

LOG_MODULE_REGISTER(board);

char __aligned(4) msgq_bmu_pb_buffer[8 * sizeof (can_controller::msg_bmu)];
char __aligned(4) msgq_board_pb_rx_buffer[8 * sizeof (msg_rcv_pb)];
char __aligned(4) msgq_board_pb_tx_buffer[8 * sizeof (can_controller::msg_board)];

class power_switch_handler {  // No pins declared
public:
    power_switch_handler(int thres_data) : thres(thres_data * 2) {
        start_time = k_uptime_get();
    }
    void poll(bool changed) {
        if (changed) {
            activated = ++counter >= thres;
            start_time = k_uptime_get();
        }
        auto elapsed{k_uptime_get() - start_time};
        if (elapsed > 1000) {
            counter = 0;
            activated = false;
        }
        if (activated && (thres == 4)) {
            led_controller::msg msg_led;
            msg_led.pattern = led_controller::msg::CHARGE_LEVEL;
            msg_led.interrupt_ms = 2000;
            while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller::msgq);
        }
    }
    bool is_activated() const {
        return activated;
    }
private:
    int64_t start_time{0};
    int thres{0}, counter{0};
    bool activated{false};
};

#define PWS_LONG_PUSHED_MS 60000
#define PWS_PUSHED_MS 3000
class power_switch { // Variables Implemented
public:
    enum class STATE {
        RELEASED, PUSHED, LONG_PUSHED,
    };
    void poll() {
        gpio_dt_spec gpio_dev = GET_GPIO(ps_sw_in);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        int now{gpio_pin_get_dt(&gpio_dev)};
        if (prev_raw != now) {
            prev_raw = now;
            count = 0;
        } else {
            ++count;
        }
        bool asserted{false};
        if (count > COUNT) {
            count = COUNT;
            asserted = true;
        }
        if (asserted) {
            bool changed{prev != now};
            sw_bat.poll(changed);
            sw_unlock.poll(changed);
            if (changed) {
                prev = now;
                start_time = k_uptime_get();
            } else if (now == 0) {
                auto elapsed{k_uptime_get() - start_time};
                if (elapsed > PWS_LONG_PUSHED_MS) {
                    if (state != STATE::LONG_PUSHED)
                        state = STATE::LONG_PUSHED;
                } else if (elapsed > PWS_PUSHED_MS) {
                    if (state == STATE::RELEASED)
                        state = STATE::PUSHED;
                }
            }
        }
    }
    void reset_state() {
        start_time = k_uptime_get();
        state = STATE::RELEASED;
    }
    STATE get_state() const {return state;}
    bool get_raw_state() {
        gpio_dt_spec gpio_dev = GET_GPIO(ps_sw_in);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return false;
        }
        return gpio_pin_get_dt(&gpio_dev) == 0;
    }
    void set_led(bool enabled) {
        gpio_dt_spec gpio_dev = GET_GPIO(ps_led_out);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        gpio_pin_set_dt(&gpio_dev, enabled ? 1 : 0);
    }
    void toggle_led() {
        set_led(led_en);
        led_en = true ? false : true;
    }
    bool is_activated_battery() const {
        return sw_bat.is_activated();
    }
    bool is_activated_unlock() const {
        return sw_unlock.is_activated();
    }
private:
    power_switch_handler sw_bat{2}, sw_unlock{10};
    int64_t start_time;
    STATE state{STATE::RELEASED};
    uint32_t count{0};
    bool led_en{false};
    int prev{-1}, prev_raw{-1};
    static constexpr uint32_t COUNT{1};
};

#define RS_PUSHED_MS 3000
class resume_switch { // Variables Implemented
public:
    enum class STATE {
        RELEASED, PUSHED
    };
    void poll() {
        gpio_dt_spec gpio_dev = GET_GPIO(resume_sw_in);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        int now{gpio_pin_get_dt(&gpio_dev)};
        if (prev_raw != now) {
            prev_raw = now;
            count = 0;
        } else {
            ++count;
        }
        bool asserted{false};
        if (count > COUNT) {
            count = COUNT;
            asserted = true;
        }
        if (asserted) {
            bool changed{prev != now};
            if (changed) {
                prev = now;
                start_time = k_uptime_get();
                if(now == 1) {
                    state = STATE::RELEASED;
                }
            } else if (now == 0) {
                auto elapsed{k_uptime_get() - start_time};
                if (elapsed > RS_PUSHED_MS) {
                    if (state == STATE::RELEASED)
                        state = STATE::PUSHED;
                }
            }
        }
    }
    void reset_state() {
        start_time = k_uptime_get();
        state = STATE::RELEASED;
    }
    STATE get_state() const {return state;}
    bool get_raw_state() {
        gpio_dt_spec gpio_dev = GET_GPIO(resume_sw_in);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return false;
        }
        return gpio_pin_get_dt(&gpio_dev) == 0;
    }
    void set_led(bool enabled) {
        gpio_dt_spec gpio_dev = GET_GPIO(resume_led_out);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        gpio_pin_set_dt(&gpio_dev, enabled ? 1 : 0);
    }
private:
    int64_t start_time;
    STATE state{STATE::RELEASED};
    uint32_t count{0};
    int prev{-1}, prev_raw{-1};
    static constexpr uint32_t COUNT{1};
};

class key_switch { // Variables Implemented
public:
    enum class STATE {
        LEFT, RIGHT, UNKNOWN
    };
    void poll() {
        gpio_dt_spec gpio_left_dev = GET_GPIO(key_switch_left);
        if (!gpio_is_ready_dt(&gpio_left_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        int now_left{gpio_pin_get_dt(&gpio_left_dev)};

        gpio_dt_spec gpio_right_dev = GET_GPIO(key_switch_right);
        if (!gpio_is_ready_dt(&gpio_right_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        int now_right{gpio_pin_get_dt(&gpio_right_dev)};

        if (prev_left != now_left || prev_right != now_right) {
            prev_left = now_left;
            prev_right = now_right;
            count = 0;
        } else {
            ++count;
        }

        bool asserted{false};
        if (count > COUNT) {
            count = COUNT;
            asserted = true;
        }
        if (asserted) {
            if(now_left == 0 && now_right == 1)
                state = STATE::LEFT;
            else if(now_left == 1 && now_right == 0)
                state = STATE::RIGHT;
            else
                state = STATE::UNKNOWN;
        }
    }
    void reset_state() {
        state = STATE::UNKNOWN;
    }
    STATE get_state() const {return state;}
    bool is_maintenance() const {
        return state != STATE::RIGHT;
    }
private:
    STATE state{STATE::UNKNOWN};
    uint32_t count{0};
    int prev_left{-1}, prev_right{-1};
    static constexpr uint32_t COUNT{1};
};

class bumper_switch { // Variables Implemented
public:
    void poll() {
        if(should_reset) {
            should_reset = false;
            gpio_dt_spec gpio_dev = GET_GPIO(bp_reset);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }

            // reset pulse for the bumper switch
            gpio_pin_set_dt(&gpio_dev, 1);
            k_msleep(5);
            gpio_pin_set_dt(&gpio_dev, 0);
        }

        gpio_dt_spec gpio_dev = GET_GPIO(bp_left);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        asserted_flag = (gpio_pin_get_dt(&gpio_dev) == 0);
    }
    void get_raw_state(bool &left, bool &right) const {
        left = right = asserted_flag;
    }
    bool is_asserted() const {return asserted_flag;}
    void request_reset() {
        should_reset = true;
    }
private:
    bool asserted_flag{false};
    bool should_reset{false};
};

class emergency_switch { // Variables Implemented
public:
    void poll() {
        bool const prev_is_asserted{is_asserted()}; 

        gpio_dt_spec gpio_dev = GET_GPIO(es_left);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        int now{gpio_pin_get_dt(&gpio_dev)};
        if (left_prev != now) {
            left_prev = now;
            left_count = 0;
        } else {
            ++left_count;
        }
        if (left_count > COUNT) {
            left_count = COUNT;
            left_asserted = now == 1;
        }
        gpio_dev = GET_GPIO(es_right);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        now = gpio_pin_get_dt(&gpio_dev);
        if (right_prev != now) {
            right_prev = now;
            right_count = 0;
        } else {
            ++right_count;
        }
        if (right_count > COUNT) {
            right_count = COUNT;
            right_asserted = now == 1;
        }

        // release the emergency switch
        if (prev_is_asserted && !is_asserted()) {
            if (callback) {
                callback();
            }
        }
    }
    void reset_state() {
        left_count = right_count = 0;
        left_asserted = right_asserted = false;
        left_prev = right_prev = -1;
    }
    bool is_asserted() const {return left_asserted || right_asserted;}
    void get_raw_state(bool &left, bool &right) const {
        left = left_asserted;
        right = right_asserted;
    }
    void set_callback(std::function<void ()> cb) {
        callback = cb;
    }
private:
    uint32_t left_count{0}, right_count{0};
    int left_prev{-1}, right_prev{-1};
    bool left_asserted{false}, right_asserted{false};
    std::function<void ()> callback{nullptr};
    static constexpr uint32_t COUNT{5};
};

class wheel_switch { // Variables Implemented
public:
    void set_disable(bool disable) {
        if (disable) {
            gpio_dt_spec gpio_dev = GET_GPIO(wheel_en);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
            left_right_disable = true; 
        } else {
            gpio_dt_spec gpio_dev = GET_GPIO(wheel_en);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 1);
            left_right_disable = false;
        }
    }
    void get_raw_state(bool &left, bool &right) {
        left = left_right_disable;
        right = left_right_disable;
    }
    bool is_enabled() const {return !left_right_disable;}
private:
    bool left_right_disable{false};   //false is enable
};

class manual_charger { // Variables Implemented
public:
    void init() {
        start_time = k_uptime_get();
        setup_first_state();
    }
    void poll() {
        gpio_dt_spec gpio_dev = GET_GPIO(mc_din);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        int now{gpio_pin_get_dt(&gpio_dev)};
        if (prev != now) {
            prev = now;
            start_time = k_uptime_get();
        } else {
            auto elapsed{k_uptime_get() - start_time};
            if (elapsed > 100) {
                plugged = now == 0;
                start_time = k_uptime_get();
            }
        }
    }
    bool is_plugged() {return plugged;}
private:
    void setup_first_state() {
        int plugged_count{0};
        for (int i{0}; i < 10; ++i) {
            gpio_dt_spec gpio_dev = GET_GPIO(mc_din);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            if (gpio_pin_get_dt(&gpio_dev) == 0)
                ++plugged_count;
            k_msleep(5);
        }
        plugged = plugged_count > 5;
    }
    int64_t start_time{0};
    int prev{1};
    bool plugged{false};
};

class serial_message {
public:
    bool decode(uint8_t data) {
        return decode_single(data);
    }
    bool decode(uint8_t *buf, int length) {
        while (length--) {
            if (decode_single(*buf++))
                return true;
        }
        return false;
    }
    uint8_t get_command(uint8_t param[3]) const {
        param[0] = request.detail.param[0];
        param[1] = request.detail.param[1];
        param[2] = request.detail.param[2];
        return request.detail.command;
    }
    static void compose(uint8_t buf[8], uint8_t command, uint8_t *param) {
        buf[0] = 'L'; // L and P creates the header of the communication protocol
        buf[1] = 'P';
        buf[2] = command;
        if (param != nullptr) {
            buf[3] = param[0];
            buf[4] = param[1];
            buf[5] = param[2];
        } else {
            buf[3] = buf[4] = buf[5] = 0;
        }
        uint16_t sum{calc_check_sum(buf, 6)};
        buf[6] = sum;
        buf[7] = sum >> 8; // This confirms that the message have been received correctly on the other end of teh communication by reverting and confirming the sum operation.
    }
    void reset() {state = STATE::HEAD0;}
    static constexpr uint8_t HEARTBEAT{0x01};
    static constexpr uint8_t POWERON{0x02};
    static constexpr uint8_t POWEROFF{0x03};
private:
    bool decode_single(uint8_t data) {
        bool result{false};
        switch (state) {
        case STATE::HEAD0:
            request.raw[0] = data;
            if (data == 'L')
                state = STATE::HEAD1;
            break;
        case STATE::HEAD1:
            request.raw[1] = data;
            if (data == 'P') {
                state = STATE::DATA;
                data_count = 2;
            } else {
                reset();
            }
            break;
        case STATE::DATA:
            request.raw[data_count] = data;
            if (++data_count >= sizeof request.raw) {
                reset();
                uint16_t sum{calc_check_sum(request.raw, 6)};
                if (((sum >> 0) & 0xff) == request.detail.sum[0] &&
                    ((sum >> 8) & 0xff) == request.detail.sum[1])
                    result = true;
            }
            break;
        }
        return result;
    }
    static uint16_t calc_check_sum(const uint8_t *buf, uint32_t length) {
        uint16_t value{0};
        while (length--)
            value += *buf++;
        uint8_t l{static_cast<uint8_t>(255 - value % 256)};
        uint8_t h{static_cast<uint8_t>(~l)};
        return (h << 8) | l;
    }
    union {
        uint8_t raw[8];
        struct {
            uint8_t head[2];
            uint8_t command;
            uint8_t param[3];
            uint8_t sum[2];
        } detail;
    } request;
    uint32_t data_count{0};
    enum class STATE {
        HEAD0, HEAD1, DATA
    } state{STATE::HEAD0};
};

#define AUTO_CHARGE_DOCKED_TIMEOUT_MS 5000
#define CHARGER_HEARTBEAT_TIMEOUT_MS 1000
#define IRDA_DATA_LEN 8
#define IRDA_TX_TIMEOUT_MS 1000

class auto_charger { // Variables Half-Implemented (Not Thermistors ADC)
public:
    void init() {
        k_timer_init(&timer_poll_1s, static_poll_1s_callback, NULL);
        k_timer_user_data_set(&timer_poll_1s, this);
        k_timer_start(&timer_poll_1s, K_MSEC(1000), K_MSEC(1000));

        start_time = k_uptime_get();

        dev = GET_DEV(usart6);

        uart_config uart_cfg = {
            .baudrate = 4800,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
        };
        
        uart_configure(dev, &uart_cfg);

        int ret = uart_irq_callback_user_data_set(dev, static_serial_read_callback, this);

        if (ret != 0) {
            LOG_INF("UART IRQ Error");
            return;
        }
        uart_irq_rx_enable(dev);

        return;
    }
    bool is_docked() const {
        return is_connected() && !is_overheat() && (k_uptime_get() - start_time) < AUTO_CHARGE_DOCKED_TIMEOUT_MS;
    }
    void set_enable(bool enable) {
        gpio_dt_spec gpio_dev = GET_GPIO(v_autocharge);
        if (!gpio_is_ready_dt(&gpio_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        gpio_pin_set_dt(&gpio_dev, enable ? 1 : 0);
    }
    void force_stop() {
        set_enable(false);
        send_heartbeat();
        return;
    }
    bool is_charger_ready() const {
        if(connector_v > (CHARGING_VOLTAGE * 0.9f)){
            LOG_DBG("charger ready voltage:%f.\n", static_cast<double>(connector_v));
            return true;
        }else {
            LOG_DBG("connector_v:%f THRESH:%f\n", static_cast<double>(connector_v), static_cast<double>(CHARGING_VOLTAGE * 0.9f));
            return false;
        }
    }
    void get_connector_temperature(int &positive, int &negative) const {
        positive = CLAMP(static_cast<int>(connector_temp[0]), 0, 255);
        negative = CLAMP(static_cast<int>(connector_temp[1]), 0, 255);
        return;
    }
    uint32_t get_connector_voltage() const {
        int32_t voltage_mv{static_cast<int32_t>(connector_v * 1e+3f)};
        return CLAMP(voltage_mv, 0L, 3300L);
    }
    uint32_t get_connect_check_count() const {return connect_check_count;}
    uint32_t get_heartbeat_delay() const {
        auto seconds{(k_uptime_get() - start_time) / 1000};
        return CLAMP(static_cast<uint32_t>(seconds), 0UL, 255UL);
    }
    
    void poll() {
        uint32_t prev_connect_check_count{connect_check_count};
        connector_v = (float)(adc_reader::get_adc3(adc_reader::CHARGING_VOLTAGE) / 1000.0f);
        if (connector_v > CONNECT_THRES_VOLTAGE) {
            if (++connect_check_count >= CONNECT_THRES_COUNT) {
                connect_check_count = CONNECT_THRES_COUNT;
                if (prev_connect_check_count < CONNECT_THRES_COUNT)
                    LOG_DBG("connected to the charger.\n");
            }
        } else {
            connect_check_count = 0;
            if (prev_connect_check_count >= CONNECT_THRES_COUNT)
                LOG_DBG("disconnected from the charger.\n");
        }

        adc_read();
        return;
    }
    void update_rsoc(uint8_t rsoc) {
        this->rsoc = rsoc;
        return;
    }
private: // Thermistor side starts here.
    static void static_poll_1s_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<auto_charger*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->poll_1s();
        }
        return;
    }
    static void static_serial_read_callback(const struct device *uart_dev, void *user_data) {
        auto* instance = static_cast<auto_charger*>(const_cast<void*>(user_data));
        if (instance) {
            instance->serial_read();
        }
        return;
    }
    void adc_read() { // Change to read the temperature sensor from ADC pin directly. Thermistor side.
        float v_th_pos{(float)(adc_reader::get(adc_reader::THERMISTOR_P) / 1000.0f)};
        float v_th_neg{(float)(adc_reader::get(adc_reader::THERMISTOR_N) / 1000.0f)};
        // LOG_DBG("%f %f", v_th_pos, v_th_neg);
        calculate_temperature(v_th_pos, 0); // Calculate the thermistor PLUS temperature
        calculate_temperature(v_th_neg, 1); // Calculate the thermistor MINUS temperature
        return;
    }
    void calculate_temperature(float adc_voltage, uint8_t sensor) { // Changed version for direct ADC measurements
        adc_voltage = CLAMP(adc_voltage, 0.0f, 3.29999f); // Clamp the value of the adc voltage received
        // see https://lexxpluss.esa.io/posts/459
        static constexpr float R0{3300.0f}, B{3970.0f}, T0{373.0f};
        float Rpu{10000.0f};
        float R{Rpu * adc_voltage / (3.3f - adc_voltage)};
        float T{1.0f / (logf(R / R0) / B + 1.0f / T0)};
        static constexpr float gain{0.02f}; // Low pass filter gain
        connector_temp[sensor] = connector_temp[sensor] * (1.0f - gain) + (T - 273.0f) * gain; // Low pass filter function
        return;
    }
    bool is_connected() const {
        return connect_check_count >= CONNECT_THRES_COUNT;
    }
    bool is_overheat() const {
        return connector_temp[0] > 80.0f || connector_temp[1] > 80.0f;
    }
    void poll_1s() {          /* Function that checks the conditions of charging while the IrDA is connected */
        if (is_connected() && !is_overheat())
            send_heartbeat();
    }
    void send_heartbeat() {  /* Creates the message to send to the robot using the "compose" function below */
        uint8_t sw_state{0};

        if (is_connected()) {
            sw_state = 1;
        } else {
            sw_state = 0;
        }

        uint8_t buf[IRDA_DATA_LEN], param[3]{++heartbeat_counter, sw_state, rsoc}; // Message composed of 8 bytes, 3 bytes parameters -- Declaration
        s_msg.compose(buf, s_msg.HEARTBEAT, param);

        if (!device_is_ready(dev)) {
            LOG_ERR("UART device is not ready\n");
            return;
        }

        for (int i{0}; i < IRDA_DATA_LEN; ++i) {
            uart_poll_out(dev, buf[i]);
        }
       
        LOG_DBG("Hearbeat Send\n");
    } // Declaration of variables
    void serial_read() {
        if (!uart_irq_update(dev)) {
            LOG_ERR("uart_irq_update Failed\n");
            return;
        }

        if (!uart_irq_rx_ready(dev)) {
            LOG_ERR("uart_irq_rx_ready Failed\n");
            return;
        }

        if ((k_uptime_get() - last_serial_recv_time) > CHARGER_HEARTBEAT_TIMEOUT_MS) {
            s_msg.reset();
        }
            
        uint8_t data;
        while(0 < uart_fifo_read(dev, &data, 1)){
            last_serial_recv_time = k_uptime_get();
            if (s_msg.decode(data)) {
                uint8_t param[3];
                uint8_t command{s_msg.get_command(param)};
                if (command == serial_message::HEARTBEAT && param[0] == heartbeat_counter) {
                    start_time = k_uptime_get();
                }
            }
        }
    }

    const device* dev{nullptr}; // UART device
    int64_t start_time{0};      // Heartbeat timer for Charger
    int64_t last_serial_recv_time{0}; // last IrDA data received time
    uint8_t heartbeat_counter{0}, rsoc{0};
    float connector_v{0.0f}, connector_temp[2]{0.0f, 0.0f};
    uint32_t connect_check_count{0};
    k_timer timer_poll_1s;
    serial_message s_msg;
    static constexpr int ADDR{0b10010010}; // I2C adress for temp sensor
    static constexpr uint32_t CONNECT_THRES_COUNT{100}; // Number of times that ...
    static constexpr float CHARGING_VOLTAGE{30.0f * 1000.0f / (9100.0f + 1000.0f)},
                           CONNECT_THRES_VOLTAGE{3.3f * 0.5f * 1000.0f / (9100.0f + 1000.0f)};
};

char __aligned(4) msgq_can_bmu_pb_buffer[8 * sizeof (can_frame)];
class bmu_controller { // Variables Implemented
public:
    void init() {
        k_msgq_init(&msgq_can_bmu_pb, msgq_can_bmu_pb_buffer, sizeof (can_frame), 8);
    }
    void poll() {
        can_frame frame;

        // BMU CAN data from bmu_controller
        while (k_msgq_get(&msgq_can_bmu_pb, &frame, K_NO_WAIT) == 0){
            handle_can(frame);
        }
    }
    bool is_ok() const {
        LOG_DBG("data.mod_status1 %d", (data.mod_status1 & 0b10111111) == 0);
        LOG_DBG("data.mod_status1 %d", (data.mod_status2 & 0b11100001) == 0);
        LOG_DBG("data.bmu_alarm1 %d", (data.bmu_alarm1  & 0b11111111) == 0);
        LOG_DBG("data.bmu_alarm2 %d", (data.bmu_alarm2  & 0b00000001) == 0);

        return ((data.mod_status1 & 0b10111111) == 0 ||
                (data.mod_status2 & 0b11100001) == 0 ||
                (data.bmu_alarm1  & 0b11111111) == 0 ||
                (data.bmu_alarm2  & 0b00000001) == 0);
    }
    void get_fet_state(bool &c_fet, bool &d_fet, bool &p_dsg) {
        gpio_dt_spec gpio_c_dev = GET_GPIO(bmu_c_fet);
        gpio_dt_spec gpio_d_dev = GET_GPIO(bmu_d_fet);
        gpio_dt_spec gpio_p_dev = GET_GPIO(bmu_p_dsg);
        if (!gpio_is_ready_dt(&gpio_c_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_d_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_p_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        c_fet = gpio_pin_get_dt(&gpio_c_dev) == 1;
        d_fet = gpio_pin_get_dt(&gpio_d_dev) == 1;
        p_dsg = gpio_pin_get_dt(&gpio_p_dev) == 1;
    }
    bool is_full_charge() const {
        return (data.mod_status1 & 0b01000000) != 0;
    }
    bool is_chargable() const {
        return !is_full_charge() && data.rsoc < 95;
    }
    bool is_charging() const {
        return data.pack_a > 0;
    }
    uint8_t get_rsoc() const {
        return data.rsoc;
    }
private:
    void handle_can(can_frame &frame) {
        switch (frame.id) {
        case 0x100:
            data.mod_status1 = frame.data[0];
            data.asoc = frame.data[2];
            data.rsoc = frame.data[3];
            break;
        case 0x101:
            data.mod_status2 = frame.data[6];
            data.pack_a = (frame.data[0] << 8) | frame.data[1];
            data.pack_v = (frame.data[4] << 8) | frame.data[5];
            break;
        case 0x113:
            data.bmu_alarm1 = frame.data[4];
            data.bmu_alarm2 = frame.data[5];
            break;
        }
    }
    const device *dev{nullptr};
    struct {
        int16_t pack_a{0};
        uint16_t pack_v{0};
        uint8_t mod_status1{0xff}, mod_status2{0xff}, bmu_alarm1{0xff}, bmu_alarm2{0xff};
        uint8_t asoc{0}, rsoc{0};
    } data;
};

class dcdc_converter { // Variables Implemented
public:
    void set_enable(bool enable) {
        gpio_dt_spec gpio_dev; 
        // 0=OFF, 1=ON
        if (enable) {
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 1);
            k_msleep(3000);
            gpio_dev = GET_GPIO(v_peripheral);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 1);
            k_msleep(3000);
            gpio_dev = GET_GPIO(v24);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
        } else {
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
            k_msleep(3000);
            gpio_dev = GET_GPIO(v_peripheral);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
            k_msleep(3000);
            gpio_dev = GET_GPIO(v24);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 1);
        }
    }
    bool is_ok() {
        // 0:OK, 1:NG
        gpio_dt_spec gpio_pgood_24v_dev = GET_GPIO(pgood_24v);
        gpio_dt_spec gpio_pgood_peripheral_dev = GET_GPIO(pgood_peripheral);
        gpio_dt_spec gpio_pgood_wheel_motor_left_dev = GET_GPIO(pgood_wheel_motor_left);
        gpio_dt_spec gpio_pgood_wheel_motor_right_dev = GET_GPIO(pgood_wheel_motor_right);
        if (!gpio_is_ready_dt(&gpio_pgood_24v_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return false;
        }
        if (!gpio_is_ready_dt(&gpio_pgood_peripheral_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return false;
        }
        if (!gpio_is_ready_dt(&gpio_pgood_wheel_motor_left_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return false;
        }
        if (!gpio_is_ready_dt(&gpio_pgood_wheel_motor_right_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return false;
        }
        bool rtn = (gpio_pin_get_dt(&gpio_pgood_24v_dev) == 0)                  
            && (gpio_pin_get_dt(&gpio_pgood_peripheral_dev) == 0)
            && (gpio_pin_get_dt(&gpio_pgood_wheel_motor_left_dev) == 0)
            && (gpio_pin_get_dt(&gpio_pgood_wheel_motor_right_dev) == 0);
        if (rtn == false) {
            LOG_ERR("dcdc is_ok() NG: %d", rtn);
        }

        return rtn;
    }
    void get_failed_state(bool &v24, bool &v_peripheral, bool &v_wheel_motor_left, bool &v_wheel_motor_right) {
        gpio_dt_spec gpio_pgood_24v_dev = GET_GPIO(pgood_24v);
        gpio_dt_spec gpio_pgood_peripheral_dev = GET_GPIO(pgood_peripheral);
        gpio_dt_spec gpio_pgood_wheel_motor_left_dev = GET_GPIO(pgood_wheel_motor_left);
        gpio_dt_spec gpio_pgood_wheel_motor_right_dev = GET_GPIO(pgood_wheel_motor_right);
        if (!gpio_is_ready_dt(&gpio_pgood_24v_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_pgood_peripheral_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_pgood_wheel_motor_left_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_pgood_wheel_motor_right_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        // 0:OK, 1:NG
        v24 = gpio_pin_get_dt(&gpio_pgood_24v_dev) == 1;
        v_peripheral = gpio_pin_get_dt(&gpio_pgood_peripheral_dev) == 1;
        v_wheel_motor_left = gpio_pin_get_dt(&gpio_pgood_wheel_motor_left_dev) == 1;
        v_wheel_motor_right = gpio_pin_get_dt(&gpio_pgood_wheel_motor_right_dev) == 1;
    }
private:
};

class fan_driver { // Variables Implemented
public:
    void init() {
        fan_on();
    }
    void fan_on() {
        gpio_dt_spec gpio_fan1_dev = GET_GPIO(fan1);
        gpio_dt_spec gpio_fan2_dev = GET_GPIO(fan2);
        gpio_dt_spec gpio_fan3_dev = GET_GPIO(fan3);
        gpio_dt_spec gpio_fan4_dev = GET_GPIO(fan4);
        if (!gpio_is_ready_dt(&gpio_fan1_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_fan2_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_fan3_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_fan4_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        gpio_pin_set_dt(&gpio_fan1_dev, 1);    // 1:ON, 0:OFF
        gpio_pin_set_dt(&gpio_fan2_dev, 1);    // 1:ON, 0:OFF
        gpio_pin_set_dt(&gpio_fan3_dev, 1);   // 1:ON, 0:OFF
        gpio_pin_set_dt(&gpio_fan4_dev, 1);   // 1:ON, 0:OFF
    }
    void fan_off() {
        gpio_dt_spec gpio_fan1_dev = GET_GPIO(fan1);
        gpio_dt_spec gpio_fan2_dev = GET_GPIO(fan2);
        gpio_dt_spec gpio_fan3_dev = GET_GPIO(fan3);
        gpio_dt_spec gpio_fan4_dev = GET_GPIO(fan4);
        if (!gpio_is_ready_dt(&gpio_fan1_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_fan2_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_fan3_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        if (!gpio_is_ready_dt(&gpio_fan4_dev)) {
            LOG_ERR("gpio_is_ready_dt Failed\n");
            return;
        }
        gpio_pin_set_dt(&gpio_fan1_dev, 0);    // 1:ON, 0:OFF
        gpio_pin_set_dt(&gpio_fan2_dev, 0);    // 1:ON, 0:OFF
        gpio_pin_set_dt(&gpio_fan3_dev, 0);   // 1:ON, 0:OFF
        gpio_pin_set_dt(&gpio_fan4_dev, 0);   // 1:ON, 0:OFF
    }
private:
};

class boardstate_controller {  // No pins declared
public:
    void init() {
        k_msgq_init(&msgq_board_pb_rx, msgq_board_pb_rx_buffer, sizeof (msg_rcv_pb), 8);
    }
    void poll() {
        msg_rcv_pb msg;
        if (k_msgq_get(&msgq_board_pb_rx, &msg, K_NO_WAIT) == 0) {
            handle_board(msg);
        }
    }
    bool emergency_stop_from_ros() const {
        return emergency_stop;
    }
    bool power_off_from_ros() const {
        return power_off;
    }
    bool is_dead() const {
        if (heartbeat_detect)
            return ros_heartbeat_timeout;
        else
            return false;
    }
    bool is_ready() const {
        return heartbeat_detect;
    }
    bool is_wheel_poweroff() const {
        return wheel_poweroff;
    }
private:
    void handle_board(const msg_rcv_pb &msg) {
        if (emergency_stop != msg.ros_emergency_stop) {
            LOG_INF("ROS Emergency Stop: %d", msg.ros_emergency_stop);
        }
        if (power_off != msg.ros_power_off) {
            LOG_INF("ROS Power Off: %d", msg.ros_power_off);
        }
        if (ros_heartbeat_timeout != msg.ros_heartbeat_timeout) {
            LOG_INF("ROS Heartbeat Timeout: %d", msg.ros_heartbeat_timeout);
        }
        if (wheel_poweroff != msg.ros_wheel_power_off) {
            LOG_INF("ROS Wheel Power Off: %d", msg.ros_wheel_power_off);
        }

        emergency_stop = msg.ros_emergency_stop;
        power_off = msg.ros_power_off;
        ros_heartbeat_timeout = msg.ros_heartbeat_timeout;
        wheel_poweroff = msg.ros_wheel_power_off;

        // heartbeat is not timeout means heartbeat is detected
        heartbeat_detect |= !ros_heartbeat_timeout;
    }
    bool heartbeat_detect{false}, ros_heartbeat_timeout{false}, emergency_stop{true}, power_off{false},
        wheel_poweroff{false};
};

#define WDT_TIMEOUT_MS 10000
#define FAN_DUTY_DEFAULT 100
class state_controller { // Variables Implemented
public:
    void init() {
        mc.init();
        ac.init();
        bmu.init();
        fan.init();
        mbd.init();

        k_timer_init(&timer_poll_100ms, static_poll_100ms_callback, NULL);
        k_timer_user_data_set(&timer_poll_100ms, this);
        k_timer_start(&timer_poll_100ms, K_MSEC(100), K_MSEC(100));

        k_timer_init(&timer_poll_1s, static_poll_1s_callback, NULL);
        k_timer_user_data_set(&timer_poll_1s, this);
        k_timer_start(&timer_poll_1s, K_MSEC(1000), K_MSEC(1000));

        k_timer_init(&current_check_timeout, static_current_check_timeout_callback, NULL);
        k_timer_user_data_set(&current_check_timeout, this);

        k_timer_init(&charge_guard_timeout, static_charge_guard_timeout_callback, NULL);
        k_timer_user_data_set(&charge_guard_timeout, this);

        timer_post = k_uptime_get();
        timer_shutdown = k_uptime_get();
        timer_poweroff = k_uptime_get();

        k_msgq_init(&msgq_board_pb_tx, msgq_board_pb_tx_buffer, sizeof(lexxhard::can_controller::msg_board), 8);

        // Setup Watch Dog Timer
        dev_wdi = GET_DEV(iwdg);

        if (!device_is_ready(dev_wdi)){
            LOG_ERR("Watchdog device is not ready\n");
            return;
        }

        struct wdt_timeout_cfg wdt_config;
        wdt_config.flags = WDT_FLAG_RESET_SOC;
        wdt_config.window.min = 0;
        wdt_config.window.max = WDT_TIMEOUT_MS;
        wdt_config.callback = NULL;

        int wdt_channel_id = wdt_install_timeout(dev_wdi, &wdt_config);
        if (wdt_channel_id < 0) {
            LOG_ERR("WDT install timeout error\n");
            return;
        }
        
        int err_setup = wdt_setup(dev_wdi, WDT_OPT_PAUSE_HALTED_BY_DBG);
        if (err_setup) {
            LOG_ERR("WDT setup error: %d\n", err_setup);
            return;
        }

        esw.set_callback([&](){
            this->bsw.request_reset();
        });
    }
    void run() {
        while(true) {
            poll();
            k_msleep(20);
        }
    }
    void power_on() {
        dcdc.set_enable(true);
    }
    void power_off() {
        dcdc.set_enable(false);
    }
    void auto_charge_on() {
        ac.set_enable(true);
    }
    void auto_charge_off() {
        ac.set_enable(false);
    }
    bool is_bmu_ok_dbg() {
        return bmu.is_ok();
    }
    void set_wheel_enable(){
        wsw.set_disable(false);
    }
    void set_wheel_disable(){
        wsw.set_disable(true);
    }
    bool is_esw_asserted(){
        return esw.is_asserted();
    }
private:
    static void static_poll_100ms_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<state_controller*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->poll_100ms();
        }
    }
    static void static_poll_1s_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<state_controller*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->poll_1s();
        }
    }
    static void static_charge_guard_timeout_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<state_controller*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->charge_guard_asserted = false;
        }
    }
    static void static_current_check_timeout_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<state_controller*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->current_check_enable = true;
        }
    }

    enum class POWER_STATE {
        OFF,
        WAIT_SW,
        POST,
        STANDBY,
        NORMAL,
        AUTO_CHARGE,
        MANUAL_CHARGE,
        LOCKDOWN,
        TIMEROFF,
        EMERGENCY,
        RESUME_WAIT,
    };
    void poll() {
        auto wheel_relay_control = [&](){
            bool wheel_poweroff{mbd.is_wheel_poweroff()};
            if (last_wheel_poweroff != wheel_poweroff) {
                last_wheel_poweroff = wheel_poweroff;
                gpio_dt_spec gpio_dev = GET_GPIO(v_wheel);
                if (!gpio_is_ready_dt(&gpio_dev)) {
                    LOG_ERR("gpio_is_ready_dt Failed\n");
                    return;
                }
                gpio_pin_set_dt(&gpio_dev, wheel_poweroff ? 0 : 1);
                LOG_DBG("wheel power control %d!\n", wheel_poweroff);
            }
        };
        
        psw.poll();
        rsw.poll();
        ksw.poll();
        bsw.poll();
        esw.poll();
        mc.poll();
        ac.poll();
        bmu.poll();
        mbd.poll();
        
        switch (state) {
        case POWER_STATE::OFF:
            set_new_state(mc.is_plugged() ? POWER_STATE::POST : POWER_STATE::WAIT_SW);
            break;
        case POWER_STATE::TIMEROFF:
            if ((k_uptime_get() - timer_poweroff) > 5000)
                set_new_state(POWER_STATE::OFF);
            break;
        case POWER_STATE::WAIT_SW:
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                poweron_by_switch = true;
                psw.reset_state();
                set_new_state(POWER_STATE::POST);
            } else if (mc.is_plugged()) {
                set_new_state(POWER_STATE::POST);
            }
            break;
        case POWER_STATE::POST:
            if (!poweron_by_switch && !mc.is_plugged()) {
                LOG_DBG("unplugged from manual charger\n");
                set_new_state(POWER_STATE::OFF);
            } else if (bmu.is_ok()) {
                LOG_DBG("BMU and temperature OK\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if ((k_uptime_get() - timer_post) > 3000) {
                LOG_DBG("timer_post > 3000\n");
                set_new_state(POWER_STATE::OFF);
            }
            break;
        case POWER_STATE::STANDBY: {
            wheel_relay_control();
            auto psw_state{psw.get_state()};
            if (!dcdc.is_ok() || psw_state == power_switch::STATE::LONG_PUSHED) {
                set_new_state(POWER_STATE::OFF);
            } else if (mbd.is_dead()) {
                set_new_state(wait_shutdown ? POWER_STATE::TIMEROFF : POWER_STATE::LOCKDOWN);
            } else if (psw_state == power_switch::STATE::PUSHED || mbd.power_off_from_ros() || !bmu.is_ok()) {
                if (wait_shutdown) {
                    if ((k_uptime_get() - timer_shutdown)> 60000) {
                        set_new_state(POWER_STATE::OFF);
                        dcdc.set_enable(false);
                        psw.reset_state();
                        esw.reset_state();
                    }
                } else {
                    LOG_DBG("wait shutdown\n");
                    wait_shutdown = true;
                    timer_shutdown = k_uptime_get();    // timer reset
                    if (psw_state == power_switch::STATE::PUSHED)
                        shutdown_reason = SHUTDOWN_REASON::SWITCH;
                    if (mbd.power_off_from_ros())
                        shutdown_reason = SHUTDOWN_REASON::ROS;
                    if (!bmu.is_ok())
                        shutdown_reason = SHUTDOWN_REASON::BMU;
                    // Set LED
                    led_controller::msg msg_led;
                    msg_led.pattern = led_controller::msg::SHOWTIME;
                    msg_led.interrupt_ms = 0;
                    while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                        k_msgq_purge(&led_controller::msgq);
                }
            } else if (ksw.is_maintenance()) {
                LOG_DBG("maintenance mode is selected by key switch\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (esw.is_asserted()) {
                LOG_DBG("emergency switch asserted\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (!esw.is_asserted() && !mbd.emergency_stop_from_ros() && mbd.is_ready()) {
                LOG_DBG("not emergency and heartbeat OK\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (mc.is_plugged()) {
                LOG_DBG("plugged to manual charger\n");
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            }
            break;
        }
        case POWER_STATE::NORMAL:
            wheel_relay_control();
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG_DBG("detect power switch\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.power_off_from_ros()) {
                LOG_DBG("receive power off from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!bmu.is_ok()) {
                LOG_DBG("BMU failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!dcdc.is_ok()) {
                LOG_DBG("DCDC failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (ksw.is_maintenance()) {
                LOG_DBG("maintenance mode is selected by key switch\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (esw.is_asserted()) {
                LOG_DBG("emergency switch asserted\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG_DBG("receive emergency stop from ROS\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.is_dead()) {
                LOG_DBG("mainboard is dead\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mc.is_plugged()) {
                LOG_DBG("plugged to manual charger\n");
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            } else if (!charge_guard_asserted && ac.is_docked() && bmu.is_chargable()) {
                if(ac.is_charger_ready() == true){
                    LOG_DBG("docked to auto charger\n");
                    set_new_state(POWER_STATE::AUTO_CHARGE);
                }
            }
            break;
        case POWER_STATE::EMERGENCY: {
            wheel_relay_control();
            auto psw_state{psw.get_state()};
            if (!dcdc.is_ok() || psw_state == power_switch::STATE::LONG_PUSHED) {
                set_new_state(POWER_STATE::OFF);
            } else if (mbd.is_dead()) {
                set_new_state(wait_shutdown ? POWER_STATE::TIMEROFF : POWER_STATE::LOCKDOWN);
            } else if (psw_state == power_switch::STATE::PUSHED || mbd.power_off_from_ros() || !bmu.is_ok()) {
                if (wait_shutdown) {
                    if ((k_uptime_get() - timer_shutdown)> 60000) {
                        set_new_state(POWER_STATE::OFF);
                        dcdc.set_enable(false);
                        psw.reset_state();
                        esw.reset_state();
                    }
                } else {
                    LOG_DBG("wait shutdown\n");
                    wait_shutdown = true;
                    timer_shutdown = k_uptime_get();    // timer reset
                    if (psw_state == power_switch::STATE::PUSHED)
                        shutdown_reason = SHUTDOWN_REASON::SWITCH;
                    if (mbd.power_off_from_ros())
                        shutdown_reason = SHUTDOWN_REASON::ROS;
                    if (!bmu.is_ok())
                        shutdown_reason = SHUTDOWN_REASON::BMU;
                    // Set LED
                    led_controller::msg msg_led;
                    msg_led.pattern = led_controller::msg::SHOWTIME;
                    msg_led.interrupt_ms = 0;
                    while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                        k_msgq_purge(&led_controller::msgq);
                }
            } else if (!ksw.is_maintenance() && !esw.is_asserted() && !mbd.emergency_stop_from_ros()) {
                LOG_DBG("not emergency\n");
                set_new_state(POWER_STATE::RESUME_WAIT);
            }
            break;
        }
        case POWER_STATE::RESUME_WAIT:
            wheel_relay_control();
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG_DBG("detect power switch\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.power_off_from_ros()) {
                LOG_DBG("receive power off from ROS\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (!bmu.is_ok()) {
                LOG_DBG("BMU failure\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (!dcdc.is_ok()) {
                LOG_DBG("DCDC failure\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (ksw.is_maintenance()) {
                LOG_DBG("maintenance mode is selected by key switch\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (esw.is_asserted()) {
                LOG_DBG("emergency switch asserted\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG_DBG("receive emergency stop from ROS\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.is_dead()) {
                LOG_DBG("mainboard is dead\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (rsw.get_state() == resume_switch::STATE::PUSHED) {
                LOG_DBG("resume switch pushed\n");
                if (mbd.is_ready()) {
                    LOG_DBG("heartbeat OK\n");
                    set_new_state(POWER_STATE::NORMAL);
                }
                else {
                    LOG_DBG("heartbeat NG\n");
                    set_new_state(POWER_STATE::STANDBY);
                }
            }
            break;
        case POWER_STATE::AUTO_CHARGE:
            ac.update_rsoc(bmu.get_rsoc());
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG_DBG("detect power switch\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.power_off_from_ros()) {
                LOG_DBG("receive power off from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!bmu.is_ok()) {
                LOG_DBG("BMU failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!dcdc.is_ok()) {
                LOG_DBG("DCDC failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (ksw.is_maintenance()) {
                LOG_DBG("maintenance mode is selected by key switch\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (esw.is_asserted()) {
                LOG_DBG("emergency switch asserted\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG_DBG("receive emergency stop from ROS\n");
                set_new_state(POWER_STATE::EMERGENCY);
            } else if (mbd.is_dead()) {
                LOG_DBG("main board or ROS dead\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (bmu.is_full_charge()) {
                LOG_DBG("full charge\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (!ac.is_docked()) {
                LOG_DBG("undocked from auto charger\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (mc.is_plugged()) {
                LOG_DBG("manual charger plugged\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (current_check_enable && !bmu.is_charging()) {
                LOG_DBG("not charging\n");
                set_new_state(POWER_STATE::NORMAL);
            }
            break;
        case POWER_STATE::MANUAL_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG_DBG("detect power switch (ignored)\n");
                psw.reset_state();
            }
            if (!mc.is_plugged()) {
                LOG_DBG("unplugged from manual charger\n");
                set_new_state(POWER_STATE::NORMAL);
            }
            break;
        case POWER_STATE::LOCKDOWN:
            if (!dcdc.is_ok()) {
                LOG_DBG("DCDC failure\n");
                set_new_state(POWER_STATE::OFF);
            } else if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG_DBG("detect power switch\n");
                set_new_state(POWER_STATE::OFF);
            } else if (psw.is_activated_unlock()) {
                LOG_DBG("force recover from lockdown\n");
                set_new_state(POWER_STATE::STANDBY);
            }
            break;
        }
    }
    void set_new_state(POWER_STATE newstate) {
        switch (state) {
        case POWER_STATE::NORMAL:
            k_timer_stop(&charge_guard_timeout);
            break;
        case POWER_STATE::AUTO_CHARGE:
            k_timer_stop(&current_check_timeout);
            ac.force_stop();
            break;
        default:
            break;
        }
        int bat_out_state{mbd.is_wheel_poweroff() ? 0 : 1};
        gpio_dt_spec gpio_dev;
        led_controller::msg msg_led;
        switch (newstate) {
        case POWER_STATE::OFF:
            LOG_INF("enter OFF\n");
            poweron_by_switch = false;
            psw.set_led(false);
            dcdc.set_enable(false);

            // Set LED OFF
            msg_led.pattern = led_controller::msg::NONE;
            msg_led.interrupt_ms = 0;
            while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller::msgq);
            break;
        case POWER_STATE::TIMEROFF:
            LOG_INF("enter TIMEROFF\n");
            timer_poweroff = k_uptime_get();    // timer reset
            break;
        case POWER_STATE::WAIT_SW:
            LOG_INF("enter WAIT_SW\n");
            break;
        case POWER_STATE::POST:
            LOG_INF("enter POST\n");
            psw.set_led(true);
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
            timer_post = k_uptime_get();    // timer reset
            // Set LED
            msg_led.pattern = led_controller::msg::SHOWTIME;
            msg_led.interrupt_ms = 0;
            while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller::msgq);
            break;
        case POWER_STATE::STANDBY:
            LOG_INF("enter STANDBY\n");
            psw.set_led(true);
            dcdc.set_enable(true);
            wsw.set_disable(true);
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, bat_out_state);
            ac.set_enable(false);
            wait_shutdown = false;
            break;
        case POWER_STATE::NORMAL:
            LOG_INF("enter NORMAL\n");
            wsw.set_disable(false);
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, bat_out_state);
            ac.set_enable(false);
            charge_guard_asserted = true;
            k_timer_start(&charge_guard_timeout, K_MSEC(10000), K_NO_WAIT); // charge_guard_asserted = false after 10sec
            break;

        case POWER_STATE::EMERGENCY:
            LOG_INF("enter EMERGENCY\n");
            psw.set_led(true);
            wsw.set_disable(true);
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, bat_out_state);
            ac.set_enable(false);
            wait_shutdown = false;
            break;
        case POWER_STATE::RESUME_WAIT:
            LOG_INF("enter RESUME_WAIT\n");
            break;
        case POWER_STATE::AUTO_CHARGE:
            LOG_INF("enter AUTO_CHARGE\n");
            ac.set_enable(true);
            current_check_enable = false;
            k_timer_start(&current_check_timeout, K_MSEC(10000), K_NO_WAIT); // current_check_timeout = true after 10sec

            // Set LED
            msg_led.pattern = led_controller::msg::CHARGE_LEVEL;
            msg_led.interrupt_ms = 2000;
            while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller::msgq);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            LOG_INF("enter MANUAL_CHARGE\n");
            wsw.set_disable(true);
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
            ac.set_enable(false);
            break;
        case POWER_STATE::LOCKDOWN:
            LOG_INF("enter LOCKDOWN\n");
            wsw.set_disable(true);
            gpio_dev = GET_GPIO(v_wheel);
            if (!gpio_is_ready_dt(&gpio_dev)) {
                LOG_ERR("gpio_is_ready_dt Failed\n");
                return;
            }
            gpio_pin_set_dt(&gpio_dev, 0);
            ac.set_enable(false);

            // Set LED
            msg_led.pattern = led_controller::msg::LOCKDOWN;
            msg_led.interrupt_ms = 1000000000;
            while (k_msgq_put(&led_controller::msgq, &msg_led, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller::msgq);
            break;
        }
        state = newstate;
    }
    void poll_100ms() {
        board2ros.state = static_cast<uint32_t>(state);
        board2ros.emergency_switch_asserted = esw.is_asserted();
        board2ros.bumper_switch_asserted = bsw.is_asserted();
        board2ros.manual_charging_status = mc.is_plugged();
        board2ros.auto_charging_status = ac.is_docked();
        board2ros.shutdown_reason = static_cast<uint32_t>(shutdown_reason);
        board2ros.wait_shutdown_state = wait_shutdown;
        board2ros.wheel_enable = wsw.is_enabled();

        bool v24{false}, v_peripheral{false}, v_wheel_motor_left{false}, v_wheel_motor_right{false};
        dcdc.get_failed_state(v24, v_peripheral, v_wheel_motor_left, v_wheel_motor_right);
        board2ros.v24_pgood = v24;
        board2ros.v_peripheral_pgood = v_peripheral;
        board2ros.v_wheel_motor_l_pgood = v_wheel_motor_left;
        board2ros.v_wheel_motor_r_pgood = v_wheel_motor_right;

        int t0, t1;
        ac.get_connector_temperature(t0, t1);
        board2ros.charge_connector_p_temp = t0;
        board2ros.charge_connector_n_temp = t1;

        board2ros.fan_duty = FAN_DUTY_DEFAULT;

        board2ros.charge_connector_voltage = ac.get_connector_voltage();
        board2ros.charge_check_count = ac.get_connect_check_count();
        board2ros.charge_heartbeat_delay = ac.get_heartbeat_delay();

        bool c_fet{false}, d_fet{false}, p_dsg{false};
        bmu.get_fet_state(c_fet, d_fet, p_dsg);
        board2ros.c_fet = c_fet;
        board2ros.d_fet = d_fet;
        board2ros.p_dsg = p_dsg;

        board2ros.is_activated_battery = psw.is_activated_battery() ? 1 : 0;

        if (state == POWER_STATE::LOCKDOWN)
            psw.toggle_led();

        if (k_msgq_put(&msgq_board_pb_tx, &board2ros, K_NO_WAIT) != 0) {
            k_msgq_purge(&msgq_board_pb_tx);
        }
    }
    void poll_1s() {
        if (!device_is_ready(dev_wdi)){
            LOG_INF("Watchdog device is not ready\n");
            return;
        }
        wdt_feed(dev_wdi, 0);   // Feed the watchdog, Second value will not be used for STM32
    }
    
    power_switch psw;
    resume_switch rsw;
    key_switch ksw;
    bumper_switch bsw;
    emergency_switch esw;
    wheel_switch wsw;
    manual_charger mc;
    auto_charger ac;
    bmu_controller bmu;
    dcdc_converter dcdc;
    fan_driver fan;
    boardstate_controller mbd;
    POWER_STATE state{POWER_STATE::OFF};
    enum class SHUTDOWN_REASON {
        NONE,
        SWITCH,
        ROS,
        MAINBOARD_TEMP,
        POWERBOARD_TEMP,
        BMU,
    } shutdown_reason{SHUTDOWN_REASON::NONE};

    lexxhard::can_controller::msg_board board2ros;
    int64_t timer_post{0}, timer_shutdown{0}, timer_poweroff{0};
    k_timer timer_poll_20ms, timer_poll_100ms, timer_poll_1s;
    k_timer current_check_timeout, charge_guard_timeout;
    const device *dev_wdi{nullptr};
    bool poweron_by_switch{false}, wait_shutdown{false}, current_check_enable{false}, charge_guard_asserted{false},
         last_wheel_poweroff{false};
} impl;

int cmd_power_on(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Force Power ON for Debug purpuse");
    
    impl.power_on();

    return 0;
}

int cmd_power_off(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Force Power OFF for Debug purpuse");
    
    impl.power_off();

    return 0;
}

int cmd_auto_charge_on(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Force Power ON for Debug purpuse");
    
    impl.auto_charge_on();

    return 0;
}

int cmd_auto_charge_off(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Force Power OFF for Debug purpuse");
    
    impl.auto_charge_off();

    return 0;
}

int cmd_is_bmu_ok(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "BMU status check");
    
    bool result = impl.is_bmu_ok_dbg();

    shell_print(shell, "BMU status[1:OK 0:NG]: %d", result);

    return 0;
}

int cmd_set_wheel_enable(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Wheel Enable");
    
    impl.set_wheel_enable();

    return 0;
}

int cmd_set_wheel_disable(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Wheel Disable");
    
    impl.set_wheel_disable();

    return 0;
}
int cmd_is_esw_asserted(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Wheel Disable");
    
    bool result = impl.is_esw_asserted();

    shell_print(shell, "ESW Asserted[1:True 0:False]: %d", result);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(power_on, NULL, "Force Power ON command", cmd_power_on),
    SHELL_CMD(power_off, NULL, "Force Power OFF command", cmd_power_off),
    SHELL_CMD(auto_charge_on, NULL, "Auto Charge ON command", cmd_auto_charge_on),
    SHELL_CMD(auto_charge_off, NULL, "Auto Charge OFF command", cmd_auto_charge_off),
    SHELL_CMD(is_bmu_ok, NULL, "BMU status check command", cmd_is_bmu_ok),
    SHELL_CMD(set_wheel_enable, NULL, "Wheel Enable command", cmd_set_wheel_enable),
    SHELL_CMD(set_wheel_disable, NULL, "Wheel Disable command", cmd_set_wheel_disable),
    SHELL_CMD(is_esw_asserted, NULL, "ESW status check command", cmd_is_esw_asserted),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pbrd, &sub, "PowerBoard commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
k_msgq msgq_board_pb_rx, msgq_board_pb_tx, msgq_can_bmu_pb;

}



// vim: set expandtab shiftwidth=4:
