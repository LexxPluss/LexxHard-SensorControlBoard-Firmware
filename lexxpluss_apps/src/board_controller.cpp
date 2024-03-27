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

#include <zephyr.h>
#include <cmath>
#include <kernel.h>
#include <device.h>
#include <drivers/can.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/watchdog.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <sys/util.h>
#include "adc_reader.hpp"
#include "board_controller.hpp"
#include "can_controller.hpp"


namespace lexxhard::board_controller {

LOG_MODULE_REGISTER(board);

char __aligned(4) msgq_bmu_pb_buffer[8 * sizeof (can_controller::msg_bmu)];
char __aligned(4) msgq_board_pb_rx_buffer[8 * sizeof (msg_rcv_pb)];
char __aligned(4) msgq_board_pb_tx_buffer[8 * sizeof (can_controller::msg_board)];

pin_def_gpio ps_sw_in{"GPIOH", 4}, ps_led_out{"GPIOH", 5}; // Power Switch handler associated pins
pin_def_gpio bp_left{"GPIOI", 7}; // Bumper Switch associated pins
pin_def_gpio es_left{"GPIOI", 4}, es_right("GPIOI", 0); // Emergency Switch associated pins
pin_def_gpio wh_left_right{"GPIOK", 3}; // Wheel switch associated pins
pin_def_gpio mc_din{"GPIOK", 7}; // Manual charging detection associated pins
pin_def_gpio ac_th_pos{"GPIOC", 4}, ac_th_neg{"GPIOC", 5}, ac_IrDA_tx{"GPIOG", 14}, ac_IrDA_rx{"GPIOG", 9}, ac_analogVol{"GPIOF", 10}, ac_chargingRelay{"GPIOD", 0}; // Auto charging detection associated pins
pin_def_gpio bmu_c_fet{"GPIOJ", 5}, bmu_d_fet{"GPIOJ", 12}, bmu_p_dsg{"GPIOJ", 13}; // BMU controller associated pins
pin_def_gpio ts_i2c_scl{"GPIOF", 14}, ts_i2c_sda{"GPIOF", 15}; // Temperature sensors associated I2C pins
pin_def_gpio pwr_control_24v{"GPIOC", 15}, pwr_control_peripheral{"GPIOD", 2}, pwr_control_wheel_motor{"GPIOD", 1}; // Power Control associated pins
pin_def_gpio pgood_24v{"GPIOH", 3}, pgood_peripheral{"GPIOH", 15}, pgood_wheel_motor_left{"GPIOH", 1}, pgood_wheel_motor_right{"GPIOH", 12}; // Power Good associated pins
pin_def_gpio pgood_linear_act_left{"GPIOK", 4}, pgood_linear_act_right{"GPIOK", 5}, pgood_linear_act_center{"GPIOK", 4};
pin_def_gpio fan_pwm_5v_1{"GPIOC", 10}, fan_pwm_5v_2{"GPIOC", 11}, fan_pwm_24v_1{"GPIOB", 14}, fan_pwm_24v_2{"GPIOB", 15}; // PWM fan signal control pin

void gpio_set_value(pin_def_gpio pin_def, uint8_t output_value) {
    const device *gpio_dev{device_get_binding(pin_def.label)};

    if (device_is_ready(gpio_dev)) {
        gpio_pin_configure(gpio_dev, pin_def.io_number, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_set(gpio_dev, pin_def.io_number, output_value);
    }
    return;
}

int8_t gpio_get_value(pin_def_gpio pin_def) {
    int8_t rtn{-1};
    const device *gpio_dev{device_get_binding(pin_def.label)}; 

    if (device_is_ready(gpio_dev)) {
        gpio_pin_configure(gpio_dev, pin_def.io_number, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        rtn = gpio_pin_get(gpio_dev, pin_def.io_number);
    }
    return rtn;
}

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
        int now{gpio_get_value(ps_sw_in)};
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
        return gpio_get_value(ps_sw_in) == 0;
    }
    void set_led(bool enabled) {
        gpio_set_value(ps_led_out, enabled ? 1 : 0);
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

#define BUMPER_SWITCH_HOLD_TIME_MS 1000
class bumper_switch { // Variables Implemented
public:
    void poll() {
        if(asserted) {
            if ((k_uptime_get() - start_time) > BUMPER_SWITCH_HOLD_TIME_MS) {
                asserted = false;
            }
        } else {
            if (gpio_get_value(bp_left) == 0) {
                asserted = true;
                start_time = k_uptime_get();
            }
        }
    }
    void get_raw_state(bool &left, bool &right) const {
        left = right = asserted;
    }
private:
    int64_t start_time;
    bool asserted{false};
};

class emergency_switch { // Variables Implemented
public:
    void poll() {
        int now{gpio_get_value(es_left)};
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
        now = gpio_get_value(es_right);
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
    }
    bool asserted() const {return left_asserted || right_asserted;}
    void get_raw_state(bool &left, bool &right) const {
        left = left_asserted;
        right = right_asserted;
    }
private:
    uint32_t left_count{0}, right_count{0};
    int left_prev{-1}, right_prev{-1};
    bool left_asserted{false}, right_asserted{false};
    static constexpr uint32_t COUNT{5};
};

class wheel_switch { // Variables Implemented
public:
    void set_disable(bool disable) {
        if (disable) {
            gpio_set_value(wh_left_right, 1);
            left_right_disable = true; 
        } else {
            gpio_set_value(wh_left_right, 0);
            left_right_disable = false;
        }
    }
    void get_raw_state(bool &left, bool &right) {
        left = left_right_disable;
        right = left_right_disable;
    }
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
        int now{gpio_get_value(mc_din)};
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
            if (gpio_get_value(mc_din) == 0)
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
#define IRDA_DATA_LEN 8
#define IRDA_TX_TIMEOUT_MS 1000
class auto_charger { // Variables Half-Implemented (Not Thermistors ADC)
public:
    void init() {
        k_timer_init(&timer_poll_1s, static_poll_1s_callback, NULL);
        k_timer_user_data_set(&timer_poll_1s, this);
        k_timer_start(&timer_poll_1s, K_MSEC(1000), K_MSEC(1000));

        start_time = k_uptime_get();

        dev = device_get_binding("UART_6");
        return;
    }
    bool is_docked() const {
        return is_connected() && !is_overheat() && (k_uptime_get() - start_time) < AUTO_CHARGE_DOCKED_TIMEOUT_MS;
    }
    void set_enable(bool enable) {
        gpio_set_value(ac_chargingRelay, enable ? 1 : 0);
    }
    void force_stop() {
        set_enable(false);
        send_heartbeat();
        return;
    }
    bool is_charger_ready() const {
        if(connector_v > (CHARGING_VOLTAGE * 0.9)){
            LOG_DBG("charger ready voltage:%f.\n", connector_v);
            return true;
        }else {
            LOG_DBG("connector_v:%f THRESH:%f\n", connector_v, (CHARGING_VOLTAGE * 0.9));
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
    void adc_read() { // Change to read the temperature sensor from ADC pin directly. Thermistor side.
        float v_th_pos{(float)(adc_reader::get(adc_reader::THERMISTOR_P) / 1000.0f)};
        float v_th_neg{(float)(adc_reader::get(adc_reader::THERMISTOR_N) / 1000.0f)};
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

        if(is_connected()){
            sw_state = 1;
        }else{
            sw_state = 0;
        }

        uint8_t buf[IRDA_DATA_LEN], param[3]{++heartbeat_counter, sw_state, rsoc}; // Message composed of 8 bytes, 3 bytes parameters -- Declaration
        serial_message::compose(buf, serial_message::HEARTBEAT, param);

        if (!device_is_ready(dev)) {
            LOG_DBG("UART device is not ready\n");
            return;
        }

        int ret = uart_tx(dev, buf, IRDA_DATA_LEN, IRDA_TX_TIMEOUT_MS);
        if (ret < 0) {
            LOG_DBG("Failed to send data over UART\n");
        }
    } // Declaration of variables

    const device* dev{nullptr};
    int64_t start_time{0};
    uint8_t heartbeat_counter{0}, rsoc{0};
    float connector_v{0.0f}, connector_temp[2]{0.0f, 0.0f};
    uint32_t connect_check_count{0};
    k_timer timer_poll_1s;
    static constexpr int ADDR{0b10010010}; // I2C adress for temp sensor
    static constexpr uint32_t CONNECT_THRES_COUNT{100}; // Number of times that ...
    static constexpr float CHARGING_VOLTAGE{30.0f * 1000.0f / (9100.0f + 1000.0f)},
                           CONNECT_THRES_VOLTAGE{3.3f * 0.5f * 1000.0f / (9100.0f + 1000.0f)};
};

CAN_DEFINE_MSGQ(msgq_can_bmu_pb, 16);
class bmu_controller { // Variables Implemented
public:
    void init() {
        k_msgq_init(&msgq_can_bmu_pb, msgq_bmu_pb_buffer, sizeof (can_controller::msg_bmu), 8);
        dev = device_get_binding("CAN_1");
        if (!device_is_ready(dev))
            return;
        setup_can_filter();
    }
    void poll() {
        zcan_frame frame;
        while (k_msgq_get(&msgq_can_bmu_pb, &frame, K_NO_WAIT) == 0){
            handle_can(frame);
        }
    }
    bool is_ok() const {
        return ((data.mod_status1 & 0b10111111) == 0 ||
                (data.mod_status2 & 0b11100001) == 0 ||
                (data.bmu_alarm1  & 0b11111111) == 0 ||
                (data.bmu_alarm2  & 0b00000001) == 0);
    }
    void get_fet_state(bool &c_fet, bool &d_fet, bool &p_dsg) {
        c_fet = gpio_get_value(bmu_c_fet) == 1;
        d_fet = gpio_get_value(bmu_d_fet) == 1;
        p_dsg = gpio_get_value(bmu_p_dsg) == 1;
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
    void setup_can_filter() const {
        static const zcan_filter filter_bmu{
            .id{0x100},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7c0},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can_bmu_pb, &filter_bmu);
    }
    void handle_can(zcan_frame &frame) {
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
        // 0=OFF, 1=ON
        if (enable) {
            gpio_set_value(pwr_control_wheel_motor, 1);
            k_msleep(1000);
            gpio_set_value(pwr_control_peripheral, 1);
            k_msleep(1000);
            gpio_set_value(pwr_control_24v, 1);
        } else {
            gpio_set_value(pwr_control_wheel_motor, 0);
            k_msleep(1000);
            gpio_set_value(pwr_control_peripheral, 0);
            k_msleep(1000);
            gpio_set_value(pwr_control_24v, 0);
        }
    }
    bool is_ok() {
        // 0:OK, 1:NG
        bool rtn = (gpio_get_value(pgood_24v) == 0)                  
            && (gpio_get_value(pgood_peripheral) == 0)
            && (gpio_get_value(pgood_wheel_motor_left) == 0)
            && (gpio_get_value(pgood_wheel_motor_right) == 0);
        return rtn;
    }
    void get_failed_state(bool &v24, bool &v_peripheral, bool &v_wheel_motor_left, bool &v_wheel_motor_right) {
        // 0:OK, 1:NG
        v24 = gpio_get_value(pgood_24v) == 1;
        v_peripheral = gpio_get_value(pgood_peripheral) == 1;
        v_wheel_motor_left = gpio_get_value(pgood_wheel_motor_left) == 1;
        v_wheel_motor_right = gpio_get_value(pgood_wheel_motor_right) == 1;
    }
private:
};

class fan_driver { // Variables Implemented
public:
    void init() {
        fan_on();
    }
    void fan_on() {
        gpio_set_value(fan_pwm_5v_1, 1);    // 1:ON, 0:OFF
        gpio_set_value(fan_pwm_5v_2, 1);    // 1:ON, 0:OFF
        gpio_set_value(fan_pwm_24v_1, 1);   // 1:ON, 0:OFF
        gpio_set_value(fan_pwm_24v_2, 1);   // 1:ON, 0:OFF
    }
    void fan_off() {
        gpio_set_value(fan_pwm_5v_1, 0);    // 1:ON, 0:OFF
        gpio_set_value(fan_pwm_5v_2, 0);    // 1:ON, 0:OFF
        gpio_set_value(fan_pwm_24v_1, 0);   // 1:ON, 0:OFF
        gpio_set_value(fan_pwm_24v_2, 0);   // 1:ON, 0:OFF
    }
private:
};

class boardstate_controller {  // No pins declared
public:
    void init() {
        // can.register_callback(0x201, callback(this, &mainboard_controller::handle_can));
        // TODO メッセージキューを初期化
    }
    void poll() {
        msg_rcv_pb msg;
        if (k_msgq_get(&can_controller::msgq_board, &msg, K_NO_WAIT) == 0) {
            handle_board(msg);
        }
        // if (timer.elapsed_time() > 3s) {
        //     heartbeat_timeout = true;
        //     timer.stop();
        //     timer.reset();
        // }
    }
    bool emergency_stop_from_ros() const {
        return emergency_stop;
    }
    bool power_off_from_ros() const {
        return power_off;
    }
    bool is_dead() const {
        if (heartbeat_detect)
            return heartbeat_timeout || ros_heartbeat_timeout;
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
    //メッセージキューに変更 msgq_board
    void handle_board(const msg_rcv_pb &msg) {
        heartbeat_timeout = false;
        // timer.reset();
        // timer.start();
        emergency_stop = msg.ros_emergency_stop;
        power_off = msg.ros_power_off;
        ros_heartbeat_timeout = msg.ros_heartbeat_timeout;
        wheel_poweroff = msg.ros_wheel_power_off;
        if (!ros_heartbeat_timeout)
            heartbeat_detect = true;
    }
    // Timer timer;
    bool heartbeat_timeout{true}, heartbeat_detect{false}, ros_heartbeat_timeout{false}, emergency_stop{true}, power_off{false},
          wheel_poweroff{false};
};

#define WDT_TIMEOUT_MS 10000
class state_controller { // Variables Implemented
public:
    void init() {
        mc.init();
        ac.init();
        bmu.init();
        fan.init();

        k_timer_init(&timer_poll_20ms, static_poll_20ms_callback, NULL);
        k_timer_user_data_set(&timer_poll_20ms, this);
        k_timer_start(&timer_poll_20ms, K_MSEC(20), K_MSEC(20));

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

        // Setup Watch Dog Timer
        dev_wdi = device_get_binding("IWDG");
        if (!device_is_ready(dev_wdi)){
            LOG_INF("Watchdog device is not ready\n");
            return;
        }

        struct wdt_timeout_cfg wdt_config;
        wdt_config.flags = WDT_FLAG_RESET_SOC;
        wdt_config.window.min = 0;
        wdt_config.window.max = WDT_TIMEOUT_MS;
        wdt_config.callback = NULL;


        int wdt_channel_id = wdt_install_timeout(dev_wdi, &wdt_config);
        if (wdt_channel_id < 0) {
            LOG_INF("WDT install timeout error\n");
            return;
        }
        
        int err_setup = wdt_setup(dev_wdi, WDT_OPT_PAUSE_HALTED_BY_DBG);
        if (err_setup) {
            LOG_INF("WDT setup error: %d\n", err_setup);
            return;
        }
    }
    void run() {
        while(true) {
            poll();
        }
    }
private:
    static void static_poll_20ms_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<state_controller*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->poll();
        }
    }
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
    static void static_current_check_timeout_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<state_controller*>(k_timer_user_data_get(timer_id));
        if (instance) {
            instance->charge_guard_asserted = false;
        }
    }
    static void static_charge_guard_timeout_callback(struct k_timer *timer_id) {
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
    };
    void poll() {
        auto wheel_relay_control = [&](){
            bool wheel_poweroff{mbd.is_wheel_poweroff()};   // TODO mbd.is_wheel_poweroff() はLexxAutoからのモーターOFF指令
            if (last_wheel_poweroff != wheel_poweroff) {
                last_wheel_poweroff = wheel_poweroff;
                gpio_set_value(pwr_control_wheel_motor, wheel_poweroff ? 0 : 1);
                LOG_DBG("wheel power control %d!\n", wheel_poweroff);
            }
        };
        psw.poll();
        bsw.poll();
        esw.poll();
        mc.poll();
        ac.poll();
        bmu.poll();
        // TODO mbd の動作をリプレースする
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
                    if ((k_uptime_get() - timer_shutdown)> 60000)
                        set_new_state(POWER_STATE::OFF);
                } else {
                    LOG_DBG("wait shutdown\n");
                    wait_shutdown = true;
                    gpio_set_value(pwr_control_wheel_motor, 0);
                    timer_shutdown = k_uptime_get();    // timer reset
                    if (psw_state == power_switch::STATE::PUSHED)
                        shutdown_reason = SHUTDOWN_REASON::SWITCH;
                    if (mbd.power_off_from_ros())
                        shutdown_reason = SHUTDOWN_REASON::ROS;
                    if (!bmu.is_ok())
                        shutdown_reason = SHUTDOWN_REASON::BMU;
                }
            } else if (!esw.asserted() && !mbd.emergency_stop_from_ros() && mbd.is_ready()) {
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
            } else if (esw.asserted()) {
                LOG_DBG("emergency switch asserted\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG_DBG("receive emergency stop from ROS\n");
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
            } else if (esw.asserted()) {
                LOG_DBG("emergency switch asserted\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG_DBG("receive emergency stop from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
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
        int bat_out_state{mbd.is_wheel_poweroff() ? 0 : 1}; // TODO mbd.is_wheel_poweroff() はLexxAutoからのモーターOFF指令
        switch (newstate) {
        case POWER_STATE::OFF:
            LOG_DBG("enter OFF\n");
            poweron_by_switch = false;
            psw.set_led(false);
            dcdc.set_enable(false);
            gpio_set_value(pwr_control_wheel_motor, 0);
            while (true) // wait power off
                continue;
            break;
        case POWER_STATE::TIMEROFF:
            LOG_DBG("enter TIMEROFF\n");
            timer_poweroff = k_uptime_get();    // timer reset
            break;
        case POWER_STATE::WAIT_SW:
            LOG_DBG("enter WAIT_SW\n");
            break;
        case POWER_STATE::POST:
            LOG_DBG("enter POST\n");
            psw.set_led(true);
            gpio_set_value(pwr_control_wheel_motor, 0);
            timer_post = k_uptime_get();    // timer reset
            break;
        case POWER_STATE::STANDBY:
            LOG_DBG("enter STANDBY\n");
            psw.set_led(true);
            dcdc.set_enable(true);
            wsw.set_disable(true);
            gpio_set_value(pwr_control_wheel_motor, bat_out_state);
            ac.set_enable(false);
            wait_shutdown = false;
            break;
        case POWER_STATE::NORMAL:
            LOG_DBG("enter NORMAL\n");
            wsw.set_disable(false);
            gpio_set_value(pwr_control_wheel_motor, bat_out_state);
            ac.set_enable(false);
            charge_guard_asserted = true;
            k_timer_start(&charge_guard_timeout, K_MSEC(10000), K_NO_WAIT); // charge_guard_asserted = false after 10sec
            break;
        case POWER_STATE::AUTO_CHARGE:
            LOG_DBG("enter AUTO_CHARGE\n");
            ac.set_enable(true);
            current_check_enable = false;
            k_timer_start(&current_check_timeout, K_MSEC(10000), K_NO_WAIT); // current_check_timeout = true after 10sec
            break;
        case POWER_STATE::MANUAL_CHARGE:
            LOG_DBG("enter MANUAL_CHARGE\n");
            wsw.set_disable(true);
            gpio_set_value(pwr_control_wheel_motor, 0);
            ac.set_enable(false);
            break;
        case POWER_STATE::LOCKDOWN:
            LOG_DBG("enter LOCKDOWN\n");
            wsw.set_disable(true);
            gpio_set_value(pwr_control_wheel_motor, 0);
            ac.set_enable(false);
            break;
        }
        state = newstate;
    }
    void poll_100ms() {
        // TODO ここで can_contoroller にデータを送信
        can_controller::msg_board msg;

        // auto temperature{temp.get_temperature()};
        // if (state == POWER_STATE::AUTO_CHARGE ||
        //     state == POWER_STATE::MANUAL_CHARGE) {
        //     fan.control_by_duty(100);
        // } else {
        //     fan.control_by_temperature(temperature);
        // }
        // uint8_t buf[8]{0};
        // if (psw.get_raw_state())
        //     buf[0] |= 0b00000001;
        bool st0, st1, st2;
        esw.get_raw_state(st0, st1);
        // if (st0)
        //     buf[0] |= 0b00000010;
        // if (st1)
        //     buf[0] |= 0b00000100;
        bsw.get_raw_state(st0, st1);
        // if (st0)
        //     buf[0] |= 0b00001000;
        // if (st1)
        //     buf[0] |= 0b00010000;
        // if (mc.is_plugged())
        //     buf[1] |= 0b00000001;
        // if (ac.is_docked())
        //     buf[1] |= 0b00000010;
        // buf[1] |= (static_cast<uint32_t>(shutdown_reason) & 0x1f) << 2;
        // if (wait_shutdown)
        //     buf[1] |= 0b10000000;
        // dcdc.get_failed_state(st0, st1);
        // if (st0)
        //     buf[2] |= 0b00000001;
        // if (st1)
        //     buf[2] |= 0b00000010;
        bmu.get_fet_state(st0, st1, st2);
        // if (st0)
        //     buf[2] |= 0b00010000;
        // if (st1)
        //     buf[2] |= 0b00100000;
        // if (st2)
        //     buf[2] |= 0b01000000;
        wsw.get_raw_state(st0, st1);
        // if (st0)
        //     buf[3] |= 0b00000001;
        // if (st1)
        //     buf[3] |= 0b00000010;
        // buf[3] |= static_cast<uint32_t>(state) << 2;
        // int t0, t1;
        // ac.get_connector_temperature(t0, t1);
        // buf[4] = fan.get_duty_percent();
        // buf[5] = t0;
        // buf[6] = t1;
        // buf[7] = temperature;
        // can.send(CANMessage{0x200, buf});
        // ThisThread::sleep_for(1ms);
        // buf[0] = psw.is_activated_battery() ? 1 : 0;
        // can.send(CANMessage{0x202, buf, 1});
        // ThisThread::sleep_for(1ms);
        if (state == POWER_STATE::LOCKDOWN)
            psw.toggle_led();
        // uint32_t v{ac.get_connector_voltage()};
        // buf[0] = v;
        // buf[1] = v >> 8;
        // buf[2] = ac.get_connect_check_count();
        // buf[3] = ac.get_heartbeat_delay();
        // buf[4] = ac.is_temperature_error();
        // can.send(CANMessage{0x204, buf, 5});

        // TODO メッセージキューで can_controller へ送る
    }
    void poll_1s() {
        if (!device_is_ready(dev_wdi)){
            LOG_INF("Watchdog device is not ready\n");
            return;
        }
        wdt_feed(dev_wdi, 0);   // Feed the watchdog, Second value will not be used for STM32
    }
    
    power_switch psw;
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

    int64_t timer_post{0}, timer_shutdown{0}, timer_poweroff{0};
    k_timer timer_poll_20ms, timer_poll_100ms, timer_poll_1s;
    k_timer current_check_timeout, charge_guard_timeout;
    const device *dev_wdi{nullptr};
    bool poweron_by_switch{false}, wait_shutdown{false}, current_check_enable{false}, charge_guard_asserted{false},
         last_wheel_poweroff{false};
} impl;

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread thread;
// k_msgq msgq_can_bmu_pb;
k_msgq msgq_board_pb_rx, msgq_board_pb_tx;

}



// vim: set expandtab shiftwidth=4:
