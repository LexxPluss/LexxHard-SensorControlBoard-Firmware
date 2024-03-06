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

#include "mbed.h"
#include "serial_message.hpp"

namespace {

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
BufferedSerial debugserial{PA_2, PA_3};
FILE *debugout{fdopen(&debugserial, "r+")};
#define LOG(...) fprintf(debugout, __VA_ARGS__)
#else
#define LOG(...) logger.print(__VA_ARGS__)
#endif

/*Switched*/ PinName can_tx{PA_12}, can_rx{PA_11}; // Can associated pins
/*Half-Changed*/ PinName ps_sw_in{PB_0}, ps_led_out{PB_12}; // Power Switch handler associated pins
/*Changed*/ PinName bp_left{PA_4}; // Bumper Switch associated pins
/*Same*/ PinName es_left{PA_6}, es_right(PA_7); // Emergency Switch associated pins
/*Same*/ PinName wh_left{PB_8}, wh_right{PB_9}; // Wheel switch associated pins
/*Same*/ PinName mc_din{PB_10}; // Manual charging detection associated pins
/*Th pins changed*/ PinName ac_th_pos{PA_0}, ac_th_neg{PA_1}, ac_IrDA_tx{PA_2}, ac_IrDA_rx{PA_3}, ac_analogVol{PB_1}, ac_chargingRelay{PB_2}; // Auto charging detection associated pins
/*Changed (not PB_11)*/ PinName bmu_main_sw{PB_11}, bmu_c_fet{PB_13}, bmu_d_fet{PB_14}, bmu_p_dsg{PB_15}; // BMU controller associated pins
/*Same*/ PinName ts_i2c_scl{PB_6}, ts_i2c_sda{PB_7}; // Temperature sensors associated I2C pins
/*Switched*/ PinName dcdc_control_16v{PB_3}, dcdc_control_5v{PA_10}, dcdc_failSignal_16v{PB_4}, dcdc_failSignal_5v{PA_15}; // DC-DC related control and fail signal pins
/*Same*/ PinName fan_pwm{PA_8}; // PWM fan signal control pin
/*Same*/ PinName sc_bat_out{PB_5}, sc_hb_led{PA_9}; // State controller associated pins
PinName main_MCU_ON{PA_5}; // Pin controlling the MainMCU power-up

EventQueue globalqueue;

class power_switch_handler {  // No pins declared
public:
    power_switch_handler(int thres) : thres(thres * 2) {
        timer.start();
    }
    void poll(bool changed) {
        if (changed) {
            activated = ++counter >= thres;
            timer.reset();
        }
        auto elapsed{timer.elapsed_time()};
        if (elapsed > 1s) {
            counter = 0;
            activated = false;
        }
    }
    bool is_activated() const {
        return activated;
    }
private:
    Timer timer;
    int thres, counter{0};
    bool activated{false};
};

class power_switch { // Variables Implemented
public:
    enum class STATE {
        RELEASED, PUSHED, LONG_PUSHED,
    };
    void poll() {
        int now{sw.read()};
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
                timer.reset();
                timer.start();
            } else if (now == 0) {
                auto elapsed{timer.elapsed_time()};
                if (elapsed > 60s) {
                    if (state != STATE::LONG_PUSHED)
                        state = STATE::LONG_PUSHED;
                } else if (elapsed > 3s) {
                    if (state == STATE::RELEASED)
                        state = STATE::PUSHED;
                }
            }
        }
    }
    void reset_state() {
        timer.stop();
        timer.reset();
        state = STATE::RELEASED;
    }
    STATE get_state() const {return state;}
    bool get_raw_state() {
        return sw.read() == 0;
    }
    void set_led(bool enabled) {
        led.write(enabled ? 1 : 0);
    }
    void toggle_led() {
        led.write(led.read() == 0 ? 1 : 0);
    }
    bool is_activated_battery() const {
        return sw_bat.is_activated();
    }
    bool is_activated_unlock() const {
        return sw_unlock.is_activated();
    }
private:
    power_switch_handler sw_bat{2}, sw_unlock{10};
    DigitalIn sw{ps_sw_in};
    DigitalOut led{ps_led_out, 0};
    Timer timer;
    STATE state{STATE::RELEASED};
    uint32_t count{0};
    int prev{-1}, prev_raw{-1};
    static constexpr uint32_t COUNT{1};
};

class bumper_switch { // Variables Implemented
public:
    void poll() {
        if (left.read() == 0) {
            asserted = true;
            timeout.attach([this](){asserted = false;}, 1s);
        }
    }
    void get_raw_state(bool &left, bool &right) const {
        left = right = asserted;
#ifdef SERIAL_DEBUG
        static bool prev{false};
        if (prev != asserted) {
            prev = asserted;
            LOG("BUMPER %s!\n", asserted ? "ON" : "OFF");
        }
#endif
    }
private:
    DigitalIn left{bp_left};
    Timeout timeout;
    bool asserted{false};
};

class emergency_switch { // Variables Implemented
public:
    void poll() {
        int now{left.read()};
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
        now = right.read();
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
    DigitalIn left{es_left}, right{es_right};
    uint32_t left_count{0}, right_count{0};
    int left_prev{-1}, right_prev{-1};
    bool left_asserted{false}, right_asserted{false};
    static constexpr uint32_t COUNT{5};
};

class wheel_switch { // Variables Implemented
public:
    void set_disable(bool disable) {
        if (disable) {
            left.write(1);
            right.write(1);
        } else {
            left.write(0);
            right.write(0);
        }
    }
    void get_raw_state(bool &left, bool &right) {
        left = this->left.read() == 1;
        right = this->right.read() == 1;
    }
private:
    DigitalOut left{wh_left, 0}, right{wh_right, 0};
};

class manual_charger { // Variables Implemented
public:
    void init() {
        setup_first_state();
    }
    void poll() {
        int now{din.read()};
        if (prev != now) {
            prev = now;
            timer.reset();
            timer.start();
        } else {
            auto elapsed{timer.elapsed_time()};
            if (elapsed > 100ms) {
                plugged = now == 0;
                timer.stop();
                timer.reset();
            }
        }
    }
    bool is_plugged() {return plugged;}
private:
    void setup_first_state() {
        int plugped_count{0};
        for (int i{0}; i < 10; ++i) {
            if (din.read() == 0)
                ++plugped_count;
            ThisThread::sleep_for(5ms);
        }
        plugged = plugped_count > 5;
    }
    DigitalIn din{mc_din};
    Timer timer;
    int prev{1};
    bool plugged{false};
};

class auto_charger { // Variables Half-Implemented (Not Thermistors ADC)
public:
    void init() {
#ifndef SERIAL_DEBUG
        serial.set_baud(4800);
        serial.set_format(8, SerialBase::None, 1);
        serial.set_blocking(false);
#endif
        globalqueue.call_every(1s, this, &auto_charger::poll_1s);
        heartbeat_timer.start();
        serial_timer.start();
    }
    bool is_docked() const {
        return is_connected() && !temperature_error && !is_overheat() && heartbeat_timer.elapsed_time() < 5s;
        // return is_connected() && !temperature_error && !is_overheat() && heartbeat_timer.elapsed_time() < 5s;
        // return is_charger_ready() && is_connected() && !temperature_error && !is_overheat() && heartbeat_timer.elapsed_time() < 5s;
    }
    void set_enable(bool enable) {
        sw.write(enable ? 1 : 0);
    }
    void force_stop() {
        set_enable(false);
        send_heartbeat();
    }
    bool is_charger_ready() const {
        if(connector_v > (CHARGING_VOLTAGE * 0.7)){
            LOG("charger ready voltage:%f.\n", connector_v);
            return true;
        }else {
            LOG("connector_v:%f THRESH:%f\n", connector_v, (CHARGING_VOLTAGE * 0.9));
            return false;
        }
    }
    void get_connector_temperature(int &positive, int &negative) const {
        positive = clamp(static_cast<int>(connector_temp[0]), 0, 255);
        negative = clamp(static_cast<int>(connector_temp[1]), 0, 255);
    }
    uint32_t get_connector_voltage() const {
        int32_t voltage_mv{static_cast<int32_t>(connector_v * 1e+3f)};
        return clamp(voltage_mv, 0L, 3300L);
    }
    uint32_t get_connect_check_count() const {return connect_check_count;}
    uint32_t get_heartbeat_delay() const {
        auto seconds{std::chrono::duration_cast<std::chrono::seconds>(heartbeat_timer.elapsed_time())};
        return clamp(static_cast<uint32_t>(seconds.count()), 0UL, 255UL);
    }
    bool is_temperature_error() const {return temperature_error;}
    void poll() {
        uint32_t prev_connect_check_count{connect_check_count};
        connector_v = connector.read_voltage();  // Read the voltage from the auto charging terminals
        if (connector_v > CONNECT_THRES_VOLTAGE) {
            if (++connect_check_count >= CONNECT_THRES_COUNT) {
                connect_check_count = CONNECT_THRES_COUNT;
                if (prev_connect_check_count < CONNECT_THRES_COUNT)
                    LOG("connected to the charger.\n");
            }
        } else {
            connect_check_count = 0;
            if (prev_connect_check_count >= CONNECT_THRES_COUNT)
                LOG("disconnected from the charger.\n");
        }
#ifndef SERIAL_DEBUG
        while (serial.readable()) {
            if (serial_timer.elapsed_time() > 1s)
                msg.reset();
            serial_timer.reset();
            uint8_t data;
            serial.read(&data, 1);
            if (msg.decode(data)) {
                uint8_t param[3];
                uint8_t command{msg.get_command(param)};
                if (command == serial_message::HEARTBEAT && param[0] == heartbeat_counter)
                    heartbeat_timer.reset();
            }
        }
#endif
        adc_read();
    }
    void update_rsoc(uint8_t rsoc) {
        this->rsoc = rsoc;
    }
private: // Thermistor side starts here.
    void adc_read() { // Change to read the temperature sensor from ADC pin directly. Thermistor side.
        float v_th_pos{therm_pos.read_voltage()}; // Read the positive thermistor voltage
        float v_th_neg{therm_neg.read_voltage()}; // Read the negative thermistor voltage
        calculate_temperature(v_th_pos, 0); // Calculate the thermistor PLUS temperature
        calculate_temperature(v_th_neg, 1); // Calculate the thermistor MINUS temperature
    }
    void calculate_temperature(float adc_voltage, uint8_t sensor) { // Changed version for direct ADC measurements
        adc_voltage = clamp(adc_voltage, 0.0f, 3.29999f); // Clamp the value of the adc voltage received
        // see https://lexxpluss.esa.io/posts/459
        static constexpr float R0{3300.0f}, B{3970.0f}, T0{373.0f};
        float Rpu{10000.0f};
        float R{Rpu * adc_voltage / (3.3f - adc_voltage)};
        float T{1.0f / (logf(R / R0) / B + 1.0f / T0)};
        static constexpr float gain{0.02f}; // Low pass filter gain
        connector_temp[sensor] = connector_temp[sensor] * (1.0f - gain) + (T - 273.0f) * gain; // Low pass filter function
    }
    bool is_connected() const {
        return connect_check_count >= CONNECT_THRES_COUNT;
    }
    bool is_overheat() const {
        return connector_temp[0] > 80.0f || connector_temp[1] > 80.0f;
    }
    void poll_1s() {                                                         /* Function that checks the conditions of charging while the IrDA is connected */
        if (is_connected() && !temperature_error && !is_overheat())
            send_heartbeat();
    }
    void send_heartbeat() {                                                    /* Creates the message to send to the robot using the "compose" function below */
        // uint8_t buf[8], param[3]{++heartbeat_counter, static_cast<uint8_t>(sw.read()), rsoc}; // Message composed of 8 bytes, 3 bytes parameters -- Declaration
        uint8_t sw_state{0};

        if(is_connected()){
            sw_state = 1;
        }else{
            sw_state = 0;
        }
         
        uint8_t buf[8], param[3]{++heartbeat_counter, sw_state, rsoc}; // Message composed of 8 bytes, 3 bytes parameters -- Declaration
        serial_message::compose(buf, serial_message::HEARTBEAT, param);
#ifndef SERIAL_DEBUG
        serial.write(buf, sizeof buf);
#endif
    } // Declaration of variables
#ifndef SERIAL_DEBUG
    BufferedSerial serial{ac_IrDA_tx, ac_IrDA_rx}; // IrDA serial pins
#endif
    AnalogIn connector{ac_analogVol, 3.3f}; // Charging connector pin 0 - 24V. (3.3f max voltage reference - map voltage between 0 - 3.3V)
    AnalogIn therm_pos{ac_th_pos, 3.3f}; // Charging connector pin where the input is 0 - 24V. (map voltage between 0 - 3.3V)
    AnalogIn therm_neg{ac_th_neg, 3.3f}; // Charging connector pin where the input is 0 - 24V. (map voltage between 0 - 3.3V)
    DigitalOut sw{ac_chargingRelay, 0}; // declare the robot auto Charging relay pin!!
    Timer heartbeat_timer, serial_timer;
    serial_message msg;
    uint8_t heartbeat_counter{0}, rsoc{0};
    float connector_v{0.0f}, connector_temp[2]{0.0f, 0.0f};
    uint32_t connect_check_count{0}, temperature_error_count{0};
    bool temperature_error{false};
    static constexpr int ADDR{0b10010010}; // I2C adress for temp sensor
    static constexpr uint32_t CONNECT_THRES_COUNT{100}; // Number of times that ...
    static constexpr float CHARGING_VOLTAGE{30.0f * 1000.0f / (9100.0f + 1000.0f)},
                           CONNECT_THRES_VOLTAGE{3.3f * 0.5f * 1000.0f / (9100.0f + 1000.0f)};
};

class bmu_controller { // Variables Implemented
public:
    bmu_controller(can_driver &can) : can(can) {}
    void init() {
        for (auto i : {0x100, 0x101, 0x113})
            can.register_callback(i, callback(this, &bmu_controller::handle_can));
    }
    void set_enable(bool enable) {main_sw = enable ? 1 : 0;}
    bool is_ok() const {
        return ((data.mod_status1 & 0b10111111) == 0 ||
                (data.mod_status2 & 0b11100001) == 0 ||
                (data.bmu_alarm1  & 0b11111111) == 0 ||
                (data.bmu_alarm2  & 0b00000001) == 0);
    }
    void get_fet_state(bool &c_fet, bool &d_fet, bool &p_dsg) {
        c_fet = this->c_fet.read() == 1;
        d_fet = this->d_fet.read() == 1;
        p_dsg = this->p_dsg.read() == 1;
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
    void handle_can(const CANMessage &msg) {
        switch (msg.id) {
        case 0x100:
            data.mod_status1 = msg.data[0];
            data.asoc = msg.data[2];
            data.rsoc = msg.data[3];
            break;
        case 0x101:
            data.mod_status2 = msg.data[6];
            data.pack_a = (msg.data[0] << 8) | msg.data[1];
            data.pack_v = (msg.data[4] << 8) | msg.data[5];
            break;
        case 0x113:
            data.bmu_alarm1 = msg.data[4];
            data.bmu_alarm2 = msg.data[5];
            break;
        }
    }
    can_driver &can;
    DigitalOut main_sw{bmu_main_sw, 0}; // MAIN_SW switch pin
    DigitalIn c_fet{bmu_c_fet}, d_fet{bmu_d_fet}, p_dsg{bmu_p_dsg};
        struct {
        int16_t pack_a{0};
        uint16_t pack_v{0};
        uint8_t mod_status1{0xff}, mod_status2{0xff}, bmu_alarm1{0xff}, bmu_alarm2{0xff};
        uint8_t asoc{0}, rsoc{0};
    } data;
};

class dcdc_converter { // Variables Implemented
public:
    // 電源のEnableをここでやる
    void set_enable(bool enable) {
        if (enable) {               // In this configuration 0=OFF, 1=ON
            control[0].write(1);    // external 5V must be turned on first.
            control[1].write(1);
            //control[2].write(1);    // control[2] controls the relay of the MAIN_SW of the BMU.
            control[2].write(1);    // control[3] controls the relay that powers ON the main MCU
        } else {
            control[1].write(0);    // 16V must be turned off first.
            control[0].write(0);
            // control[2].write(0);
            control[2].write(0);
        }
    }
    bool is_ok() {
        return fail[0].read() != 0 && fail[1].read() != 0;
    }
    void get_failed_state(bool &v5, bool &v16) {
        v5 = fail[0].read() == 0;
        v16 = fail[1].read() == 0;
    }
private:
    DigitalOut control[3]{{dcdc_control_5v, 0}, {dcdc_control_16v, 0}, {main_MCU_ON, 0}};
    DigitalIn fail[2]{dcdc_failSignal_5v, dcdc_failSignal_16v};
};

class fan_driver { // Variables Implemented
/* FanをONにするファンクション */
// public:
//     void init() {
//         pwm.period_us(1000000 / CONTROL_HZ);
//         pwm.pulsewidth_us(0);
//     }
//     void control_by_temperature(float temperature) {
//         static constexpr float temp_min{15.0f}, temp_l{20.0f}, temp_h{30.0f};
//         static constexpr int duty_l{10}, duty_h{100};
//         static constexpr float A{(duty_h - duty_l) / (temp_h - temp_l)};
//         int duty_percent;
//         if (temperature < temp_min)
//             duty_percent = 0;
//         else if (temperature < temp_l)
//             duty_percent = duty_l;
//         else if (temperature > temp_h)
//             duty_percent = duty_h;
//         else
//             duty_percent = A * (temperature - temp_l) + duty_l; // Linearly interpolate between temp_l and temp_h.
//         control_by_duty(duty_percent);
//     }
//     void control_by_duty(int duty_percent) {
//         int pulsewidth{duty_percent * 1000000 / 100 / CONTROL_HZ};
//         pwm.pulsewidth_us(pulsewidth);
//     }
//     int get_duty_percent() {
//         return pwm.read_pulsewitdth_us() * CONTROL_HZ * 100 / 1000000;
//     }
// private:
//     PwmOut pwm{fan_pwm};
//     static constexpr int CONTROL_HZ{5000};
};

class state_controller { // Variables Implemented
public:
    void init() {
        mc.init();
        ac.init();
        bmu.init();
        fan.init();

        globalqueue.call_every(20ms, this, &state_controller::poll); //違う周期のポーリングをどうするか？
        globalqueue.call_every(100ms, this, &state_controller::poll_100ms);
        globalqueue.call_every(1s, this, &state_controller::poll_1s);
        globalqueue.call_every(10s, this, &state_controller::poll_10s);

        Watchdog &watchdog{Watchdog::get_instance()}; //ZephyrでWatchdogはどうやるのか？
        uint32_t watchdog_max{watchdog.get_max_timeout()};
        uint32_t watchdog_timeout{10000U};
        if (watchdog_timeout > watchdog_max)
            watchdog_timeout = watchdog_max;
        watchdog.start(watchdog_timeout);
    }
private:
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
            bool wheel_poweroff{mbd.is_wheel_poweroff()};
            if (last_wheel_poweroff != wheel_poweroff) {
                last_wheel_poweroff = wheel_poweroff;
                bat_out.write(wheel_poweroff ? 0 : 1);
                LOG("wheel power control %d!\n", wheel_poweroff);
            }
        };
        psw.poll();
        bsw.poll();
        esw.poll();
        mc.poll();
        ac.poll();

        switch (state) {
        case POWER_STATE::OFF:
            set_new_state(mc.is_plugged() ? POWER_STATE::POST : POWER_STATE::WAIT_SW);
            break;
        case POWER_STATE::TIMEROFF:
            if (timer_poweroff.elapsed_time() > 5s)
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
                LOG("unplugged from manual charger\n");
                set_new_state(POWER_STATE::OFF);
            } else if (bmu.is_ok() && temp.is_ok()) {
                LOG("BMU and temperature OK\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (timer_post.elapsed_time() > 3s) {
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
            } else if (psw_state == power_switch::STATE::PUSHED || mbd.power_off_from_ros() ||
                mbd.is_overheat() || !bmu.is_ok() || !temp.is_ok()) {
                if (wait_shutdown) {
                    if (timer_shutdown.elapsed_time() > 60s)
                        set_new_state(POWER_STATE::OFF);
                } else {
                    LOG("wait shutdown\n");
                    wait_shutdown = true;
                    bat_out.write(0);
                    timer_shutdown.reset();
                    timer_shutdown.start();
                    if (psw_state == power_switch::STATE::PUSHED)
                        shutdown_reason = SHUTDOWN_REASON::SWITCH;
                    if (mbd.power_off_from_ros())
                        shutdown_reason = SHUTDOWN_REASON::ROS;
                    if (!bmu.is_ok())
                        shutdown_reason = SHUTDOWN_REASON::BMU;
                }
            } else if (!esw.asserted() && !mbd.emergency_stop_from_ros() && mbd.is_ready()) {
                LOG("not emergency and heartbeat OK\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (mc.is_plugged()) {
                LOG("plugged to manual charger\n");
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            }
            break;
        }
        case POWER_STATE::NORMAL:
            wheel_relay_control();
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG("detect power switch\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.power_off_from_ros()) {
                LOG("receive power off from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!bmu.is_ok()) {
                LOG("BMU failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!dcdc.is_ok()) {
                LOG("DCDC failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (esw.asserted()) {
                LOG("emergency switch asserted\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG("receive emergency stop from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mc.is_plugged()) {
                LOG("plugged to manual charger\n");
                set_new_state(POWER_STATE::MANUAL_CHARGE);
            } else if (!charge_guard_asserted && ac.is_docked() && bmu.is_chargable()) {
                if(ac.is_charger_ready() == true){
                    LOG("docked to auto charger\n");
                    set_new_state(POWER_STATE::AUTO_CHARGE);
                }
            }
            break;
        case POWER_STATE::AUTO_CHARGE:
            ac.update_rsoc(bmu.get_rsoc());
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG("detect power switch\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.power_off_from_ros()) {
                LOG("receive power off from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!bmu.is_ok()) {
                LOG("BMU failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (!dcdc.is_ok()) {
                LOG("DCDC failure\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (esw.asserted()) {
                LOG("emergency switch asserted\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.emergency_stop_from_ros()) {
                LOG("receive emergency stop from ROS\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (mbd.is_dead()) {
                LOG("main board or ROS dead\n");
                set_new_state(POWER_STATE::STANDBY);
            } else if (bmu.is_full_charge()) {
                LOG("full charge\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (!ac.is_docked()) {
                LOG("undocked from auto charger\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (mc.is_plugged()) {
                LOG("manual charger plugged\n");
                set_new_state(POWER_STATE::NORMAL);
            } else if (current_check_enable && !bmu.is_charging()) {
                LOG("not charging\n");
                set_new_state(POWER_STATE::NORMAL);
            }
            break;
        case POWER_STATE::MANUAL_CHARGE:
            if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG("detect power switch (ignored)\n");
                psw.reset_state();
            }
            if (!mc.is_plugged()) {
                LOG("unplugged from manual charger\n");
                set_new_state(POWER_STATE::NORMAL);
            }
            break;
        case POWER_STATE::LOCKDOWN:
            if (!dcdc.is_ok()) {
                LOG("DCDC failure\n");
                set_new_state(POWER_STATE::OFF);
            } else if (psw.get_state() != power_switch::STATE::RELEASED) {
                LOG("detect power switch\n");
                set_new_state(POWER_STATE::OFF);
            } else if (psw.is_activated_unlock()) {
                LOG("force recover from lockdown\n");
                set_new_state(POWER_STATE::STANDBY);
            }
            break;
        }
    }
    void set_new_state(POWER_STATE newstate) {
        switch (state) {
        case POWER_STATE::NORMAL:
            charge_guard_timeout.detach();
            break;
        case POWER_STATE::AUTO_CHARGE:
            current_check_timeout.detach();
            ac.force_stop();
            break;
        default:
            break;
        }
        int bat_out_state{mbd.is_wheel_poweroff() ? 0 : 1};
        switch (newstate) {
        case POWER_STATE::OFF:
            LOG("enter OFF\n");
            poweron_by_switch = false;
            psw.set_led(false);
            dcdc.set_enable(false);
            bmu.set_enable(false);
            bat_out.write(0);
            while (true) // wait power off
                continue;
            break;
        case POWER_STATE::TIMEROFF:
            LOG("enter TIMEROFF\n");
            timer_poweroff.reset();
            timer_poweroff.start();
            break;
        case POWER_STATE::WAIT_SW:
            LOG("enter WAIT_SW\n");
            break;
        case POWER_STATE::POST:
            LOG("enter POST\n");
            psw.set_led(true);
            bmu.set_enable(true);
            bat_out.write(0);
            timer_post.reset();
            timer_post.start();
            break;
        case POWER_STATE::STANDBY:
            LOG("enter STANDBY\n");
            psw.set_led(true);
            dcdc.set_enable(true);
            wsw.set_disable(true);
            bat_out.write(bat_out_state);
            ac.set_enable(false);
            wait_shutdown = false;
            break;
        case POWER_STATE::NORMAL:
            LOG("enter NORMAL\n");
            wsw.set_disable(false);
            bat_out.write(bat_out_state);
            ac.set_enable(false);
            charge_guard_asserted = true;
            charge_guard_timeout.attach([this](){charge_guard_asserted = false;}, 10s);
            break;
        case POWER_STATE::AUTO_CHARGE:
            LOG("enter AUTO_CHARGE\n");
            ac.set_enable(true);
            current_check_enable = false;
            current_check_timeout.attach([this](){current_check_enable = true;}, 10s);
            break;
        case POWER_STATE::MANUAL_CHARGE:
            LOG("enter MANUAL_CHARGE\n");
            wsw.set_disable(true);
            bat_out.write(0);
            ac.set_enable(false);
            break;
        case POWER_STATE::LOCKDOWN:
            LOG("enter LOCKDOWN\n");
            wsw.set_disable(true);
            bat_out.write(0);
            ac.set_enable(false);
            break;
        }
        state = newstate;
    }
    void poll_100ms() {
        auto temperature{temp.get_temperature()};
        if (state == POWER_STATE::AUTO_CHARGE ||
            state == POWER_STATE::MANUAL_CHARGE) {
            fan.control_by_duty(100);
        } else {
            fan.control_by_temperature(temperature);
        }
        uint8_t buf[8]{0};
        if (psw.get_raw_state())
            buf[0] |= 0b00000001;
        bool st0, st1, st2;
        esw.get_raw_state(st0, st1);
        if (st0)
            buf[0] |= 0b00000010;
        if (st1)
            buf[0] |= 0b00000100;
        bsw.get_raw_state(st0, st1);
        if (st0)
            buf[0] |= 0b00001000;
        if (st1)
            buf[0] |= 0b00010000;
        if (mc.is_plugged())
            buf[1] |= 0b00000001;
        if (ac.is_docked())
            buf[1] |= 0b00000010;
        buf[1] |= (static_cast<uint32_t>(shutdown_reason) & 0x1f) << 2;
        if (wait_shutdown)
            buf[1] |= 0b10000000;
        dcdc.get_failed_state(st0, st1);
        if (st0)
            buf[2] |= 0b00000001;
        if (st1)
            buf[2] |= 0b00000010;
        bmu.get_fet_state(st0, st1, st2);
        if (st0)
            buf[2] |= 0b00010000;
        if (st1)
            buf[2] |= 0b00100000;
        if (st2)
            buf[2] |= 0b01000000;
        wsw.get_raw_state(st0, st1);
        if (st0)
            buf[3] |= 0b00000001;
        if (st1)
            buf[3] |= 0b00000010;
        buf[3] |= static_cast<uint32_t>(state) << 2;
        int t0, t1;
        ac.get_connector_temperature(t0, t1);
        buf[4] = fan.get_duty_percent();
        buf[5] = t0;
        buf[6] = t1;
        buf[7] = temperature;
        can.send(CANMessage{0x200, buf});
        ThisThread::sleep_for(1ms);
        buf[0] = psw.is_activated_battery() ? 1 : 0;
        can.send(CANMessage{0x202, buf, 1});
        ThisThread::sleep_for(1ms);
        if (state == POWER_STATE::LOCKDOWN)
            psw.toggle_led();
        uint32_t v{ac.get_connector_voltage()};
        buf[0] = v;
        buf[1] = v >> 8;
        buf[2] = ac.get_connect_check_count();
        buf[3] = ac.get_heartbeat_delay();
        buf[4] = ac.is_temperature_error();
        can.send(CANMessage{0x204, buf, 5});
    }
    void poll_1s() {
        Watchdog &watchdog{Watchdog::get_instance()};
        watchdog.kick();
    }
    
    can_driver can;
    power_switch psw;
    bumper_switch bsw;
    emergency_switch esw;
    wheel_switch wsw;
    manual_charger mc;
    auto_charger ac;
    bmu_controller bmu{can};
    dcdc_converter dcdc;
    fan_driver fan;
    DigitalOut bat_out{sc_bat_out, 0}, heartbeat_led{sc_hb_led, 0};
    POWER_STATE state{POWER_STATE::OFF};
    enum class SHUTDOWN_REASON {
        NONE,
        SWITCH,
        ROS,
        MAINBOARD_TEMP,
        POWERBOARD_TEMP,
        BMU,
    } shutdown_reason{SHUTDOWN_REASON::NONE};
    Timer timer_post, timer_shutdown, timer_poweroff;
    Timeout current_check_timeout, charge_guard_timeout;
    bool poweron_by_switch{false}, wait_shutdown{false}, current_check_enable{false}, charge_guard_asserted{false},
         last_wheel_poweroff{false};
};

}

int main()
{
    LOG("RUN!\n");
    state_controller ctrl;
    ctrl.init();
    globalqueue.dispatch_forever();
    return 0;
}

// vim: set expandtab shiftwidth=4:
