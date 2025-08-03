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

#include <optional>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <tuple>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
#include "board_controller.hpp"
#include "common.hpp"
#include "tug_encoder_controller.hpp"

// for HW counter
// extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim_encoder)
// {
//     GPIO_InitTypeDef GPIO_InitStruct{0};
//     // if(htim_encoder->Instance == TIM1) {
//     //     __HAL_RCC_TIM1_CLK_ENABLE();
//     //     __HAL_RCC_GPIOE_CLK_ENABLE();
//     //     GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
//     //     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     //     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     //     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     //     GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
//     //     HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//     // } else if(htim_encoder->Instance == TIM3) {
//     //     __HAL_RCC_TIM3_CLK_ENABLE();
//     //     __HAL_RCC_GPIOC_CLK_ENABLE();
//     //     GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//     //     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     //     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     //     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     //     GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//     //     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//     // } else 
//     if(htim_encoder->Instance == TIM4) {
//         __HAL_RCC_TIM4_CLK_ENABLE();
//         __HAL_RCC_GPIOD_CLK_ENABLE();
//         GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
//         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//         GPIO_InitStruct.Pull = GPIO_NOPULL;
//         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//         GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//         HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//     }
// }

namespace lexxhard::actuator_controller {

LOG_MODULE_REGISTER(actuator);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];
char __aligned(4) msgq_control_buffer[8 * sizeof (msg_control)];

static constexpr uint32_t ACTUATOR_NUM{3};

struct msg_pwmtrampoline {
    bool all{false};
    int8_t index{0};
    int8_t direction{msg_control::STOP};
    uint8_t duty{0};
};
K_MSGQ_DEFINE(msgq_pwmtrampoline, sizeof (msg_pwmtrampoline), 8, 4);

enum class POS {
    CENTER, LEFT, RIGHT
};

// for HW counter
// class encoder {
// public:
//     int init(TIM_TypeDef *tim) {
//         this->tim = tim;
//         TIM_Encoder_InitTypeDef sConfig{0};
//         TIM_MasterConfigTypeDef sMasterConfig{0};
//         if (tim == TIM1) {
//             timh.Init.RepetitionCounter = 0;
//             sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//         }
//         timh.Instance = tim;
//         timh.Init.Prescaler = 0;
//         timh.Init.CounterMode = TIM_COUNTERMODE_UP;
//         timh.Init.Period = 65535;
//         timh.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//         timh.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//         sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
//         sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//         sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//         sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//         sConfig.IC1Filter = 0b1111;
//         sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//         sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//         sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//         sConfig.IC2Filter = 0b1111;
//         if (HAL_TIM_Encoder_Init(&timh, &sConfig) != HAL_OK)
//             return -1;
//         sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//         sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//         if (HAL_TIMEx_MasterConfigSynchronization(&timh, &sMasterConfig) != HAL_OK)
//             return -1;
//         HAL_TIM_Encoder_Start(&timh, TIM_CHANNEL_ALL);
//         return 0;
//     }
//     int16_t get() const {
//         int16_t count{static_cast<int16_t>(tim->CNT)};
//         tim->CNT = 0;
//         return -count;
//     }
// private:
//     TIM_TypeDef *tim{nullptr};
//     TIM_HandleTypeDef timh;
// };

#define TIMER_PERIOD_US 250
class encoder {
public:
    int init(POS pos) {
        gpio_dt_spec dev;
        switch (pos) {
        case POS::CENTER:
            // init IO dev exists for each instances
            dev = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_center_actuator_ch1), gpios);
            // PD12 CH1, PD13 CH2
            if (gpio_is_ready_dt(&dev)) {
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
            }

            dev = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_center_actuator_ch2), gpios);
            if (gpio_is_ready_dt(&dev)) {
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
            }
        
            // set callback function for center
            k_timer_init(&encoder_count_c, static_center_enc_callback, NULL);
            k_timer_user_data_set(&encoder_count_c, this);
            k_timer_start(&encoder_count_c, K_USEC(TIMER_PERIOD_US), K_USEC(TIMER_PERIOD_US));
            break;
        case POS::LEFT:
            // init IO dev exists for each instances
            dev = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_left_actuator_ch1), gpios);
            // PD12 CH1, PD13 CH2
            if (gpio_is_ready_dt(&dev)) {
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
            }

            dev = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_left_actuator_ch2), gpios);
            if (gpio_is_ready_dt(&dev)) {
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
            }

            // set callback function for LEFT
            k_timer_init(&encoder_count_l, static_center_left_callback, NULL);
            k_timer_user_data_set(&encoder_count_l, this);
            k_timer_start(&encoder_count_l, K_USEC(TIMER_PERIOD_US), K_USEC(TIMER_PERIOD_US));
            break;
        case POS::RIGHT:
        // init IO dev exists for each instances
            dev = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_right_actuator_ch1), gpios);
            // PD12 CH1, PD13 CH2
            if (gpio_is_ready_dt(&dev)) {
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
            }

            dev = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_right_actuator_ch2), gpios);
            if (gpio_is_ready_dt(&dev)) {
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
            }

            // set callback function for RIGHT
            k_timer_init(&encoder_count_r, static_center_right_callback, NULL);
            k_timer_user_data_set(&encoder_count_r, this);
            k_timer_start(&encoder_count_r, K_USEC(TIMER_PERIOD_US), K_USEC(TIMER_PERIOD_US));
            break;
        }
        
        return 0;
    }
    int16_t get() const {
        return count;
    }
    void reset() {
        ch1_prev = ch2_prev = 0;
        count = 0;
        is_rise_edge = false;
    }
private:
    static void static_center_enc_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<encoder*>(k_timer_user_data_get(timer_id));
        if (instance) {
            int ch1{0}, ch2{0};
            gpio_dt_spec dev_ch1 = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_center_actuator_ch1), gpios);
            gpio_dt_spec dev_ch2 = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_center_actuator_ch2), gpios);

            if(gpio_is_ready_dt(&dev_ch1)) {
                ch1 = gpio_pin_get_dt(&dev_ch1);
            }

            if(gpio_is_ready_dt(&dev_ch2)) {
                ch2 = gpio_pin_get_dt(&dev_ch2);
            }

            instance->count += instance->calc_delta_count(instance->ch1_prev, instance->ch2_prev, ch1, ch2);
            instance->ch1_prev = ch1;
            instance->ch2_prev = ch2;
        }
    }
    static void static_center_left_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<encoder*>(k_timer_user_data_get(timer_id));
        if (instance) {
            int ch1{0}, ch2{0};
            gpio_dt_spec dev_ch1 = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_left_actuator_ch1), gpios);
            gpio_dt_spec dev_ch2 = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_left_actuator_ch2), gpios);

            if(gpio_is_ready_dt(&dev_ch1)) {
                ch1 = gpio_pin_get_dt(&dev_ch1);
            }

            if(gpio_is_ready_dt(&dev_ch2)) {
                ch2 = gpio_pin_get_dt(&dev_ch2);
            }

            instance->count += instance->calc_delta_count(instance->ch1_prev, instance->ch2_prev, ch1, ch2);
            instance->ch1_prev = ch1;
            instance->ch2_prev = ch2;
        }
    }
    static void static_center_right_callback(struct k_timer *timer_id) {
        auto* instance = static_cast<encoder*>(k_timer_user_data_get(timer_id));
        if (instance) {
            int ch1{0}, ch2{0};
            gpio_dt_spec dev_ch1 = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_right_actuator_ch1), gpios);
            gpio_dt_spec dev_ch2 = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_right_actuator_ch2), gpios);

            if(gpio_is_ready_dt(&dev_ch1)) {
                ch1 = gpio_pin_get_dt(&dev_ch1);
            }

            if(gpio_is_ready_dt(&dev_ch2)) {
                ch2 = gpio_pin_get_dt(&dev_ch2);
            }

            instance->count += instance->calc_delta_count(instance->ch1_prev, instance->ch2_prev, ch1, ch2);
            instance->ch1_prev = ch1;
            instance->ch2_prev = ch2;
        }
    }
    inline int16_t calc_delta_count(const int prev_ch1, const int prev_ch2, const int ch1, const int ch2) const {
        // index is prev_ch1,prev_ch2,ch1,ch2
        static constexpr std::optional<int16_t> delta_table[16]{
          0,            // 00 -> 00 no change
          -1,           // 00 -> 01
          1,            // 00 -> 10
          std::nullopt, // 00 -> 11 error(ignore)
          1,            // 01 -> 00
          0,            // 01 -> 01 no change
          std::nullopt, // 01 -> 10 error(ignore)
          -1,           // 01 -> 11
          -1,           // 10 -> 00
          std::nullopt, // 10 -> 01 error(ignore)
          0,            // 10 -> 10 no change
          1,            // 10 -> 11
          std::nullopt, // 11 -> 00 error(ignore)
          1,            // 11 -> 01
          -1,           // 11 -> 10
          0             // 11 -> 11 no change
        };

        const int index = (prev_ch1 << 3) | (prev_ch2 << 2) | (ch1 << 1) | (ch2 << 0);
        const auto ret{delta_table[index]};
        if (!ret.has_value()) {
            LOG_WRN("Wrong encoder state: %d%d -> %d%d", prev_ch1, prev_ch2, ch1, ch2);
        }

        return ret.value_or(0);  // return 0 if error
    }

    k_timer encoder_count_c, encoder_count_l, encoder_count_r;
    int ch1_prev{0}, ch2_prev{0};
    int16_t count{0};
    bool is_rise_edge{false};
};

class counter {
public:
    int init(POS pos) {
        const int result{enc.init(pos)};
        mm_per_pulse = get_mm_per_pulse(pos);
        reset_pulse();
        return result;
    }
    void reset() {
        reset_pulse();
        wait_stabilize();
        reset_pulse();
    }
    void poll(uint32_t dt_ms) {
        int16_t pulse{update_pulse()};
        if (dt_ms != 0)
            velocity = static_cast<float>(pulse) * mm_per_pulse * 1000.0f / static_cast<float>(dt_ms) + 0.5f;
    }
    int32_t get_location() const {return pulse_value * mm_per_pulse;}
    int32_t get_velocity() const {return velocity;}
    int32_t get_pulse() const {return pulse_value;}
    int32_t get_enc_pulse() const {return enc.get();}
    int32_t get_delta_pulse() {
        int32_t value{pulse_value - prev_pulse_value};
        prev_pulse_value = pulse_value;
        return value;
    }
    static float get_mm_per_pulse(POS pos) {
        switch (pos) {
        case POS::CENTER:
            return 50.0f / 1054.0f;
        case POS::LEFT:
            if (tug_encoder_controller::is_tug_connected()) {
                return 0.0033f;
            } else {
                return 50.0f / 1054.0f;
            }
        case POS::RIGHT:
            if (tug_encoder_controller::is_tug_connected()) {
                return 0.0033f;
            } else {
                return 50.0f / 1054.0f;
            }
        }

        // Not reach here
        return 0.0f;
    }

private:
    void reset_pulse() {
        pulse_value = prev_pulse_value = 0;
        enc.reset();
    }
    void wait_stabilize() {
        for (int i{0}; i < 10; ++i) {
            update_pulse();
            if (get_delta_pulse() == 0)
                break;
            k_msleep(100);
        }
    }
    int16_t update_pulse() {
        int16_t pulse{enc.get()};
        // pulse_value += pulse;
        pulse_value = pulse;
        return pulse;
    }
    encoder enc;
    float mm_per_pulse{0.0f};
    int32_t velocity{0}, pulse_value{0}, prev_pulse_value{0};
};

// This class for control 2 GPIOs as PWM
class pwm_driver {
public:
    int init(POS pos) {
        switch (pos) {
        case POS::CENTER:
            dev[0] = DEVICE_DT_GET(DT_NODELABEL(pwm5));
            dev[1] = dev[0];
            pin[0] = 1;
            pin[1] = 2;
            break;
        case POS::LEFT:
            dev[0] = DEVICE_DT_GET(DT_NODELABEL(pwm8));
            dev[1] = dev[0];
            pin[0] = 1;
            pin[1] = 2;
            break;
        case POS::RIGHT:
            dev[0] = DEVICE_DT_GET(DT_NODELABEL(pwm2));
            dev[1] = dev[0];
            pin[0] = 3;
            pin[1] = 4;
            break;
        }
        if (!device_is_ready(dev[0]) || !device_is_ready(dev[1]))
            return -1;
        set_duty(msg_control::STOP);
        return 0;
    }
    void set_duty(int8_t direction, uint8_t duty = 0) {
        uint32_t pulse_ns[2]{CONTROL_PERIOD_NS, CONTROL_PERIOD_NS};
        if (direction != msg_control::STOP && duty != 0) {
            uint32_t duty_rev{std::clamp(100U - duty, 0U, 100U)};
            uint32_t ns{duty_rev * CONTROL_PERIOD_NS / 100};
            pulse_ns[direction < msg_control::STOP ? 0 : 1] = ns;
        }
        pwm_set(dev[0], pin[0], CONTROL_PERIOD_NS, pulse_ns[0], PWM_POLARITY_NORMAL);
        pwm_set(dev[1], pin[1], CONTROL_PERIOD_NS, pulse_ns[1], PWM_POLARITY_NORMAL);
        this->direction = direction;
        this->duty = duty;
    }
    std::tuple<int8_t, uint8_t> get_duty() const {
        return {direction, duty};
    }
private:
    uint32_t pin[2]{0, 0};
    int8_t direction{msg_control::STOP};
    uint8_t duty{0};
    const device *dev[2]{nullptr, nullptr};
    static constexpr uint32_t CONTROL_HZ{10000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
};

// This calls counter
class position_control {
public:
    position_control(counter &cnt) : cnt(cnt) {}
    std::tuple<bool, int8_t, float> poll(uint32_t dt_ms) {
        if (!activated)
            return {false, msg_control::STOP, 0.0f};
        float control{0.0f};
        int32_t diff_position{target_position - cnt.get_location()};
        int8_t direction{diff_position < 0 ? msg_control::DOWN : msg_control::UP};
        if (activated) {
            float dt{dt_ms * 1e-3f};
            int32_t target_velocity{static_cast<int32_t>(diff_position * POS_P)};
            if (target_velocity < 0 && target_velocity > -vel_min)
                target_velocity = -vel_min;
            if (target_velocity > 0 && target_velocity < vel_min)
                target_velocity = vel_min;
            target_velocity = std::clamp(target_velocity, -vel_max, vel_max);
            int32_t diff_velocity{target_velocity - cnt.get_velocity()};
            float control_p{diff_velocity * VEL_P};
            control_i += diff_velocity * dt * VEL_I;
            control_p = std::clamp(control_p, -1.0f, 1.0f);
            control_i = std::clamp(control_i, -1.0f, 1.0f);
            control = control_p + control_i;
            control = std::clamp(control, -1.0f, 1.0f);
            control *= 0.3f;
            control += direction == msg_control::UP ? 0.7f : -0.7f;
        }
        return {activated, direction, control};
    }
    void on(int32_t target_position, int32_t target_power) {
        this->target_position = target_position;
        this->vel_max = 20 * target_power / 100;
        this->vel_min = 10 * target_power / 100;
        activated = true;
    }
    void off() {
        control_i = 0.0f;
        target_position = 0;
        vel_max = 20;
        vel_min = 10;
        activated = false;
    }
    bool is_near(int32_t thres_mm = 1) const {
        int32_t diff_abs{abs(target_position - cnt.get_location())};
        return diff_abs < thres_mm;
    }
private:
    counter &cnt;
    float control_i{0.0f};
    int32_t target_position{0}, vel_max{20}, vel_min{10};
    bool activated{false};
    static constexpr float POS_P{1.0f}, VEL_P{0.0f}, VEL_I{0.13f};
};

// This calls all
class actuator {
public:
    int init(POS pos) {
        if (pwm.init(pos) != 0)
            return -1;
        cnt.init(pos);

        switch (pos) {
        case POS::CENTER:
            current_adc = adc_reader::ACTUATOR_C;
            fail_checker.init(POS::CENTER);
            break;
        case POS::LEFT:
            current_adc = adc_reader::ACTUATOR_L;
            fail_checker.init(POS::LEFT);
            break;
        case POS::RIGHT:
            current_adc = adc_reader::ACTUATOR_R;
            fail_checker.init(POS::RIGHT);
            break;
        default:
            LOG_INF("invalid actuator position.");
            return -1;
        }
        if (!fail_checker.ready())
            return -1;
        return 0;
    }
    void poll() {
        uint32_t now_cycle{k_cycle_get_32()}, dt_ms{0};
        if (prev_cycle != 0)
            dt_ms = k_cyc_to_ms_near32(now_cycle - prev_cycle);
        prev_cycle = now_cycle;
        if (dt_ms > 0) {
            cnt.poll(dt_ms);
            auto [activated, direction, control]{posctl.poll(dt_ms)};
            if (activated) {
                if (float control_abs{fabsf(control)}; control_abs < 0.1f || posctl.is_near()) {
                    posctl.off();
                    pwm.set_duty(msg_control::STOP);
                } else {
                    uint8_t duty{static_cast<uint8_t>(control_abs * 100)};
                    pwm.set_duty(direction, duty);
                }
            }
        }
    }
    void to_location(int32_t location, int32_t power) {
        posctl.on(location, power);
    }
    void direct(int direction, uint8_t duty) {
        posctl.off();
        pwm.set_duty(direction, duty);
    }
    bool is_moving() {
        return cnt.get_delta_pulse() != 0;
    }
    void reset() {
        cnt.reset();
    }
    std::tuple<int32_t, int32_t, bool, int8_t, uint8_t> get_info() const {
        auto [direction, duty]{pwm.get_duty()};
        return {
            cnt.get_pulse(),
            calc_current(current_adc >= 0 ? adc_reader::get(current_adc) : 0),
            fail_checker.is_failed(),
            direction,
            duty
        };
    }
private:
    int32_t calc_current(int32_t adc_voltage_mv) const {
        static constexpr float AMP_GAIN{50.0f}, VOLTAGE_DIVIDER{1.0f}, SHUNT_REGISTER{0.01f};
        float current_a{adc_voltage_mv * 1e-3f / AMP_GAIN * VOLTAGE_DIVIDER / SHUNT_REGISTER};
        return static_cast<int32_t>(current_a * 1e+3f);
    }
    counter cnt;
    pwm_driver pwm;
    position_control posctl{cnt};
    uint32_t prev_cycle{0};
    int32_t current_adc{-1};
    class {
    public:
        void init(POS pos) {
            switch (pos) {
            case POS::CENTER:
                dev = GPIO_DT_SPEC_GET(DT_NODELABEL(act_c_fail), gpios);
                break;
            case POS::LEFT:
                dev = GPIO_DT_SPEC_GET(DT_NODELABEL(act_l_fail), gpios);
                break;
            case POS::RIGHT:
                dev = GPIO_DT_SPEC_GET(DT_NODELABEL(act_r_fail), gpios);
                break;
            default:
                LOG_INF("invalid actuator position.");
                return;
            }
            
            if (gpio_is_ready_dt(&dev))
                gpio_pin_configure_dt(&dev, GPIO_INPUT | GPIO_ACTIVE_HIGH);
            this->pin = pin;
        }
        bool ready() const {return gpio_is_ready_dt(&dev);}
        
        bool is_failed() const {
            return ready() ? gpio_pin_get_dt(&dev) == 0 : false;
        }
    private:
        uint32_t pin{0};
        // const device *dev{nullptr};
        gpio_dt_spec dev;
    } fail_checker;
};

class actuator_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        k_msgq_init(&msgq_control, msgq_control_buffer, sizeof (msg_control), 8);
        return 0;
    }

    void run() {
        if (act[0].init(POS::CENTER) != 0 ||
            act[1].init(POS::LEFT) != 0 ||
            act[2].init(POS::RIGHT) != 0)
        {
            LOG_ERR("actuator init failed.");
            return;
        }

        // Enable Pin, Reset Functions
        gpio_dt_spec dev_enable = GET_GPIO(ps_lift_actuator);
        if (gpio_is_ready_dt(&dev_enable))
            gpio_pin_configure_dt(&dev_enable, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        auto reset_actuator = [&]() {
            if (gpio_is_ready_dt(&dev_enable)) {
                gpio_pin_set_dt(&dev_enable, 0);
                k_msleep(100);
                gpio_pin_set_dt(&dev_enable, 1);
            }
        };
        int fail_count{0};
        auto fail_check = [&](bool failed) {
            if (failed) {
                static constexpr int fail_max{10};
                if (fail_count < fail_max) {
                    LOG_WRN("fail of actuator detected, reset.");
                    reset_actuator();
                    ++fail_count;
                } else if (fail_count == fail_max) {
                    LOG_WRN("continued fail of actuator detected.");
                    fail_count = fail_max + 1;
                }
            } else {
                if (fail_count > 0)
                    LOG_WRN("recovered from actuator fail.");
                fail_count = 0;
            }
        };
        reset_actuator();

        uint32_t prev_cycle{k_cycle_get_32()};

        while (true) {
            for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
                act[i].poll();
            bool is_emergency{board_controller::is_emergency()};  // -> board controller
            msg_control can2actuator;
            if (k_msgq_get(&msgq_control, &can2actuator, K_NO_WAIT) == 0 && !is_emergency)
                handle_control(can2actuator);
            msg_pwmtrampoline pwmtrampoline;
            if (k_msgq_get(&msgq_pwmtrampoline, &pwmtrampoline, K_NO_WAIT) == 0 && !is_emergency)
                handle_pwmtrampoline(pwmtrampoline);
            if (is_emergency)
                pwm_direct_all(msg_control::STOP);
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 20) {
                prev_cycle = now_cycle;
                bool failed{false};
                for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
                    int8_t direction;
                    uint8_t duty;
                    std::tie(actuator2can.encoder_count[i],
                             actuator2can.current[i],
                             actuator2can.fail[i],
                             direction,
                             duty) = act[i].get_info();
                    if (actuator2can.fail[i])
                        failed = true;
                }
                fail_check(failed);
                actuator2can.connect = 0;
                while (k_msgq_put(&msgq, &actuator2can, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq);
            }
            k_msleep(10);
        }
    }
    int init_location(const int8_t (&directions)[ACTUATOR_NUM]) {
        LOG_INF("initialize location.");

        if (board_controller::is_emergency()) {
            LOG_WRN("can not initialize location in emergency.");
            return -1;
        }

        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            if (directions[i] != msg_control::UP && directions[i] != msg_control::STOP && directions[i] != msg_control::DOWN) {
                LOG_WRN("invalid direction.");
                return -1;
            }
        }

        location_initialized = false;
        constexpr uint8_t powers[ACTUATOR_NUM]{100, 100, 100};
        control_trampoline(directions, powers);
        bool stopped{wait_actuator_stop(30000, 100)};
        pwm_trampoline_all(msg_control::STOP);
        if (!stopped || board_controller::is_emergency()) {
            LOG_WRN("can not initialize location.");
            return -1;
        }
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            if (directions[i] == msg_control::UP || directions[i] == msg_control::DOWN) {
                act[i].reset();
            }
        }
        location_initialized = true;
        return 0;
    }
    int to_location(const int8_t (&location)[ACTUATOR_NUM], const uint8_t (&power)[ACTUATOR_NUM], uint8_t (&detail)[ACTUATOR_NUM]) {
        LOG_INF("move location.");
        if (!location_initialized) {
            for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
                detail[i] = 3;
            LOG_WRN("location not initialized.");
            return -1;
        }
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            act[i].to_location(location[i], power[i]);
        bool stopped{wait_actuator_stop(30000, 100)};
        pwm_trampoline_all(msg_control::STOP);
        if (!stopped || board_controller::is_emergency()) {
            LOG_WRN("unable to move location.");
            return -1;
        }
        return 0;
    }
    void set_current_monitor() const {
    }
    void info(const shell *shell) const {
        shell_print(shell, "[notice] parameter order [Center] [Left] [Right]");
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            auto [pulse, current, fail, direction, duty]{act[i].get_info()};
            const double mm_per_pulse{counter::get_mm_per_pulse(static_cast<POS>(i))};
            shell_print(shell,
                        "actuator: %d encoder: %d pulse current: %d mV fail: %d dir: %d duty: %u mm/pulse: %f",
                        i, pulse, current, fail, direction, duty, mm_per_pulse);
        }
    }
    void pwm_trampoline(int index, int direction, uint8_t pwm_duty = 0) const {
        msg_pwmtrampoline message;
        message.all = false;
        message.index = index;
        message.direction = direction;
        message.duty = pwm_duty;
        while (k_msgq_put(&msgq_pwmtrampoline, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_pwmtrampoline);
    }
    void control_trampoline(const int8_t (&directions)[ACTUATOR_NUM], const uint8_t (&powers)[ACTUATOR_NUM]) {
        msg_control message;
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            message.actuators[i].direction = directions[i];
            message.actuators[i].power = powers[i];
        }
        while (k_msgq_put(&msgq_control, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_control);
    }
private:
    void handle_control(const msg_control &msg) {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            act[i].direct(msg.actuators[i].direction, msg.actuators[i].power);
    }
    void handle_pwmtrampoline(const msg_pwmtrampoline &msg) {
        if (msg.all)
            pwm_direct_all(msg.direction, msg.duty);
        else
            act[msg.index].direct(msg.direction, msg.duty);
    }
    void pwm_direct_all(int direction, uint8_t pwm_duty = 0) {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            act[i].direct(direction, pwm_duty);
    }
    void pwm_trampoline_all(int direction, uint8_t pwm_duty = 0) const {
        msg_pwmtrampoline message;
        message.all = true;
        message.direction = direction;
        message.duty = pwm_duty;
        while (k_msgq_put(&msgq_pwmtrampoline, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_pwmtrampoline);
    }
    bool wait_actuator_stop(uint32_t timeout_ms, uint32_t sleep_ms) {
        int remaining{3};
        for (uint32_t i{0}, end{timeout_ms / sleep_ms}; i < end; ++i) {
            remaining = 3;
            for (uint32_t j{0}; j < ACTUATOR_NUM; ++j) {
                if (i >= 4 && !act[j].is_moving()) {
                    pwm_trampoline(j, msg_control::STOP);
                    --remaining;
                }
            }
            if (remaining <= 0)
                break;
            k_msleep(sleep_ms);
        }
        return remaining <= 0;
    }
    msg actuator2can;
    actuator act[3];
    bool location_initialized{false};
} impl;

int cmd_duty(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "[notice] parameter order [Center] [Left] [Right]");
    if (argc != 3 && argc != 5 && argc != 7) {
        shell_error(shell, "Usage: %s %s <direction> <power> ...\n", argv[-1], argv[0]);
        return 1;
    }
    for (size_t i{0}, end{argc / 2}; i < end; ++i) {
        uint8_t direction, duty;
        direction = atoi(argv[i * 2 + 1]);
        duty      = atoi(argv[i * 2 + 2]);
        impl.pwm_trampoline(i, direction, duty);
    }
    return 0;
}

int cmd_duty_rep_all(const shell *shell, size_t argc, char **argv)
{
    int rep_num = 10000;

    shell_print(shell, "Repeat Up-Down 10,000 times with 5sec interval.");
    for (int ii{0}; ii < rep_num; ++ii) {
        for (size_t i{0}; i < 3; ++i) {
            uint8_t direction, duty;
            direction = 1;
            duty      = 100;
            impl.pwm_trampoline(i, direction, duty);
        }

        k_msleep(5000);

        for (size_t i{0}; i < 3; ++i) {
            uint8_t direction, duty;
            direction = -1;
            duty      = 100;
            impl.pwm_trampoline(i, direction, duty);
        }

        k_msleep(5000);
    }
    
    return 0;
}

int cmd_init(const shell *shell, size_t argc, char **argv)
{
    if (argc != 1 && argc != (ACTUATOR_NUM+1)) {
        shell_error(shell, "Usage: %s %s <direction> ...\n", argv[-1], argv[0]);
        return 1;
    }

    int8_t directions[ACTUATOR_NUM]{msg_control::DOWN, msg_control::DOWN, msg_control::DOWN};
    for (size_t i{1}; i < argc; ++i) {
        int8_t cur_dir = atoi(argv[i]);
        if (cur_dir != msg_control::UP && cur_dir != msg_control::STOP && cur_dir != msg_control::DOWN) {
            shell_print(shell, "Invalid direction.");
            return 1;
        }

        directions[i-1] = cur_dir;
    }

    shell_print(shell, "[notice] parameter order [Center] [Left] [Right]");
    if (impl.init_location(directions) != 0)
        shell_print(shell, "init error.");
    return 0;
}

int locate(const shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "[notice] parameter order [Center] [Left] [Right]");
    int8_t location[ACTUATOR_NUM]{0, 0, 0};
    uint8_t power[ACTUATOR_NUM]{0, 0, 0};
    uint8_t detail[ACTUATOR_NUM]{0, 0, 0};
    if (argc != 3 && argc != 5 && argc != 7) {
        shell_error(shell, "Usage: %s %s <location> <power> ...\n", argv[-1], argv[0]);
        return 1;
    }
    for (size_t i{0}, end{argc / 2}; i < end; ++i) {
        location[i] = atoi(argv[i * 2 + 1]);
        power[i]    = atoi(argv[i * 2 + 2]);
    }
    if (impl.to_location(location, power, detail) != 0)
        shell_print(shell, "location error.");
    return 0;
}

int current(const shell *shell, size_t argc, char **argv)
{
    impl.set_current_monitor();
    return 0;
}

int info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(duty, NULL, "Actuator duty command", cmd_duty),
    SHELL_CMD(duty_rep, NULL, "Actuator duty command", cmd_duty_rep_all),
    SHELL_CMD(init, NULL, "Actuator initialize command", cmd_init),
    SHELL_CMD(loc, NULL, "Actuator locate command", locate),
    SHELL_CMD(current, NULL, "Actuator current monitor", current),
    SHELL_CMD(info, NULL, "Actuator information", info),
    // SHELL_CMD(param, NULL, "Actuator param", set_param),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(act, &sub, "Actuator commands", NULL);

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

int init_location(const int8_t (&directions)[ACTUATOR_NUM])
{
    return impl.init_location(directions);
}

int to_location(const int8_t (&location)[ACTUATOR_NUM], const uint8_t (&power)[ACTUATOR_NUM], uint8_t (&detail)[ACTUATOR_NUM])
{
    return impl.to_location(location, power, detail);
}

k_thread thread;
k_msgq msgq, msgq_control;

}

// vim: set expandtab shiftwidth=4:
