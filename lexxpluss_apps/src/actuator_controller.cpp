#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <cmath>
#include <cstdlib>
#include <tuple>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
#include "can_controller.hpp"

extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim_encoder)
{
    GPIO_InitTypeDef GPIO_InitStruct{0};
    if(htim_encoder->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    } else if(htim_encoder->Instance == TIM5) {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    } else if(htim_encoder->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

namespace lexxfirm::actuator_controller {

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

template<typename T>
inline const T &constrain(const T &val, const T &min, const T &max)
{
    return val < min ? min : (val > max ? max : val);
}

enum class POS {
    LEFT, CENTER, RIGHT
};

class encoder {
public:
    int init(TIM_TypeDef *tim) {
        this->tim = tim;
        TIM_Encoder_InitTypeDef sConfig{0};
        TIM_MasterConfigTypeDef sMasterConfig{0};
        if (tim == TIM1 || tim == TIM8) {
            timh.Init.Period = 65535;
            timh.Init.RepetitionCounter = 0;
            sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        } else if (tim == TIM5) {
            timh.Init.Period = 4294967295;
        }
        timh.Instance = tim;
        timh.Init.Prescaler = 0;
        timh.Init.CounterMode = TIM_COUNTERMODE_UP;
        timh.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0b1111;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0b1111;
        if (HAL_TIM_Encoder_Init(&timh, &sConfig) != HAL_OK)
            return -1;
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh, &sMasterConfig) != HAL_OK)
            return -1;
        HAL_TIM_Encoder_Start(&timh, TIM_CHANNEL_ALL);
        return 0;
    }
    int16_t get() const {
        int16_t count{static_cast<int16_t>(tim->CNT)};
        tim->CNT = 0;
        return count;
    }
private:
    TIM_TypeDef *tim{nullptr};
    TIM_HandleTypeDef timh;
};

class counter {
public:
    int init(POS pos) {
        int result{0};
        switch (pos) {
        case POS::LEFT:
            result = enc.init(TIM1);
            mm_per_pulse = 50.0f / 538.0f;
            break;
        case POS::CENTER:
            result = enc.init(TIM8);
            mm_per_pulse = 50.0f / 1054.0f;
            break;
        case POS::RIGHT:
            result = enc.init(TIM5);
            mm_per_pulse = 50.0f / 538.0f;
            break;
        }
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
    int32_t get_delta_pulse() {
        int32_t value{pulse_value - prev_pulse_value};
        prev_pulse_value = pulse_value;
        return value;
    }
private:
    void reset_pulse() {
        pulse_value = prev_pulse_value = 0;
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
        pulse_value += pulse;
        return pulse;
    }
    encoder enc;
    float mm_per_pulse{0.0f};
    int32_t velocity{0}, pulse_value{0}, prev_pulse_value{0};
};

class pwm_driver {
public:
    int init(POS pos) {
        switch (pos) {
        case POS::LEFT:
            dev[0] = device_get_binding("PWM_12");
            dev[1] = device_get_binding("PWM_2");
            pin[0] = 1;
            pin[1] = 3;
            break;
        case POS::CENTER:
            dev[0] = device_get_binding("PWM_14");
            dev[1] = device_get_binding("PWM_4");
            pin[0] = 1;
            pin[1] = 1;
            break;
        case POS::RIGHT:
            dev[0] = device_get_binding("PWM_3");
            dev[1] = device_get_binding("PWM_9");
            pin[0] = 3;
            pin[1] = 1;
            break;
        }
        if (!device_is_ready(dev[0]) || !device_is_ready(dev[1]))
            return -1;
        set_duty(msg_control::STOP);
        return 0;
    }
    void set_duty(int8_t direction, uint8_t duty = 0) const {
        uint32_t pulse_ns[2]{0, 0};
        if (direction != msg_control::STOP && duty != 0) {
            uint32_t ns{duty * CONTROL_PERIOD_NS / 100};
            pulse_ns[direction > msg_control::STOP ? 0 : 1] = ns;
        }
        pwm_pin_set_nsec(dev[0], pin[0], CONTROL_PERIOD_NS, pulse_ns[0], PWM_POLARITY_NORMAL);
        pwm_pin_set_nsec(dev[1], pin[1], CONTROL_PERIOD_NS, pulse_ns[1], PWM_POLARITY_NORMAL);
    }
private:
    uint32_t pin[2]{0, 0};
    const device *dev[2]{nullptr, nullptr};
    static constexpr uint32_t CONTROL_HZ{10000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
};

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
            target_velocity = constrain(target_velocity, -vel_max, vel_max);
            int32_t diff_velocity{target_velocity - cnt.get_velocity()};
            float control_p{diff_velocity * VEL_P};
            control_i += diff_velocity * dt * VEL_I;
            control_p = constrain(control_p, -1.0f, 1.0f);
            control_i = constrain(control_i, -1.0f, 1.0f);
            control = control_p + control_i;
            control = constrain(control, -1.0f, 1.0f);
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
    // void set_param(float pp, float vp, float vi) {
    //     POS_P = pp;
    //     VEL_P = vp;
    //     VEL_I = vi;
    // }
private:
    counter &cnt;
    float control_i{0.0f};
    int32_t target_position{0}, vel_max{20}, vel_min{10};
    bool activated{false};
    static constexpr float POS_P{1.0f}, VEL_P{0.0f}, VEL_I{0.13f};
};

class actuator {
public:
    int init(POS pos) {
        if (pwm.init(pos) != 0)
            return -1;
        cnt.init(pos);
        switch (pos) {
        case POS::LEFT:
            current_adc = adc_reader::ACTUATOR_0;
            fail_checker.init("GPIOF", 12);
            break;
        case POS::CENTER:
            current_adc = adc_reader::ACTUATOR_1;
            fail_checker.init("GPIOF", 11);
            break;
        case POS::RIGHT:
            current_adc = adc_reader::ACTUATOR_2;
            fail_checker.init("GPIOJ", 4);
            break;
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
    std::tuple<int32_t, int32_t, bool> get_info() const {
        return {
            cnt.get_pulse(),
            current_adc >= 0 ? adc_reader::get(current_adc) : 0,
            fail_checker.is_failed()
        };
    }
    // void set_param(float pp, float vp, float vi) {
    //     posctl.set_param(pp, vp, vi);
    // }
private:
    counter cnt;
    pwm_driver pwm;
    position_control posctl{cnt};
    uint32_t prev_cycle{0};
    int32_t current_adc{-1};
    class {
    public:
        void init(const char *devname, uint32_t pin) {
            dev = device_get_binding(devname);
            if (device_is_ready(dev))
                gpio_pin_configure(dev, pin, GPIO_INPUT | GPIO_ACTIVE_HIGH);
            this->pin = pin;
        }
        bool ready() const {return device_is_ready(dev);}
        bool is_failed() const {
            return ready() ? gpio_pin_get(dev, pin) == 0 : false;
        }
    private:
        uint32_t pin{0};
        const device *dev{nullptr};
    } fail_checker;
};

class actuator_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        k_msgq_init(&msgq_control, msgq_control_buffer, sizeof (msg_control), 8);
        if (act[0].init(POS::LEFT) != 0 ||
            act[1].init(POS::CENTER) != 0 ||
            act[2].init(POS::RIGHT) != 0)
            return -1;
        return 0;
    }
    void run() {

        if (const device *dev_enable = device_get_binding("GPIOB"); device_is_ready(dev_enable))
            gpio_pin_configure(dev_enable, 1, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        const device *gpiok = device_get_binding("GPIOK");
        if (device_is_ready(gpiok))
            gpio_pin_configure(gpiok, 4, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        int heartbeat_led{1};
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            act[i].reset();
        uint32_t prev_cycle{k_cycle_get_32()};
        while (true) {
            for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
                act[i].poll();
            bool is_emergency{can_controller::is_emergency()};
            msg_control ros2actuator;
            if (k_msgq_get(&msgq_control, &ros2actuator, K_NO_WAIT) == 0 && !is_emergency)
                handle_control(ros2actuator);
            msg_pwmtrampoline pwmtrampoline;
            if (k_msgq_get(&msgq_pwmtrampoline, &pwmtrampoline, K_NO_WAIT) == 0 && !is_emergency)
                handle_pwmtrampoline(pwmtrampoline);
            if (is_emergency)
                pwm_direct_all(msg_control::STOP);
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 100) {
                prev_cycle = now_cycle;
                for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
                    std::tie(actuator2ros.encoder_count[i],
                             actuator2ros.current[i],
                             actuator2ros.fail[i]) = act[i].get_info();
                }
                actuator2ros.connect = adc_reader::get(adc_reader::TROLLEY);
                while (k_msgq_put(&msgq, &actuator2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq);
                if (device_is_ready(gpiok)) {
                    gpio_pin_set(gpiok, 4, heartbeat_led);
                    heartbeat_led = !heartbeat_led;
                }
            }
            k_msleep(10);
        }
    }
    int init_location() {
        LOG_INF("initialize location.");
        location_initialized = false;
        pwm_trampoline_all(msg_control::DOWN, 100);
        bool stopped{wait_actuator_stop(30000, 500)};
        pwm_trampoline_all(msg_control::STOP);
        if (!stopped || can_controller::is_emergency()) {
            LOG_WRN("can not initialize location.");
            return -1;
        }
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            act[i].reset();
        location_initialized = true;
        return 0;
    }
    int to_location(const uint8_t (&location)[ACTUATOR_NUM], const uint8_t (&power)[ACTUATOR_NUM], uint8_t (&detail)[ACTUATOR_NUM]) {
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
        if (!stopped || can_controller::is_emergency()) {
            LOG_WRN("unable to move location.");
            return -1;
        }
        return 0;
    }
    void set_current_monitor() {
    }
    void info(const shell *shell) const {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            auto [pulse, current, fail] = act[i].get_info();
            shell_print(shell, "actuator: %d encoder: %d pulse current: %d mV", i, pulse, current);
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
    // void set_param(float pp, float vp, float vi) {
    //     for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
    //         act[i].set_param(pp, vp, vi);
    // }
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
    msg actuator2ros;
    actuator act[3];
    bool location_initialized{false};
} impl;

int cmd_duty(const shell *shell, size_t argc, char **argv)
{
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

int cmd_init(const shell *shell, size_t argc, char **argv)
{
    if (impl.init_location() != 0)
        shell_print(shell, "init error.");
    return 0;
}

int locate(const shell *shell, size_t argc, char **argv)
{
    uint8_t location[ACTUATOR_NUM]{0, 0, 0}, power[ACTUATOR_NUM]{0, 0, 0}, detail[ACTUATOR_NUM]{0, 0, 0};
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

// int set_param(const shell *shell, size_t argc, char **argv)
// {
//     float pp{0.0f}, vp{0.0f}, vi{0.0f};
//     if (argc > 1)
//         pp = atof(argv[1]);
//     if (argc > 2)
//         vp = atof(argv[2]);
//     if (argc > 3)
//         vi = atof(argv[3]);
//     impl.set_param(pp, vp, vi);
//     return 0;
// }

SHELL_STATIC_SUBCMD_SET_CREATE(sub,
    SHELL_CMD(duty, NULL, "Actuator duty command", cmd_duty),
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

int init_location()
{
    return impl.init_location();
}

int to_location(const uint8_t (&location)[ACTUATOR_NUM], const uint8_t (&power)[ACTUATOR_NUM], uint8_t (&detail)[ACTUATOR_NUM])
{
    return impl.to_location(location, power, detail);
}

k_thread thread;
k_msgq msgq, msgq_control;

}

// vim: set expandtab shiftwidth=4:
