#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <cstdlib>
#include "adc_reader.hpp"
#include "actuator_controller.hpp"
#include "rosdiagnostic.hpp"

k_msgq msgq_actuator2ros;
k_msgq msgq_ros2actuator;

extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim_encoder)
{
    GPIO_InitTypeDef GPIO_InitStruct{0};
    if(htim_encoder->Instance==TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    } else if(htim_encoder->Instance==TIM5) {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    } else if(htim_encoder->Instance==TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

namespace {

LOG_MODULE_REGISTER(actuator);

char __aligned(4) msgq_actuator2ros_buffer[8 * sizeof (msg_actuator2ros)];
char __aligned(4) msgq_ros2actuator_buffer[8 * sizeof (msg_ros2actuator)];

static constexpr uint32_t ACTUATOR_NUM{3};

class timer_hal_helper {
public:
    int init() {
        if (init_tim1() != 0 || init_tim5() != 0 || init_tim8() != 0)
            return -1;
        return 0;
    }
    void get_count(int16_t (&data)[ACTUATOR_NUM]) const {
        data[0] = TIM1->CNT;
        TIM1->CNT = 0;
        data[1] = TIM8->CNT;
        TIM8->CNT = 0;
        data[2] = TIM5->CNT;
        TIM5->CNT = 0;
    }
private:
    int init_tim1() {
        TIM_Encoder_InitTypeDef sConfig{0};
        timh[0].Instance = TIM1;
        timh[0].Init.Prescaler = 0;
        timh[0].Init.CounterMode = TIM_COUNTERMODE_UP;
        timh[0].Init.Period = 65535;
        timh[0].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh[0].Init.RepetitionCounter = 0;
        timh[0].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0b1111;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0b1111;
        if (HAL_TIM_Encoder_Init(&timh[0], &sConfig) != HAL_OK)
            return -1;
        TIM_MasterConfigTypeDef sMasterConfig{0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh[0], &sMasterConfig) != HAL_OK)
            return -1;
        HAL_TIM_Encoder_Start(&timh[0], TIM_CHANNEL_ALL);
        return 0;
    }
    int init_tim5() {
        TIM_Encoder_InitTypeDef sConfig{0};
        timh[1].Instance = TIM5;
        timh[1].Init.Prescaler = 0;
        timh[1].Init.CounterMode = TIM_COUNTERMODE_UP;
        timh[1].Init.Period = 4294967295;
        timh[1].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh[1].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0b1111;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0b1111;
        if (HAL_TIM_Encoder_Init(&timh[1], &sConfig) != HAL_OK)
            return -1;
        TIM_MasterConfigTypeDef sMasterConfig{0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh[1], &sMasterConfig) != HAL_OK)
            return -1;
        return 0;
    }
    int init_tim8() {
        TIM_Encoder_InitTypeDef sConfig{0};
        timh[2].Instance = TIM8;
        timh[2].Init.Prescaler = 0;
        timh[2].Init.CounterMode = TIM_COUNTERMODE_UP;
        timh[2].Init.Period = 65535;
        timh[2].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh[2].Init.RepetitionCounter = 0;
        timh[2].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0b1111;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0b1111;
        if (HAL_TIM_Encoder_Init(&timh[2], &sConfig) != HAL_OK)
            return -1;
        TIM_MasterConfigTypeDef sMasterConfig{0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh[2], &sMasterConfig) != HAL_OK)
            return -1;
        return 0;
    }
    TIM_HandleTypeDef timh[ACTUATOR_NUM];
};

class location_calculator {
public:
    location_calculator() {
        mm_per_pulse[0] = 50.0f / 1054.0f;
        mm_per_pulse[1] = 50.0f / 538.0f;
        mm_per_pulse[2] = 50.0f / 538.0f;
    }
    int init() {
        reset();
        return helper.init();
    }
    void reset() {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            pulse_value[i] = prev_pulse_value[i] = 0;
    }
    void poll() {
        int16_t d[ACTUATOR_NUM];
        helper.get_count(d);
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            update_pulse(i, d[i]);
    }
    int32_t get_location(int index) const {
        return static_cast<float>(pulse_value[index]) * mm_per_pulse[index];
    }
    void get_pulse(int32_t (&value)[ACTUATOR_NUM]) const {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            value[i] = pulse_value[i];
    }
    void get_delta_pulse(int32_t (&value)[ACTUATOR_NUM]) {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            value[i] = pulse_value[i] - prev_pulse_value[i];
            prev_pulse_value[i] = pulse_value[i];
        }
    }
private:
    void update_pulse(int index, int pulse) {
        pulse_value[index] += -pulse;
    }
    timer_hal_helper helper;
    int32_t pulse_value[ACTUATOR_NUM]{0, 0, 0}, prev_pulse_value[ACTUATOR_NUM]{0, 0, 0};
    float mm_per_pulse[ACTUATOR_NUM];
};

static const struct {
    const char *name;
    uint32_t pin;
} config[ACTUATOR_NUM][2]{
    {{"PWM_12", 1}, {"PWM_2", 3}},
    {{"PWM_14", 1}, {"PWM_4", 1}},
    {{"PWM_3",  3}, {"PWM_9", 1}}
};

class actuator_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_actuator2ros, msgq_actuator2ros_buffer, sizeof (msg_actuator2ros), 8);
        k_msgq_init(&msgq_ros2actuator, msgq_ros2actuator_buffer, sizeof (msg_ros2actuator), 8);
        prev_cycle = k_cycle_get_32();
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            for (uint32_t j{0}; j < 2; ++j) {
                dev_pwm[i][j] = device_get_binding(config[i][j].name);
                if (dev_pwm[i][j] == nullptr)
                    return -1;
            }
        }
        dev_power = device_get_binding("GPIOB");
        dev_fail_01 = device_get_binding("GPIOF");
        dev_fail_2 = device_get_binding("GPIOJ");
        if (dev_power == nullptr || dev_fail_01 == nullptr || dev_fail_2 == nullptr)
            return -1;
        gpio_pin_configure(dev_power, 1, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_fail_01, 11, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_fail_01, 12, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_fail_2, 4, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        k_mutex_init(&service_mutex);
        return calculator.init();
    }
    void run() {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            for (uint32_t j{0}; j < 2; ++j) {
                if (!device_is_ready(dev_pwm[i][j]))
                    return;
                pwm_pin_set_nsec(dev_pwm[i][j], config[i][j].pin, CONTROL_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
            }
        }
        wait_encoder_stabilize();
        const device *gpiok = device_get_binding("GPIOK");
        if (gpiok != nullptr)
            gpio_pin_configure(gpiok, 4, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        int heartbeat_led{1};
        while (true) {
            msg_ros2actuator message;
            if (k_msgq_get(&msgq_ros2actuator, &message, K_NO_WAIT) == 0)
                handle_control(message);
            calculator.poll();
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 100) {
                prev_cycle = now_cycle;
                calculator.get_pulse(actuator2ros.encoder_count);
                get_current(actuator2ros.current);
                actuator2ros.connect = get_trolley();
                get_fail(actuator2ros.fail);
                while (k_msgq_put(&msgq_actuator2ros, &actuator2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_actuator2ros);
                if (gpiok != nullptr) {
                    gpio_pin_set(gpiok, 4, heartbeat_led);
                    heartbeat_led = !heartbeat_led;
                }
            }
            k_msleep(10);
        }
    }
    void run_error() const {
        msg_rosdiag message{msg_rosdiag::ERROR, "actuator", "no device"};
        while (true) {
            while (k_msgq_put(&msgq_rosdiag, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_rosdiag);
            k_msleep(5000);
        }
    }
    void wait_encoder_stabilize() {
        for (int i{0}; i < 10; ++i) {
            int32_t value[ACTUATOR_NUM];
            calculator.poll();
            calculator.get_delta_pulse(value);
            if (value[0] == 0 && value[1] == 0 && value[2] == 0)
                break;
            k_msleep(100);
        }
        calculator.reset();
    }
    int init_location() {
        LOG_INF("initialize location.");
        location_initialized = false;
        if (k_mutex_lock(&service_mutex, K_MSEC(30000)) != 0) {
            LOG_WRN("can not lock mutex.");
            return -1;
        }
        pwm_control_all(msg_ros2actuator::DOWN, 100);
        static constexpr uint32_t timeout_ms{30000}, sleep_ms{500};
        int remaining{3};
        for (uint32_t i{0}, end{timeout_ms / sleep_ms}; i < end; ++i) {
            int32_t value[ACTUATOR_NUM];
            calculator.get_delta_pulse(value);
            remaining = 3;
            for (uint32_t j{0}; j < ACTUATOR_NUM; ++j) {
                if (i >= 2 && value[j] == 0) {
                    pwm_control(j, msg_ros2actuator::STOP, 0);
                    --remaining;
                }
            }
            if (remaining <= 0)
                break;
            k_msleep(sleep_ms);
        }
        pwm_control_all(msg_ros2actuator::STOP);
        k_mutex_unlock(&service_mutex);
        calculator.reset();
        if (remaining <= 0) {
            location_initialized = true;
            return 0;
        } else {
            LOG_WRN("can not initialize location.");
            return -1;
        }
    }
    int to_location(const uint8_t location[ACTUATOR_NUM], const uint8_t power[ACTUATOR_NUM], uint8_t detail[ACTUATOR_NUM]) {
        LOG_INF("move location.");
        if (!location_initialized) {
            for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
                detail[i] = 3;
            LOG_WRN("location not initialized.");
            return -1;
        }
        if (k_mutex_lock(&service_mutex, K_MSEC(30000)) != 0) {
            LOG_WRN("can not lock mutex.");
            return -1;
        }
        int direction[ACTUATOR_NUM];
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            if (power[i] != 0) {
                int32_t diff{static_cast<int32_t>(location[i]) - calculator.get_location(i)};
                direction[i] = diff < 0 ? msg_ros2actuator::DOWN : msg_ros2actuator::UP;
                pwm_control(i, direction[i], power[i]);
                detail[i] = 1;
            } else {
                direction[i] = msg_ros2actuator::STOP;
                detail[i] = 0;
            }
        }
        static constexpr uint32_t timeout_ms{30000}, sleep_ms{20};
        int remaining{3};
        for (uint32_t i{0}, end{timeout_ms / sleep_ms}; i < end; ++i) {
            int32_t value[ACTUATOR_NUM];
            calculator.get_delta_pulse(value);
            remaining = 3;
            for (uint32_t j{0}; j < ACTUATOR_NUM; ++j) {
                int32_t diff{static_cast<int32_t>(location[j]) - calculator.get_location(j)};
                if ((direction[j] == msg_ros2actuator::DOWN && diff >= 0) ||
                    (direction[j] == msg_ros2actuator::UP   && diff <= 0) ||
                    (direction[j] != msg_ros2actuator::STOP && i >= 5 && value[j] == 0)) {
                    direction[j] = msg_ros2actuator::STOP;
                    pwm_control(j, msg_ros2actuator::STOP, 0);
                    detail[j] = 0;
                    --remaining;
                } else if (direction[j] == msg_ros2actuator::STOP) {
                    --remaining;
                }
            }
            if (remaining <= 0)
                break;
            k_msleep(sleep_ms);
        }
        pwm_control_all(msg_ros2actuator::STOP);
        k_mutex_unlock(&service_mutex);
        if (remaining <= 0) {
            return 0;
        } else {
            LOG_WRN("unable to move location.");
            return -1;
        }
    }
    void set_current_monitor() {
        current_monitor = !current_monitor;
    }
    void info(const shell *shell) const {
        int32_t encoder[ACTUATOR_NUM], current[ACTUATOR_NUM];
        calculator.get_pulse(encoder);
        get_current(current);
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            shell_print(shell, "actuator: %d encoder: %dpulse current: %dmV", i, encoder[i], current[i]);
    }
private:
    void handle_control(const msg_ros2actuator &msg) {
        if (k_mutex_lock(&service_mutex, K_MSEC(10)) != 0)
            return;
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            pwm_control(i, msg.actuators[i].direction, msg.actuators[i].power);
        k_mutex_unlock(&service_mutex);
    }
    void pwm_control_all(int direction, uint8_t pwm_duty = 0) const {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            pwm_control(i, direction, pwm_duty);
    }
    void pwm_control(int index, int direction, uint8_t pwm_duty) const {
        if (direction == msg_ros2actuator::STOP || pwm_duty == 0) {
            for (uint32_t i{0}; i < 2; ++i)
                pwm_pin_set_nsec(dev_pwm[index][i], config[index][i].pin, CONTROL_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
        } else {
            uint32_t pulse_ns{pwm_duty * CONTROL_PERIOD_NS / 100};
            if (direction < msg_ros2actuator::STOP) {
                pwm_pin_set_nsec(dev_pwm[index][0], config[index][0].pin, CONTROL_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
                pwm_pin_set_nsec(dev_pwm[index][1], config[index][1].pin, CONTROL_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
            } else {
                pwm_pin_set_nsec(dev_pwm[index][0], config[index][0].pin, CONTROL_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
                pwm_pin_set_nsec(dev_pwm[index][1], config[index][1].pin, CONTROL_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
            }
        }
    }
    void get_current(int32_t (&data)[ACTUATOR_NUM]) const {
        data[0] = adc_reader::get(adc_reader::INDEX_ACTUATOR_0);
        data[1] = adc_reader::get(adc_reader::INDEX_ACTUATOR_1);
        data[2] = adc_reader::get(adc_reader::INDEX_ACTUATOR_2);
        if (current_monitor)
            LOG_INF("current: %8d %8d %8d", data[0], data[1], data[2]);
    }
    int32_t get_trolley() const {
        return adc_reader::get(adc_reader::INDEX_TROLLEY);
    }
    void get_fail(bool (&fail)[ACTUATOR_NUM]) const {
        fail[0] = gpio_pin_get(dev_fail_01, 11) == 0;
        fail[1] = gpio_pin_get(dev_fail_01, 12) == 0;
        fail[2] = gpio_pin_get(dev_fail_2, 4) == 0;
    }
    location_calculator calculator;
    msg_actuator2ros actuator2ros;
    k_mutex service_mutex;
    uint32_t prev_cycle{0};
    bool location_initialized{false}, current_monitor{false};
    const device *dev_pwm[ACTUATOR_NUM][2]{{nullptr, nullptr}, {nullptr, nullptr}, {nullptr, nullptr}};
    const device *dev_power{nullptr}, *dev_fail_01{nullptr}, *dev_fail_2{nullptr};
    static constexpr uint32_t CONTROL_HZ{5000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
} impl;

static int cmd_init(const shell *shell, size_t argc, char **argv)
{
    impl.init_location();
    return 0;
}

static int cmd_locate(const shell *shell, size_t argc, char **argv)
{
    uint8_t location[ACTUATOR_NUM]{0, 0, 0}, power[ACTUATOR_NUM]{0, 0, 0}, detail[ACTUATOR_NUM]{0, 0, 0};
    if (argc < 3) {
        shell_error(shell, "Usage: %s %s <location> <power> ...\n", argv[-1], argv[0]);
        return 1;
    } else if (argc == 3) {
        location[0] = atoi(argv[1]);
        power[0]    = atoi(argv[2]);
    } else if (argc == 5) {
        location[0] = atoi(argv[1]);
        power[0]    = atoi(argv[2]);
        location[1] = atoi(argv[3]);
        power[1]    = atoi(argv[4]);
    } else if (argc == 7) {
        location[0] = atoi(argv[1]);
        power[0]    = atoi(argv[2]);
        location[1] = atoi(argv[3]);
        power[1]    = atoi(argv[4]);
        location[2] = atoi(argv[5]);
        power[2]    = atoi(argv[6]);
    } else {
        shell_error(shell, "Usage: %s %s <location> <power> ...\n", argv[-1], argv[0]);
        return 1;
    }
    impl.to_location(location, power, detail);
    return 0;
}

static int cmd_current(const shell *shell, size_t argc, char **argv)
{
    impl.set_current_monitor();
    return 0;
}

static int cmd_info(const shell *shell, size_t argc, char **argv)
{
    impl.info(shell);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_act,
    SHELL_CMD(init, NULL, "Actuator initialize command", cmd_init),
    SHELL_CMD(loc, NULL, "Actuator locate command", cmd_locate),
    SHELL_CMD(current, NULL, "Actuator current monitor", cmd_current),
    SHELL_CMD(info, NULL, "Actuator information", cmd_info),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(act, &sub_act, "Actuator commands", NULL);

}

void actuator_controller::init()
{
    impl.init();
}

void actuator_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
    impl.run_error();
}

int actuator_controller::init_location()
{
    return impl.init_location();
}

int actuator_controller::to_location(const uint8_t location[ACTUATOR_NUM], const uint8_t power[ACTUATOR_NUM], uint8_t detail[ACTUATOR_NUM])
{
    return impl.to_location(location, power, detail);
}

k_thread actuator_controller::thread;

// vim: set expandtab shiftwidth=4:
