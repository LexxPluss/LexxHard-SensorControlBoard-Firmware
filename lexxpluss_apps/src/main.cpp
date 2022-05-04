#include <zephyr.h>
#include <drivers/gpio.h>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
#include "can_controller.hpp"
#include "imu_controller.hpp"
#include "led_controller.hpp"
#include "misc_controller.hpp"
#include "pgv_controller.hpp"
#include "rosserial.hpp"
#include "rosserial_service.hpp"
#include "tof_controller.hpp"
#include "uss_controller.hpp"

namespace {

K_THREAD_STACK_DEFINE(actuator_controller_stack, 2048);
K_THREAD_STACK_DEFINE(adc_reader_stack, 2048);
K_THREAD_STACK_DEFINE(can_controller_stack, 2048);
K_THREAD_STACK_DEFINE(imu_controller_stack, 2048);
K_THREAD_STACK_DEFINE(led_controller_stack, 2048);
K_THREAD_STACK_DEFINE(misc_controller_stack, 2048);
K_THREAD_STACK_DEFINE(pgv_controller_stack, 2048);
K_THREAD_STACK_DEFINE(rosserial_stack, 2048);
K_THREAD_STACK_DEFINE(rosserial_service_stack, 2048);
K_THREAD_STACK_DEFINE(tof_controller_stack, 2048);
K_THREAD_STACK_DEFINE(uss_controller_stack, 2048);

#define RUN(name, prio) \
    k_thread_create(&lexxhard::name::thread, name##_stack, K_THREAD_STACK_SIZEOF(name##_stack), \
                    lexxhard::name::run, nullptr, nullptr, nullptr, prio, K_FP_REGS, K_MSEC(2000));

void reset_usb_hub()
{
    if (const device *gpioa{device_get_binding("GPIOA")}; device_is_ready(gpioa)) {
        gpio_pin_configure(gpioa, 3, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        k_msleep(1);
        gpio_pin_set(gpioa, 3, 0);
        k_msleep(1);
        gpio_pin_set(gpioa, 3, 1);
    }
}

}

void main()
{
    reset_usb_hub();
    lexxhard::actuator_controller::init();
    lexxhard::adc_reader::init();
    lexxhard::can_controller::init();
    lexxhard::imu_controller::init();
    lexxhard::led_controller::init();
    lexxhard::misc_controller::init();
    lexxhard::pgv_controller::init();
    lexxhard::rosserial::init();
    lexxhard::rosserial_service::init();
    lexxhard::tof_controller::init();
    lexxhard::uss_controller::init();
    RUN(actuator_controller, 2);
    RUN(adc_reader, 2);
    RUN(can_controller, 4);
    RUN(imu_controller, 2);
    RUN(led_controller, 1);
    RUN(misc_controller, 2);
    RUN(pgv_controller, 1);
    RUN(tof_controller, 2);
    RUN(uss_controller, 2);
    RUN(rosserial, 5); // The rosserial thread will be started last.
    RUN(rosserial_service, 6); // The rosserial thread will be started last.
    const device *gpiog{device_get_binding("GPIOG")};
    if (gpiog != nullptr)
        gpio_pin_configure(gpiog, 12, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    int heartbeat_led{1};
    while (true) {
        if (gpiog != nullptr) {
            gpio_pin_set(gpiog, 12, heartbeat_led);
            heartbeat_led = !heartbeat_led;
        }
        k_msleep(1000);
    }
}

// vim: set expandtab shiftwidth=4:
