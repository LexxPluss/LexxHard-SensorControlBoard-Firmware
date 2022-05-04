#include "rosserial_hardware_zephyr.hpp"
#include "rosserial_actuator_service.hpp"
#include "rosserial_service.hpp"

namespace lexxhard::rosserial_service {

class {
public:
    int init() {
        nh.initNode(const_cast<char*>("UART_2"));
        actuator_service.init(nh);
        return 0;
    }
    void run() {
        while (true) {
            nh.spinOnce();
            k_usleep(1);
        }
    }
private:
    ros::NodeHandle nh;
    ros_actuator_service actuator_service;
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

}

// vim: set expandtab shiftwidth=4:
