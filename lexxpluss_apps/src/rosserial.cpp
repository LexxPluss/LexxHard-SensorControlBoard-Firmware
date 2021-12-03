#include "rosserial_hardware_zephyr.hpp"
#include "rosserial_actuator.hpp"
#include "rosserial_bmu.hpp"
#include "rosserial_board.hpp"
#include "rosserial_diag.hpp"
#include "rosserial_imu.hpp"
#include "rosserial_led.hpp"
#include "rosserial_pgv.hpp"
#include "rosserial_tof.hpp"
#include "rosserial_uss.hpp"
#include "rosserial.hpp"

namespace {

class rosserial_impl {
public:
    int init() {
        nh.getHardware()->set_baudrate(921600);
        nh.initNode(const_cast<char*>("UART_1"));
        actuator.init(nh);
        bmu.init(nh);
        board.init(nh);
        diag.init(nh);
        imu.init(nh);
        led.init(nh);
        pgv.init(nh);
        tof.init(nh);
        uss.init(nh);
        return 0;
    }
    void run() {
        while (true) {
            nh.spinOnce();
            actuator.poll();
            bmu.poll();
            board.poll();
            diag.poll();
            imu.poll();
            led.poll();
            pgv.poll();
            tof.poll();
            uss.poll();
            k_usleep(1);
        }
    }
private:
    ros::NodeHandle nh;
    ros_actuator actuator;
    ros_bmu bmu;
    ros_board board;
    ros_diag diag;
    ros_imu imu;
    ros_led led;
    ros_pgv pgv;
    ros_tof tof;
    ros_uss uss;
} impl;

}

void rosserial::init()
{
    impl.init();
}

void rosserial::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread rosserial::thread;

// vim: set expandtab shiftwidth=4:
