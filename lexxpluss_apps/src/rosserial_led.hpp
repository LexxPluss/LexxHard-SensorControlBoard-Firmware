#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "lexxauto_msgs/Led.h"
#include "led_controller.hpp"

namespace lexxfirm {

class ros_led {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub_string);
        nh.subscribe(sub_direct);
    }
    void poll() {}
private:
    void callback_string(const std_msgs::String &req) {
        led_controller::msg message{req.data};
        while (k_msgq_put(&led_controller::msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&led_controller::msgq);
    }
    void callback_direct(const lexxauto_msgs::Led &req) {
        led_controller::msg message;
        message.pattern = led_controller::msg::RGB;
        message.interrupt_ms = 0;
        message.rgb[0] = req.r;
        message.rgb[1] = req.g;
        message.rgb[2] = req.b;
        while (k_msgq_put(&led_controller::msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&led_controller::msgq);
    }
    ros::Subscriber<std_msgs::String, ros_led> sub_string{"/body_control/led", &ros_led::callback_string, this};
    ros::Subscriber<lexxauto_msgs::Led, ros_led> sub_direct{"/body_control/led_direct", &ros_led::callback_direct, this};
};

}

// vim: set expandtab shiftwidth=4:
