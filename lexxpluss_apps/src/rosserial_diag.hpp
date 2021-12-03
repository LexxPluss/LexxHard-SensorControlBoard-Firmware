#pragma once

#include "ros/node_handle.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "rosdiagnostic.hpp"

class ros_diag {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub_diag);
        msg.status_length = 1;
        msg.status = status;
    }
    void poll() {
        msg_rosdiag message;
        while (k_msgq_get(&msgq_rosdiag, &message, K_NO_WAIT) == 0) {
            msg.status[0].level = message.level;
            msg.status[0].name = message.name;
            msg.status[0].message = message.message;
            msg.status[0].hardware_id = message.id;
            msg.status[0].values_length = 0;
            pub_diag.publish(&msg);
        }
    }
private:
    diagnostic_msgs::DiagnosticArray msg;
    diagnostic_msgs::DiagnosticStatus status[1];
    ros::Publisher pub_diag{"/diagnostics", &msg};
};

// vim: set expandtab shiftwidth=4:
