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

#pragma once

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/can.h>
#include "ros/node_handle.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "lexxauto_msgs/LinearActuatorControlArray.h"
#include "actuator_controller.hpp"

#define CAN_ID_ACTUATOR_CONTROL 0x208 //based on saito-san's CAN ID assignment
#define CAN_ID_ACTUATOR_ENCODER 0x209 //based on saito-san's CAN ID assignment
#define CAN_ID_ACTUATOR_CURRENT 0x20a //based on saito-san's CAN ID assignment
#define CAN_DATALENGTH_ACTUATOR_CONTROL 6
#define CAN_DATALENGTH_ACTUATOR_ENCODER 6
#define CAN_DATALENGTH_ACTUATOR_CURRENT 8



namespace lexxhard {

char __aligned(4) msgq_can_actuator_control_buffer[8 * sizeof (ms)];


class ros_actuator {
public:
    int init() {
        // msg_encoder.data = msg_encoder_data;
        // msg_encoder.data_length = sizeof msg_encoder_data / sizeof msg_encoder_data[0];
        // msg_connection.data = msg_connection_data;
        // msg_connection.data_length = sizeof msg_connection_data / sizeof msg_connection_data[0];
        // msg_current.data = msg_current_data;
        // msg_current.data_length = sizeof msg_current_data / sizeof msg_current_data[0];

        //can device bind`
        k_msgq_init(&msgq_can_actuator_control, msgq_can_actuator_control_buffer, 8/*???*/, 8);
        dev = device_get_binding("CAN_2");
        if (!device_is_ready(dev))
            return -1;

        //setup can filter
        static const zcan_filter filter_actuator_control{
            .id{CAN_ID_ACTUATOR_CONTROL},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7ff},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can_actuator_control, &filter_actuator_control);
        return 0;
    }
    void poll() {
        zcan_frame can_frame_actuator_encoder{
            .id = CAN_ID_ACTUATOR_ENCODER,
            .rtr = CAN_DATAFRAME,
            .id_type = CAN_STANDARD_IDENTIFIER,
            .dlc = CAN_DATALENGTH_ACTUATOR_ENCODER
        };
        zcan_frame can_frame_actuator_current{
            .id = CAN_ID_ACTUATOR_CURRENT,
            .rtr = CAN_DATAFRAME,
            .id_type = CAN_STANDARD_IDENTIFIER,
            .dlc = CAN_DATALENGTH_ACTUATOR_CURRENT
        };
        //actuator_controller::can_format_encoder tmp_frame_encoder;
        //actuator_controller::can_format_current tmp_frame_current;
        //send to IPC of sensor informations
        actuator_controller::msg message;
        while (k_msgq_get(&actuator_controller::msgq, &message, K_NO_WAIT) == 0) {
            // ROS:[center,left,right], ROBOT:[left,center,right

            /* memcpyなどを使って.dataにencoder=countなどを詰める。*/




            // memcpy(can_frame_actuator_encoder.data, actuator_controller::can_format_encoder(message.encoder_count[1], message.encoder_count[0], message.encoder_count[2]);
            // can_frame_actuator_current.data = actuator_controller::can_format_encoder(message.current[1], message.current[0], message.current[2]);
            // frame_enc.encoder_count[0] = message.encoder_count[1];
            // frame_enc.encoder_count[1] = message.encoder_count[0];
            // frame_enc.encoder_count[2] = message.encoder_count[2];
            //pub_encoder.publish(&msg_encoder);
            frame_cur.connection_mv = message.connect * 1e-3f;
            //pub_connection.publish(&msg_connection);
            frame_cur.current_mv[0] = message.current[1] * 1e-3f;
            frame_cur.current_mv[1] = message.current[0] * 1e-3f;
            frame_cur.current_mv[2] = message.current[2] * 1e-3f;
            //pub_current.publish(&msg_current);
            
            
            can_send(dev, &can_frame_actuator_encoder, K_MSEC(100), nullptr, nullptr);
            can_send(dev, &can_frame_actuator_current, K_MSEC(100), nullptr, nullptr);

        }

        //receive from IPC of motion control
        while (k_msgq_get(&msgq_can_actuator_control, &frame_cntl, K_NO_WAIT) == 0) {
            msg_cntl = actuator_controller::msg_control(frame_cntl);
            while (k_msgq_put(&actuator_controller::msgq_control, &msg_cntl, K_NO_WAIT) != 0)
                k_msgq_purge(&actuator_controller::msgq_control);
        }
    }
private:

    // static const zcan_frame can_frame_actuator_control{
    //     .id = CAN_ID_ACTUATOR_CONTROL,
    //     .rtr = CAN_DATAFRAME,
    //     .id_type = CAN_STANDARD_IDENTIFIER,
    //     .dlc = CAN_DATALENGTH_ACTUATOR_CONTROL
    // };
    // static const zcan_frame can_frame_actuator_encoder{
    //     .id = CAN_ID_ACTUATOR_ENCODER,
    //     .rtr = CAN_DATAFRAME,
    //     .id_type = CAN_STANDARD_IDENTIFIER,
    //     .dlc = CAN_DATALENGTH_ACTUATOR_ENCODER
    // };
    // static const zcan_frame can_frame_actuator_current{
    //     .id = CAN_ID_ACTUATOR_CURRENT,
    //     .rtr = CAN_DATAFRAME,
    //     .id_type = CAN_STANDARD_IDENTIFIER,
    //     .dlc = CAN_DATALENGTH_ACTUATOR_CURRENT
    // };
    
    void callback_control(const lexxauto_msgs::LinearActuatorControlArray &req) {
        actuator_controller::msg_control message;
        // ROS:[center,left,right], ROBOT:[left,center,right]
        message.actuators[0].direction = req.actuators[1].direction;
        message.actuators[1].direction = req.actuators[0].direction;
        message.actuators[2].direction = req.actuators[2].direction;
        message.actuators[0].power = req.actuators[1].power;
        message.actuators[1].power = req.actuators[0].power;
        message.actuators[2].power = req.actuators[2].power;
        while (k_msgq_put(&actuator_controller::msgq_control, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&actuator_controller::msgq_control);
    }
    //std_msgs::Int32MultiArray msg_encoder;
    actuator_controller::can_format_control frame_cntl;
    //actuator_controller::can_format_current frame_cur;
    //actuator_controller::can_format_encoder frame_enc;
    actuator_controller::msg_control msg_cntl;
    const device *dev{nullptr};

    //std_msgs::Float32MultiArray msg_connection, msg_current;
    //int32_t msg_encoder_data[3];
    //float msg_connection_data[1], msg_current_data[3];
    // ros::Publisher pub_encoder{"/body_control/encoder_count", &msg_encoder};
    // ros::Publisher pub_connection{"/body_control/shelf_connection", &msg_connection};
    // ros::Publisher pub_current{"/body_control/linear_actuator_current", &msg_current};
    // ros::Subscriber<lexxauto_msgs::LinearActuatorControlArray, ros_actuator>
    //     sub_control{"/body_control/linear_actuator", &ros_actuator::callback_control, this};

};

k_msgq msgq_can_actuator_control;

}

// vim: set expandtab shiftwidth=4:
