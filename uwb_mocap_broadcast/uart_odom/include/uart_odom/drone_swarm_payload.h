//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_DRONE_SWARM_PAYLOAD_H
#define UWB_COMM_DRONE_SWARM_PAYLOAD_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "uart_odom/payload.h"

namespace uwb_comm {

    struct drone_swarm_payload_t: public payload_base_t {

        ros::NodeHandle & nh;

        ros::Subscriber sub_odom;

        ros::Subscriber sub_cmd;

        ros::Publisher pub_odom;

        ros::Publisher pub_cmd;

        nav_msgs::Odometry latest_odom;

        quadrotor_msgs::PositionCommand latest_cmd;

        bool init = false;

        explicit drone_swarm_payload_t( ros::NodeHandle & );

        void on_odom( nav_msgs::OdometryConstPtr const & );

        void on_cmd( quadrotor_msgs::PositionCommandConstPtr const & );

    public:

        void init_as_master() final;

        void init_as_slave() final;

        bool master_encode(char * buffer) final;

        void slave_decode(char const * buffer) final;

        int data_length_master() final;

        int data_length_slave() final;

        bool slave_encode(char * buffer) final;

        void master_decode(char const * buffer) final;

    };

}

#endif //UWB_COMM_DRONE_SWARM_PAYLOAD_H
