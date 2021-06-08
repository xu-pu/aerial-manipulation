//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_DRONE_SWARM_PAYLOAD_H
#define UWB_COMM_DRONE_SWARM_PAYLOAD_H

#include <queue>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <n3ctrl/N3CtrlState.h>
#include <std_msgs/UInt8.h>

#include "uwb_transceiver/payload.h"

namespace uwb_comm {

    struct drone_swarm_payload_t: public payload_base_t {

        void init_as_master() final;

        void init_as_slave() final;

        bool master_encode(char * buffer) final;

        void master_decode(char const * buffer) final;

        bool slave_encode(char * buffer) final;

        void slave_decode(char const * buffer) final;

        int data_length_master() final;

        int data_length_slave() final;

    public:

        ros::NodeHandle & nh;

        ros::Subscriber sub_odom;

        ros::Subscriber sub_cmd;

        ros::Subscriber sub_n3ctrl;

        ros::Subscriber sub_triggers;

        ros::Publisher pub_n3ctrl;

        ros::Publisher pub_odom;

        ros::Publisher pub_cmd;

        nav_msgs::Odometry latest_odom;

        quadrotor_msgs::PositionCommand latest_cmd;

        n3ctrl::N3CtrlState latest_n3ctrl;

        double msg_timeout_sec = 1;

        double odom_latency_ms = 0;

        ros::Duration odom_latency;

        std::queue<uint8_t> trigger_queue;

        explicit drone_swarm_payload_t( ros::NodeHandle & );

        void on_odom( nav_msgs::OdometryConstPtr const & );

        void on_cmd( quadrotor_msgs::PositionCommandConstPtr const & );

        void on_n3ctrl( n3ctrl::N3CtrlStateConstPtr const & );

        void on_triggers( std_msgs::UInt8ConstPtr const & );

        bool odom_on_time() const;

        bool n3ctrl_on_time() const;

        static void arm_motors();

    };

}

#endif //UWB_COMM_DRONE_SWARM_PAYLOAD_H
