//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_RNW_PAYLOAD_H
#define UWB_COMM_RNW_PAYLOAD_H

#include "uart_odom/payload.h"
#include <nav_msgs/Odometry.h>
#include "mini_odom.h"

using mini_odom_t = Mini_odom<float,int>;

namespace uwb_comm {

    struct rnw_payload_t: public payload_base_t {

        ros::NodeHandle & nh;

        ros::Subscriber sub_odom_uav;

        ros::Publisher pub_odom_uav;

        ros::Publisher pub_odom_cone;

        nav_msgs::Odometry latest_msg;

        explicit rnw_payload_t( ros::NodeHandle & );

        void on_odom_uav( nav_msgs::OdometryConstPtr const & );

    public:

        void init_as_master() final;

        void init_as_slave() final;

        void encode(char * buffer) final;

        void decode(char const * buffer) final;

        int data_length() final;

    };

}

#endif //UWB_COMM_RNW_PAYLOAD_H
