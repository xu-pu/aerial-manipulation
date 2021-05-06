#ifndef UWB_COMM_RNW_PAYLOAD_H
#define UWB_COMM_RNW_PAYLOAD_H

#include <nav_msgs/Odometry.h>

#include "uwb_transceiver/payload.h"

namespace uwb_comm {

    struct rnw_payload_t: public payload_base_t {

        ros::NodeHandle & nh;

        ros::Subscriber sub_odom_uav;

        ros::Publisher pub_odom_uav;

        ros::Publisher pub_odom_cone;

        nav_msgs::Odometry latest_msg;

        bool init = false;

        explicit rnw_payload_t( ros::NodeHandle & );

        void on_odom_uav( nav_msgs::OdometryConstPtr const & );

    public:

        void init_as_master() final;

        void init_as_slave() final;

        bool master_encode(char * buffer) final;

        void slave_decode(char const * buffer) final;

        int data_length_master() final;

    };

}

#endif //UWB_COMM_RNW_PAYLOAD_H
