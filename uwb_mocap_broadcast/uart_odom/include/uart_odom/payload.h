//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_PAYLOAD_H
#define UWB_COMM_PAYLOAD_H

#include <ros/ros.h>

namespace uwb_comm {

    struct payload_base_t {

        ros::NodeHandle & nh;

        explicit payload_base_t( ros::NodeHandle & );



    };

}


#endif //UWB_COMM_PAYLOAD_H
