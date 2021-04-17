//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_PAYLOAD_H
#define UWB_COMM_PAYLOAD_H

#include <ros/ros.h>

namespace uwb_comm {

    struct payload_base_t {

        /**
         * Register ros::Subscriber here
         */
        virtual void init_as_master() = 0;

        /**
         * Register ros::Publisher here
         */
        virtual void init_as_slave() = 0;

        /**
         * Encode and write data frame to the buffer
         * @param buffer
         */
        virtual void encode(char * buffer) = 0;

        /**
         * Decode data in the buffer and send it out
         * @param buffer
         */
        virtual void decode(char const * buffer) = 0;

        virtual int data_length() = 0;

    };

}


#endif //UWB_COMM_PAYLOAD_H
