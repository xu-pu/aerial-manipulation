//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_PAYLOAD_H
#define UWB_COMM_PAYLOAD_H

#include <ros/ros.h>

namespace uwb_comm {

    struct payload_base_t {

        //========== Master Node ============

        /**
         * Register ros::Publisher and ros::Subscriber here
         */
        virtual void init_as_master() = 0;

        virtual int data_length_master() = 0;

        /**
         * Encode data to be sent from master
         * @param buffer
         * @return if true, send the data, if false, don't send anything
         */
        virtual bool master_encode(char * buffer) = 0;

        /**
         * Decode data sent from slave
         * @param buffer
         */
        virtual void master_decode(char const * buffer) = 0;

        //========== Slave Node ============

        /**
         * Register ros::Publisher and ros::Subscriber here
         */
        virtual void init_as_slave() = 0;

        virtual int data_length_slave() = 0;

        /**
         * Encode data to be sent from slave
         * @param buffer
         * @return if true, send the data, if false, don't send anything
         */
        virtual bool slave_encode(char * buffer) = 0;

        /**
         * Decode data sent from master
         * @param buffer
         */
        virtual void slave_decode(char const * buffer) = 0;

    };

}


#endif //UWB_COMM_PAYLOAD_H
