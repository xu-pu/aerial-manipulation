//
// Created by sheep on 2021/4/18.
//

#ifndef UWB_COMM_RNW_PAYLOAD_H
#define UWB_COMM_RNW_PAYLOAD_H

#include "uart_odom/payload.h"

namespace uwb_comm {

    struct rnw_payload_t: public payload_base_t {

        ros::NodeHandle & nh;

        explicit rnw_payload_t( ros::NodeHandle & );

        void init_as_master() final;

        void init_as_slave() final;

        void encode(char * buffer) final;

        void decode(char const * buffer) final;

        int data_length() final;

    };

}

#endif //UWB_COMM_RNW_PAYLOAD_H
