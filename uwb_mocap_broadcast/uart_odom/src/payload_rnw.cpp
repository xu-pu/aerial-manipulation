#include "uart_odom/rnw_payload.h"

using namespace uwb_comm;

rnw_payload_t::rnw_payload_t( ros::NodeHandle & _nh ) : nh(_nh) {}

void rnw_payload_t::init_as_master(){

}

void rnw_payload_t::init_as_slave() {

}

int rnw_payload_t::data_length() {

}

void rnw_payload_t::decode(const char *buffer) {

}

void rnw_payload_t::encode(char *buffer) {

}