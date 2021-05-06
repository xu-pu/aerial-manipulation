#include "uwb_transceiver/serial_service.h"
#include "uwb_transceiver/payload.h"
#include "uwb_transceiver/drone_swarm_payload.h"

using namespace std;

struct uwb_transceiver_t {

    enum class role_e {
        unset = 0,
        master = 1,
        slave = 2,
    } m_role = role_e::unset;

    // configuration (from ros)

    string m_para_serial_port;
    int m_para_baud_rate;
    int m_para_serial_protocal_mode = 1;
    int m_para_read_timer_frequency;
    int m_para_sent_timer_frequency;
    int m_para_odom_packet_id;
    int m_para_test_packet_id;
    int m_para_test_packet_size;
    int m_para_test_send_total;
    int m_para_debug_protocal;
    int m_para_debug_packet_info;

    // state

    serial::Serial  m_serial;

    Protocal_to_mcu m_protocal_to_mcu;

    Serial_packet   m_serial_send_pack;

    int m_receiver_counter = 0;

    double m_last_time;
    double m_current_time;
    double m_send_last_time;
    double m_send_current_time;

    ros::Timer m_timer_receive;
    ros::Timer m_timer_send;

    ros::NodeHandle nh;

    std::shared_ptr<uwb_comm::payload_base_t> payload_ptr;

    template < typename TName, typename TVal >
    void read_essential_param( const TName &name, TVal &val ) {
      if ( nh.getParam( name, val ) )
      {
        // pass
      }
      else
      {
        ROS_FATAL_STREAM( "[uwb_transceiver] Read param: " << name << " failed." );
        ROS_BREAK();
      }
    };

    void load_parameter(){
      cout << "=====  load_parameter ===== " << endl;
      ros::Duration( 1.0 ).sleep();

      read_essential_param( "serial_port", m_para_serial_port );
      read_essential_param( "baud_rate", m_para_baud_rate );
      read_essential_param( "read_timer_frequency", m_para_read_timer_frequency );
      read_essential_param( "send_timer_frequency", m_para_sent_timer_frequency );
      read_essential_param( "serial_protocal_mode", m_para_serial_protocal_mode );
      read_essential_param( "odom_packet_id", m_para_odom_packet_id );

      read_essential_param( "test/send_total", m_para_test_send_total );
      read_essential_param( "test/send_packet_size", m_para_test_packet_size );
      read_essential_param( "test/send_packet_size", m_para_test_packet_size );

      read_essential_param( "debug/m_debug_packet_info", m_para_debug_packet_info );
      read_essential_param( "debug/m_debug_protocal_info", m_para_debug_protocal );

      cout << "serial_port:    " << m_para_serial_port << endl;
      cout << "baud_rate:      " << m_para_baud_rate << endl;
      cout << "read_timer_frequency:    " << m_para_read_timer_frequency << endl;
      cout << "send_timer_frequency:      " << m_para_sent_timer_frequency << endl;
      cout << "serial_protocal_mode:    " << m_para_serial_protocal_mode << endl;
      cout << "odom_packet_id:    " << m_para_odom_packet_id << endl;

      cout << "test/send_total:    " << m_para_test_send_total << endl;
      cout << "test/send_packet_size:      " << m_para_test_send_total << endl;

      cout << "debug/m_debug_packet_info:      " << m_para_debug_packet_info << endl;
      cout << "debug/m_para_debug_protocal:      " << m_para_debug_protocal << endl;
      cout << "=====  load_parameter finish ===== " << endl;

    }

    void send_data(const ros::TimerEvent &event){
      if ( m_role == role_e::slave && payload_ptr->slave_encode(m_serial_send_pack.data) ) {
        m_serial_send_pack.id = m_para_odom_packet_id;
        m_serial_send_pack.data_length = payload_ptr->data_length_slave();
        m_send_current_time = ros::Time::now().toSec();
        m_protocal_to_mcu.send_packet(m_serial_send_pack);
        m_send_last_time = m_send_current_time;
      }
      else if ( m_role == role_e::master && payload_ptr->master_encode(m_serial_send_pack.data) ) {
        m_serial_send_pack.id = m_para_odom_packet_id;
        m_serial_send_pack.data_length = payload_ptr->data_length_master();
        m_send_current_time = ros::Time::now().toSec();
        m_protocal_to_mcu.send_packet(m_serial_send_pack);
        m_send_last_time = m_send_current_time;
      }
    }

    void receive_data(const ros::TimerEvent &event){

      // try to receive data
      Serial_packet pack;
      int pack_num = m_protocal_to_mcu.receive_packet(pack);
      if ( pack_num == 0 ) return;

      // record the data
      m_receiver_counter++;
      m_current_time = ros::Time::now().toSec();
      if ( m_para_debug_packet_info ) {
        ROS_INFO_STREAM("Packet  " << m_receiver_counter << ", length = " << pack.data_length << ", freq = " << (int) (1.0 / (m_current_time - m_last_time )));
      }
      m_last_time = m_current_time;

      // decode and dispatch the data
      if (pack.id == m_para_odom_packet_id ){
        if (m_role == role_e::master ) {
          payload_ptr->master_decode(pack.data);
        }
        else if ( m_role == role_e::slave ){
          payload_ptr->slave_decode(pack.data);
        }
      }
      else {
        ROS_WARN_STREAM("[uwb_transceiver] unexpected packet");
      }

    };

    void init_as_master(){
      ROS_INFO( "[uwb_transceiver]: init as master node" );
      payload_ptr->init_as_master();
    }

    void init_as_slave(){
      ROS_INFO( "[uwb_transceiver]: init as slave node" );
      payload_ptr->init_as_slave();
    }

    uwb_transceiver_t():nh("~"){

      load_parameter();

      //payload_ptr = std::make_shared<uwb_comm::rnw_payload_t>(nh);
      payload_ptr = std::make_shared<uwb_comm::drone_swarm_payload_t>(nh);

      int role = (int)role_e::unset;
      nh.getParam("role",role);
      m_role = (role_e)role;

      switch (m_role) {
        case role_e::master:
          init_as_master();
          break;
        case role_e::slave:
          init_as_slave();
          break;
        default:
          ROS_FATAL("[uwb_transceiver] did not assign a role!");
          ROS_BREAK();
      }

      m_timer_receive = nh.createTimer(ros::Rate(m_para_read_timer_frequency), &uwb_transceiver_t::receive_data, this );
      m_timer_send = nh.createTimer(ros::Rate(m_para_sent_timer_frequency), &uwb_transceiver_t::send_data, this );

      try {

        m_serial.setPort( m_para_serial_port );
        m_serial.setBaudrate( m_para_baud_rate ); // 10000/10000 packet, all rec
        m_serial.open();

        m_protocal_to_mcu.set_protocal_mode( m_para_serial_protocal_mode );
        m_protocal_to_mcu.set_debug_info( m_para_debug_protocal );
        m_protocal_to_mcu.init( &m_serial );

      } catch (...) {
        ROS_FATAL("[uwb_transceiver] Open serial fail !!");
        ROS_BREAK();
      }

      if ( m_serial.isOpen() ) {
        ROS_INFO_STREAM("[uwb_transceiver] Open serial success !!");
      }
      else {
        ROS_FATAL("[uwb_transceiver] Open serial fail !!");
        ROS_BREAK();
      }

    }

};

int main( int argc, char *argv[] ) {
  ros::init( argc, argv, "uwb_transceiver_node" );
  uwb_transceiver_t uwb_transceiver;
  ros::MultiThreadedSpinner spinner( 6 ); // Use 4 threads
  spinner.spin();
  ros::shutdown();
  return 0;
}
