#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "serial_service.h"
#include "uart_odom/payload.h"
#include "uart_odom/rnw_payload.h"
#include "uart_odom/drone_swarm_payload.h"

using namespace std;

class Uart_odom
{
    enum e_role
    {
        e_role_unset = 0,
        e_master = 1,
        e_role_slave = 2,
    };

public:
    serial::Serial  m_serial;
    Protocal_to_mcu m_protocal_to_mcu;
    Serial_packet   m_serial_pack;
    Serial_packet   m_serial_send_pack;
    Serial_packet   m_serial_read_pack;

    int m_test_rec_count = 0;

    e_role m_role = e_role_unset; // The master or slave

    string m_para_serial_port;
    int    m_para_baud_rate;
    int    m_para_serial_protocal_mode = 1;
    int    m_para_read_timer_frequency;
    int    m_para_sent_timer_frequency;
    int    m_para_odom_packet_id;

    int m_para_test_packet_id;
    int m_para_test_packet_size;
    int m_para_test_send_total;

    int m_para_debug_protocal;
    int m_para_debug_packet_info;

    double     m_last_time;
    double     m_current_time;
    double     m_send_last_time;
    double     m_send_current_time;
    ros::Timer m_timer_test_send;
    ros::Timer m_timer_test_read;

    ros::Timer m_timer_send_odom;

    ros::NodeHandle m_ros_nh;
    int                m_if_odom_init = 0;
    int                m_idx_odom;

    std::shared_ptr<uwb_comm::payload_base_t> payload_ptr;

public:
    void send_service_odom_message( const ros::TimerEvent &event ) {
      //        printf("enter odom_message_send_service");

      if ( m_role == e_role_slave ) {
        if (payload_ptr->slave_encode(m_serial_send_pack.data)){
          m_serial_send_pack.id = m_para_odom_packet_id;
          m_serial_send_pack.data_length = payload_ptr->data_length_slave();
          m_send_current_time = ros::Time::now().toSec();
          m_protocal_to_mcu.send_packet( m_serial_send_pack );
          m_send_last_time = m_send_current_time;
        }
      }
      else if (m_role == e_master ) {
        if ( payload_ptr->encode(m_serial_send_pack.data) ) {
          m_serial_send_pack.id = m_para_odom_packet_id;
          m_serial_send_pack.data_length = payload_ptr->data_length();
          m_send_current_time = ros::Time::now().toSec();
          m_protocal_to_mcu.send_packet( m_serial_send_pack );
          //cout << (1.0 / (ros::Time::now().toSec() - m_send_current_time) ) <<endl;
          //cout << "Send frequency = " << ( int ) ( 1.0 / ( m_send_current_time - m_send_last_time ) ) << endl;
          m_send_last_time = m_send_current_time;
        }
      }

    }

    void send_service_eval_stability( const ros::TimerEvent &event )
    {

      char str[ 2048 ] = "Hello~\r\n";
      for ( int i = 10; i < m_para_test_packet_size; i++ )
      {
        str[ i ] = i & 0xff;
      }
      //cout << "Enter eval_stability_send_service" << endl;
      m_serial_pack.id = m_para_test_packet_id;
      m_serial_pack.data_length = m_para_test_packet_size;
      memcpy( m_serial_pack.data, str, m_para_test_packet_size );
      m_para_test_send_total--;
      if ( m_para_test_send_total >= 0 )
      {
        m_protocal_to_mcu.send_packet( m_serial_pack );
        //protocal_to_mcu.display( 100 );
        //ser_pack.display();
      }
    };

    template < typename TName, typename TVal >
    void read_essential_param( const ros::NodeHandle &nh, const TName &name, TVal &val )
    {
      if ( nh.getParam( name, val ) )
      {
        // pass
      }
      else
      {
        ROS_ERROR_STREAM( "Read param: " << name << " failed." );
        ROS_BREAK();
      }
    };

    void load_parameter( const ros::NodeHandle &nh )
    {
      cout << "=====  load_parameter ===== " << endl;
      ros::Duration( 1.0 ).sleep();

      read_essential_param( nh, "serial_port", m_para_serial_port );
      read_essential_param( nh, "baud_rate", m_para_baud_rate );
      read_essential_param( nh, "read_timer_frequency", m_para_read_timer_frequency );
      read_essential_param( nh, "send_timer_frequency", m_para_sent_timer_frequency );
      read_essential_param( nh, "serial_protocal_mode", m_para_serial_protocal_mode );
      read_essential_param( nh, "odom_packet_id", m_para_odom_packet_id );

      read_essential_param( nh, "test/send_total", m_para_test_send_total );
      read_essential_param( nh, "test/send_packet_size", m_para_test_packet_size );
      read_essential_param( nh, "test/send_packet_size", m_para_test_packet_size );

      read_essential_param( nh, "debug/m_debug_packet_info", m_para_debug_packet_info );
      read_essential_param( nh, "debug/m_debug_protocal_info", m_para_debug_protocal );

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

    void read_serive_eval_stability( const ros::TimerEvent &event ) {
      Serial_packet temp_serial_pack;
      int pack_num = m_protocal_to_mcu.receive_packet( temp_serial_pack );
      if ( pack_num != 0 ) { // Receive packet
        m_test_rec_count++;
        m_current_time = ros::Time::now().toSec();
        if ( m_para_debug_packet_info ) {
          cout << "Packet  " << m_test_rec_count
               << ", length = " << temp_serial_pack.data_length
               << ", freq = " << (int) (1.0 / ( m_current_time - m_last_time ))
               << endl;
        }
        m_last_time = m_current_time;
        if ( temp_serial_pack.id == m_para_odom_packet_id ){
          if ( m_role == e_master ) {
            payload_ptr->master_decode(temp_serial_pack.data);
          }
          else if ( m_role == e_role_slave ){
            payload_ptr->decode(temp_serial_pack.data);
          }
        }
      }
    };

    void test_uart() {
      cout << " ===== test_uart =====" << endl;
      m_timer_test_send = m_ros_nh.createTimer( ros::Duration( 1.0 / m_para_sent_timer_frequency ), &Uart_odom::send_service_eval_stability, this );
      m_timer_test_send.start();
      m_timer_test_read.start();
      cout << "test_uart exit" << endl;
    };

    void init_as_master(){
      ROS_INFO( "[UART_odom]: I have receive the odom,  therefore I am master." );
      //m_timer_test_send.stop(); // Turn of test sender.
      payload_ptr->init_as_master();
    }

    void init_as_slave(){
      ROS_INFO( "[UART_ODOM]: I read the odom from uart, therefore I am slave" );
      //m_timer_test_send.stop(); // Turn of test sender.
      payload_ptr->init_as_slave();
    }

    explicit Uart_odom( ros::NodeHandle &nh ) {

      m_ros_nh = nh;
      load_parameter( m_ros_nh );

      //payload_ptr = std::make_shared<uwb_comm::rnw_payload_t>(nh);
      payload_ptr = std::make_shared<uwb_comm::drone_swarm_payload_t>(nh);

      int role = e_role::e_role_unset;
      nh.getParam("role",role);
      m_role = (e_role)role;

      switch (m_role) {
        case e_role::e_master:
          init_as_master();
          break;
        case e_role::e_role_slave:
          init_as_slave();
          break;
        default:
          ROS_ERROR_STREAM("[UWB] did not assign a role!");
          exit(-1);
      }

      // m_timer_test_read = m_ros_nh.createTimer( ros::Duration( 1.0 / m_para_read_timer_frequency ), &Uart_odom::read_serive_eval_stability, this );
      m_timer_test_read = nh.createTimer( ros::Duration( 1.0 / m_para_read_timer_frequency ), &Uart_odom::read_serive_eval_stability, this );
      m_timer_send_odom = m_ros_nh.createTimer( ros::Duration( 1.0 / m_para_sent_timer_frequency ), &Uart_odom::send_service_odom_message, this );

      m_serial.setPort( m_para_serial_port );
      m_serial.setBaudrate( m_para_baud_rate ); // 10000/10000 packet, all rec
      m_serial.open();

      m_protocal_to_mcu.set_protocal_mode( m_para_serial_protocal_mode );
      m_protocal_to_mcu.set_debug_info( m_para_debug_protocal );

      if ( m_serial.isOpen() ) {
        cout << "Open serial success !!" << endl;
        m_protocal_to_mcu.init( &m_serial );
      }
      else {
        cout << "Open serial fail !! " << endl;
      }
    }
};

int main( int argc, char *argv[] ) {
  ros::init( argc, argv, "uart_odom" );
  ros::NodeHandle nh = ros::NodeHandle( "~" );
  Uart_odom uart_odom( nh );
  //uart_odom.test_uart();
  ros::MultiThreadedSpinner spinner( 6 ); // Use 4 threads
  spinner.spin();
  return 0;
}
