#include "mini_odom.h"
#include "serial_protocal.h"
#include "serial_service.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <stdio.h>

#include "uart_odom/payload.h"

using namespace std;

using mini_odom_t = Mini_odom<float,int>;

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
    ros::Subscriber m_sub_odom;
    ros::Subscriber m_sub_odom_cone;
    ros::Publisher  m_pub_odom;
    ros::Publisher  m_pub_odom_cone;
    ros::Publisher  m_pub_odom_test;
    nav_msgs::Odometry m_init_odom, m_current_odom, m_current_odom_cone;
    int                m_if_odom_init = 0;
    int                m_idx_odom;

    std::shared_ptr<uwb_comm::payload_base_t> payload_ptr;

public:
    void send_service_odom_message( const ros::TimerEvent &event )
    {

      //        printf("enter odom_message_send_service");
      m_serial_send_pack.id = m_para_odom_packet_id;
      m_serial_send_pack.data_length = sizeof(mini_odom_t) * 2;
      mini_odom_t m_mini_odom_uav, m_mini_odom_cone;
      odom_to_miniodom<float, int>(m_current_odom, m_mini_odom_uav );
      odom_to_miniodom<float, int>( m_current_odom_cone, m_mini_odom_cone );
      memcpy(m_serial_send_pack.data, (char*)&m_mini_odom_uav, sizeof(mini_odom_t));
      memcpy(m_serial_send_pack.data + sizeof(mini_odom_t), (char*)&m_mini_odom_cone, sizeof(mini_odom_t));
      m_send_current_time = ros::Time::now().toSec();

      m_protocal_to_mcu.send_packet( m_serial_send_pack );
      //cout << (1.0 / (ros::Time::now().toSec() - m_send_current_time) ) <<endl;
      //cout << "Send frequency = " << ( int ) ( 1.0 / ( m_send_current_time - m_send_last_time ) ) << endl;
      m_send_last_time = m_send_current_time;

    }

    void odom_callback( const nav_msgs::Odometry &msg )
    {
      m_idx_odom++;
      m_current_odom = msg;
      if ( m_if_odom_init == 0 )
      {
        m_init_odom = msg;
        m_if_odom_init = 1;
      }
    };

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

    void read_serive_eval_stability( const ros::TimerEvent &event )
    {
      //cout << "Enter read_serive_eval_stability" << endl;
      Serial_packet temp_serial_pack;
      int           pack_num = m_protocal_to_mcu.receive_packet( temp_serial_pack );
      if ( pack_num != 0 ) // Receive packet
      {
        m_test_rec_count++;
        m_current_time = ros::Time::now().toSec();
        if ( m_para_debug_packet_info )
        {
          cout << "Packet  " << m_test_rec_count
               << ", length = " << temp_serial_pack.data_length
               << ", freq = " << (int) (1.0 / ( m_current_time - m_last_time ))
               << endl;
        }
        m_last_time = m_current_time;
        if ( temp_serial_pack.id == m_para_test_packet_id )
        {
          return;
        }
        else if ( temp_serial_pack.id == m_para_odom_packet_id )
        {
          if ( m_role == e_role_unset )
          {
            m_role = e_role_slave;
            ROS_INFO( "[UART_ODOM]: I read the odom from uart, therefore I am slave" );
            m_timer_test_send.stop(); // Turn of test sender.
          }

          //cout << __FUNCTION__ << "  " << __LINE__ << endl;

          mini_odom_t m_read_mini_odom_uav, m_read_mini_odom_cone;
          memcpy(&m_read_mini_odom_uav, temp_serial_pack.data, sizeof(mini_odom_t));
          memcpy(&m_read_mini_odom_cone, temp_serial_pack.data+sizeof(mini_odom_t), sizeof(mini_odom_t));
          nav_msgs::Odometry odom_uav, odom_cone;
          miniodom_to_odom(m_read_mini_odom_uav, odom_uav);
          miniodom_to_odom(m_read_mini_odom_cone, odom_cone);
          odom_uav.header.frame_id = "world";
          odom_cone.header.frame_id = "world";
          m_pub_odom.publish(odom_uav);
          m_pub_odom_cone.publish(odom_cone);

          //cout << "Finish publish" << endl;
        }
        else
        {
          return;
        }

        //ser_pack.display();
        //ros::Duration( 1).sleep();
      }
    };

    void test_uart()
    {
      cout << " ===== test_uart =====" << endl;
      m_timer_test_send = m_ros_nh.createTimer( ros::Duration( 1.0 / m_para_sent_timer_frequency ), &Uart_odom::send_service_eval_stability, this );

      m_timer_test_send.start();
      m_timer_test_read.start();

      cout << "test_uart exit" << endl;
    };

    void init_as_master(){
      ROS_INFO( "[UART_odom]: I have receive the odom,  therefore I am master." );
      m_timer_send_odom = m_ros_nh.createTimer( ros::Duration( 1.0 / m_para_sent_timer_frequency ), &Uart_odom::send_service_odom_message, this );
      m_timer_test_send.stop(); // Turn of test sender.

      payload_ptr->init_as_master();

    }

    void init_as_slave(){
      payload_ptr->init_as_slave();
    }

    explicit Uart_odom( ros::NodeHandle &nh ) {

      m_ros_nh = nh;
      load_parameter( m_ros_nh );

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

      m_sub_odom = m_ros_nh.subscribe(
              "odom_in",
              1,
              &Uart_odom::odom_callback,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      // m_timer_test_read = m_ros_nh.createTimer( ros::Duration( 1.0 / m_para_read_timer_frequency ), &Uart_odom::read_serive_eval_stability, this );

      m_pub_odom_test = nh.advertise< nav_msgs::Odometry >( "test_odom", 100 );
      m_pub_odom = nh.advertise< nav_msgs::Odometry >( "out_odom_uav", 100 );
      m_pub_odom_cone = nh.advertise< nav_msgs::Odometry >( "out_odom_cone", 100 );
      m_timer_test_read = nh.createTimer( ros::Duration( 1.0 / m_para_read_timer_frequency ), &Uart_odom::read_serive_eval_stability, this );

      m_serial.setPort( m_para_serial_port );
      m_serial.setBaudrate( m_para_baud_rate ); // 10000/10000 packet, all rec
      m_serial.open();

      m_protocal_to_mcu.set_protocal_mode( m_para_serial_protocal_mode );
      m_protocal_to_mcu.set_debug_info( m_para_debug_protocal );

      if ( m_serial.isOpen() )
      {
        cout << "Open serial success !!" << endl;
        m_protocal_to_mcu.init( &m_serial );
      }
      else
      {
        cout << "Open serial fail !! " << endl;
      }
    }
};

int main( int argc, char *argv[] ) {
  printf( "Hello, this is uart odom \n" );
  //    printf()
  cout << "Size of odom \r\n"
       << sizeof( nav_msgs::Odometry ) << endl;

  ros::init( argc, argv, "uart_odom" );

  ros::NodeHandle nh = ros::NodeHandle( "~" );

  Uart_odom uart_odom( nh );

  uart_odom.test_uart();
  ros::MultiThreadedSpinner spinner( 6 ); // Use 4 threads
  spinner.spin();
  return 0;
}
