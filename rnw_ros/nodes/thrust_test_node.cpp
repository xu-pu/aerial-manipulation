#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <dynamic_reconfigure/server.h>
#include <rnw_ros/ThrustTestConfig.h>
#include <djiros/DroneArmControl.h>

struct thrust_test_node_t {

    djiros::DroneArmControl arm;

    enum class flight_status_e {
        STOPED      = 0,
        ON_GROUND   = 1,
        IN_AIR      = 2
    } flight_status = flight_status_e::STOPED;

    ros::Publisher pub_ctrl;

    ros::Subscriber sub_flight_status;

    ros::Timer timer;

    double thrust_percent = 0;

    double arm_threshold = 0.1;

    thrust_test_node_t(){

      ros::NodeHandle nh("~");

      pub_ctrl = nh.advertise<sensor_msgs::Joy>("ctrl",100);

      sub_flight_status = nh.subscribe<std_msgs::UInt8>(
              "flight_status",
              1,
              &thrust_test_node_t::on_flight_status,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      timer = nh.createTimer(ros::Rate(100),&thrust_test_node_t::spin,this);

    }

    void spin( ros::TimerEvent const & e ){

      if ( thrust_percent > arm_threshold && flight_status == flight_status_e::STOPED ) {
        // need to arm
        arm.request.arm = 1;
        ros::service::call("/djiros/drone_arm_control",arm);
      }
      else if ( thrust_percent < arm_threshold && flight_status != flight_status_e::STOPED ) {
        arm.request.arm = 0;
        ros::service::call("/djiros/drone_arm_control",arm);
      }

      pub_ctrl.publish(make_thrust_cmd(thrust_percent));

    }

    static sensor_msgs::Joy make_thrust_cmd( double thr_percent ){

      sensor_msgs::Joy msg;

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = std::string("FRD");

      // need to translate to forward-right-down frame
      msg.axes.push_back(0);
      msg.axes.push_back(0);
      msg.axes.push_back(100.f*thr_percent);
      msg.axes.push_back(0); // yaw rate
      msg.axes.push_back(1); // thrust mode
      msg.axes.push_back(1); // yaw mode

      return msg;

    }

    void on_flight_status( std_msgs::UInt8ConstPtr const & msg ){
      flight_status = (flight_status_e)msg->data;
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"thrust_test_node");

  thrust_test_node_t node;

  dynamic_reconfigure::Server<rnw_ros::ThrustTestConfig> server;
  server.setCallback([&]( rnw_ros::ThrustTestConfig & config, uint32_t level ){
      node.thrust_percent = config.thrust_percentage / 100.;
      ROS_WARN("[thrust_test] set thrust to %d%%",(int)config.thrust_percentage);
  });

  ros::spin();

  ros::shutdown();

  return 0;

}