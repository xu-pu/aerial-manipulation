#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"

Eigen::Matrix3d calc_rnw_body_frame( rnw_msgs::ConeState const & cone_state ){
  return (Eigen::AngleAxisd( cone_state.euler_angles.x, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd( cone_state.euler_angles.y, Eigen::Vector3d::UnitY())).toRotationMatrix();
}

struct my_node_t {

    ros::Subscriber sub_cone_state;

    ros::Publisher pub_body_frame;

    my_node_t(){

      ros::NodeHandle nh("~");

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              100,
              &my_node_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      pub_body_frame = nh.advertise<nav_msgs::Odometry>("/cone/body_frame",100);

    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ) const {
      auto R = calc_rnw_body_frame(*msg);
      Eigen::Quaterniond quat(R);
      nav_msgs::Odometry rst;
      rst.header = msg->header;
      rst.pose.pose.position = msg->disc_center;
      rst.pose.pose.orientation = uav_utils::to_quaternion_msg(quat);
      pub_body_frame.publish(rst);
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_rnw_body_frame_node");

  my_node_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}