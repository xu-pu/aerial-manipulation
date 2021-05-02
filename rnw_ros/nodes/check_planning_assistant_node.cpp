#include "rnw_ros/rnw_utils.h"
#include <quadrotor_msgs/PositionCommand.h>

quadrotor_msgs::PositionCommand pt2cmd( Vector3d const & pt ){
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.position = uav_utils::to_point_msg(pt);
  cmd.yaw_dot = 0;
  cmd.yaw = 0;
}

Vector3d calc_desired_drone_pos( Vector3d const & CP, double heading, double cable, double rad ){
  // reference frame at the control point
  Matrix3d R = Eigen::AngleAxisd(heading,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d pt(0,cable*std::sin(rad),cable*cos(rad));
  return R*pt+CP;
}

struct assistant_node_t {

    rnw_config_t rnw_config;

    rnw_msgs::ConeState latest_cone_state;

    ros::Subscriber sub_cone_state;

    ros::Subscriber sub_start_trigger;

    ros::Publisher pub_pos_cmd_drone1;

    ros::Publisher pub_pos_cmd_drone2;

    ros::Publisher pub_start_planning;

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      latest_cone_state = *msg;
    }

    void on_start( std_msgs::HeaderConstPtr const & msg ){

      if ( (ros::Time::now() - latest_cone_state.header.stamp).toSec() > 1 ) {
        ROS_ERROR_STREAM("cone state timeout!");
        return;
      }

      double rad = rnw_config.swarm.angle * deg2rad * 0.5;
      double heading = cone_yaw(latest_cone_state);
      Vector3d CP = uav_utils::from_point_msg(latest_cone_state.tip);
      Vector3d drone1_pos = calc_desired_drone_pos(CP,heading,rnw_config.swarm.cable1,rad);
      Vector3d drone2_pos = calc_desired_drone_pos(CP,heading,rnw_config.swarm.cable2,rad);

      pub_pos_cmd_drone1.publish(pt2cmd(drone1_pos));
      pub_pos_cmd_drone2.publish(pt2cmd(drone2_pos));

      usleep(1e5);

      pub_pos_cmd_drone1.publish(pt2cmd(drone1_pos));
      pub_pos_cmd_drone2.publish(pt2cmd(drone2_pos));

      usleep(1e5);

      pub_start_planning.publish(latest_cone_state.header);

    }

    assistant_node_t(){

      ros::NodeHandle nh("~");

      rnw_config.load_from_ros(nh);

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              100,
              &assistant_node_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_start_trigger = nh.subscribe<std_msgs::Header>(
              "start",
              100,
              &assistant_node_t::on_start,
              this
      );

      pub_pos_cmd_drone1 = nh.advertise<quadrotor_msgs::PositionCommand>("/drone1/position_cmd",100);
      pub_pos_cmd_drone2 = nh.advertise<quadrotor_msgs::PositionCommand>("/drone2/position_cmd",100);

      pub_start_planning = nh.advertise<std_msgs::Header>("/rnw/start",100);

    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"check_planning_assistant_node");

  assistant_node_t assistant_node;

  ros::spin();

  ros::shutdown();

  return 0;

}