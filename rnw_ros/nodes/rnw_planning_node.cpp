#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/rnw_planner.h"

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_cone_node");

  ros::NodeHandle nh("~");

  rnw_config_t rnw_cfg; rnw_cfg.load_from_ros(nh);

  rnw_planner_t rnw_planner(nh,rnw_cfg);

  ros::Subscriber sub_traj = nh.subscribe<rnw_msgs::ConeState>(
          "cone_state",
          100,
          &rnw_planner_t::on_cone_state,
          &rnw_planner,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_dbg_trigger = nh.subscribe<std_msgs::Header>(
          "dbg_trigger",
          100,
          &rnw_planner_t::on_debug_trigger,
          &rnw_planner,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}