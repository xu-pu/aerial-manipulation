#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"

#include <uav_utils/converters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

struct rnw_planner_t {

    explicit rnw_planner_t( ros::NodeHandle & nh ){}

    void on_cone_state( rnw_ros::ConeStateConstPtr const & msg ){

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_cone_node");

  ros::NodeHandle nh("~");

  rnw_planner_t rnw_planner(nh);

  ros::Subscriber sub_traj = nh.subscribe<rnw_ros::ConeState>(
          "cone_state",
          100,
          &rnw_planner_t::on_cone_state,
          &rnw_planner,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}