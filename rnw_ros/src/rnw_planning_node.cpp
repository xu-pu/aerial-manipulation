#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"

#include <uav_utils/converters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

struct rnw_planner_t {

    static constexpr double deg2rad = M_PI/180.;

    static constexpr double min_tilt = 20 * deg2rad;

    double ang_vel_threshold = 1;

    enum class cone_fsm_e {
        idle, qstatic, rocking
    };

    cone_fsm_e fsm;

    rnw_ros::ConeState latest_cone_state;

    explicit rnw_planner_t( ros::NodeHandle & nh ){

    }

    void on_cone_state( rnw_ros::ConeStateConstPtr const & msg ){

      latest_cone_state = *msg;

      update_state(latest_cone_state);

      switch ( fsm ) {
        case cone_fsm_e::idle:
          break;
        case cone_fsm_e::rocking:
          break;
        case cone_fsm_e::qstatic:
          plan_next_position();
          break;
        default:
          ROS_ERROR_STREAM("[rnw_planner] Invalid Cone State");
          break;
      }

    }

    void plan_next_position(){
      Vector3d contact_point = uav_utils::from_point_msg(latest_cone_state.contact_point);
      Vector3d apex = uav_utils::from_point_msg(latest_cone_state.tip);

    }

    void update_state( rnw_ros::ConeState const & msg ){

      if ( msg.euler_angles.x < min_tilt ) {
        fsm = cone_fsm_e::idle;
      }
      else if ( !msg.is_point_contact ){
        fsm = cone_fsm_e::idle;
      }
      else if ( msg.euler_angles_velocity.y < ang_vel_threshold ) {
        fsm = cone_fsm_e::qstatic;
      }
      else {
        fsm = cone_fsm_e::rocking;
      }

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