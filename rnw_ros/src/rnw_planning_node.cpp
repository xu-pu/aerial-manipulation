#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"

#include <uav_utils/converters.h>

using namespace std;

struct rnw_planner_t {

    static constexpr double deg2rad = M_PI/180.;

    static constexpr double min_tilt = 10 * deg2rad;

    double ang_vel_threshold = 0.5;

    enum class cone_fsm_e {
        idle, qstatic, rocking
    };

    ros::Publisher pub_rocking_cmd;

    cone_fsm_e fsm;

    rnw_ros::ConeState latest_cone_state;

    explicit rnw_planner_t( ros::NodeHandle & nh ){
      pub_rocking_cmd = nh.advertise<rnw_ros::RockingCmd>("rocking_cmd",10);
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

    double rot_dir = -1;
    double rot_amp_deg = 30;

    size_t step_count = 0;

    void plan_next_position(){
      Vector3d contact_point = uav_utils::from_point_msg(latest_cone_state.contact_point);
      Vector3d apex = uav_utils::from_point_msg(latest_cone_state.tip);
      Vector3d v = apex - contact_point;
      Matrix3d offset = Eigen::AngleAxisd( rot_amp_deg*deg2rad*rot_dir, Vector3d::UnitZ() ).toRotationMatrix();
      Vector3d next_v = offset * v;
      Vector3d next_tip = contact_point + next_v;

      rnw_ros::RockingCmd msg;
      msg.header.stamp = latest_cone_state.header.stamp;
      msg.step_count = step_count;
      msg.tip_setpoint = uav_utils::to_point_msg(next_tip);
      pub_rocking_cmd.publish(msg);

    }

    void update_state( rnw_ros::ConeState const & msg ){

      if ( msg.euler_angles.y < min_tilt ) {
        state_transition(fsm,cone_fsm_e::idle);
      }
      else if ( !msg.is_point_contact ){
        state_transition(fsm,cone_fsm_e::idle);
      }
      else if ( abs(msg.euler_angles_velocity.y) < ang_vel_threshold ) {
        state_transition(fsm,cone_fsm_e::qstatic);
      }
      else {
        state_transition(fsm,cone_fsm_e::rocking);
      }

    }

    void state_transition( cone_fsm_e from, cone_fsm_e to ){

      if ( from == cone_fsm_e::rocking && to == cone_fsm_e::qstatic ) {
        ROS_INFO_STREAM("[rnw] from rocking to qstatic");
        rot_dir = -rot_dir; // switch rocking direction
        step_count++;
      }

      if ( to == cone_fsm_e::rocking && from == cone_fsm_e::qstatic ) {
        ROS_INFO_STREAM("[rnw] from qstatic to rocking");
      }

      if ( to == cone_fsm_e::idle ) {
        step_count = 0;
      }

      if ( from != cone_fsm_e::idle && to == cone_fsm_e::idle ) {
        ROS_INFO_STREAM("[rnw] object became idle");
      }

      fsm = to;

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