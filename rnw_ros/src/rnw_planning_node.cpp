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

    // planning

    double rot_dir = -1;

    double rot_amp_deg = 30;

    size_t step_count = 0;

    // rocking command

    bool plan_cmd = false;

    rnw_ros::RockingCmd latest_cmd;

    bool cmd_pending = false;

    size_t cmd_idx = 0;

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

    void plan_next_position(){

      if ( !plan_cmd ) {
        return;
      }

      if ( cmd_pending ) {
        pub_rocking_cmd.publish(latest_cmd);
        return;
      }

      rot_dir = -rot_dir;

      Vector3d contact_point = uav_utils::from_point_msg(latest_cone_state.contact_point);
      Vector3d apex = uav_utils::from_point_msg(latest_cone_state.tip);
      Vector3d v = apex - contact_point;
      Matrix3d offset = Eigen::AngleAxisd( rot_amp_deg*deg2rad*rot_dir, Vector3d::UnitZ() ).toRotationMatrix();
      Vector3d next_v = offset * v;
      Vector3d next_tip = contact_point + next_v;

      latest_cmd.header.stamp = latest_cone_state.header.stamp;
      latest_cmd.step_count = step_count;
      latest_cmd.tip_setpoint = uav_utils::to_point_msg(next_tip);
      latest_cmd.cmd_idx = cmd_idx++;
      pub_rocking_cmd.publish(latest_cmd);

      cmd_pending = true;

    }

    void update_state( rnw_ros::ConeState const & msg ){

      if ( msg.euler_angles.y < min_tilt ) {
        state_transition(fsm,cone_fsm_e::idle);
      }
      else if ( !msg.is_point_contact ){
        state_transition(fsm,cone_fsm_e::idle);
      }
      else if ( abs(msg.euler_angles_velocity.z) < ang_vel_threshold ) {
        state_transition(fsm,cone_fsm_e::qstatic);
      }
      else {
        state_transition(fsm,cone_fsm_e::rocking);
      }

    }

    void state_transition( cone_fsm_e from, cone_fsm_e to ){

      if ( from == cone_fsm_e::rocking && to == cone_fsm_e::qstatic ) {
        ROS_INFO_STREAM("[rnw] from rocking to qstatic");
        step_count++;
      }

      if ( to == cone_fsm_e::rocking && from == cone_fsm_e::qstatic ) {
        ROS_INFO_STREAM("[rnw] from qstatic to rocking");
      }

      if ( to == cone_fsm_e::idle ) {
        cmd_pending = false;
        step_count = 0;
      }

      if ( from != cone_fsm_e::idle && to == cone_fsm_e::idle ) {
        ROS_INFO_STREAM("[rnw] object became idle");
        stop_planning_cmd();
      }

      fsm = to;

    }

    void start_planning_cmd(){
      ROS_INFO_STREAM("[rnw] Start planning rocking commands");
      plan_cmd = true;
    }

    void stop_planning_cmd(){
      ROS_INFO_STREAM("[rnw] Stop planning rocking commands");
      plan_cmd = false;
    }

    void rocking_ack(){
      cmd_pending = false;
    }

    // debug

    void on_debug_trigger( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw] Got debug trigger");
      if ( !plan_cmd ) {
        start_planning_cmd();
      }
      else {
        rocking_ack();
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