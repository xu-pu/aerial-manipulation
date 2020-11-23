#include "rnw_ros/poly_traj.h"
#include "rnw_ros/rnw_planner.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <rnw_msgs/RockingCmd.h>

using namespace std;

using Eigen::Vector3d;

struct data_logger_t {

    quadrotor_msgs::PolynomialTrajectory latest_msg;

    poly_traj_t poly_traj;

    std::ofstream ofs;

    std::ofstream ofs_amp;

    explicit data_logger_t( ros::NodeHandle & nh ) : ofs("/home/sheep/reloc_log.txt"), ofs_amp("/home/sheep/amp_log.txt") {}

    void on_traj(  quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg  ){
      latest_msg = *msg;
      poly_traj = poly_traj_t(latest_msg);
      cout << poly_traj.traj_id << " " << poly_traj.duration() << endl;
      ofs << poly_traj.duration() << endl;
      ofs.flush();
    }

    void log_reloc_length(  quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg  ){
      latest_msg = *msg;
      poly_traj = poly_traj_t(latest_msg);
      Vector3d start_pt = poly_traj.eval_pos(poly_traj.start_time);
      Vector3d end_pt = poly_traj.eval_pos(poly_traj.final_time);
      double dist = (start_pt-end_pt).norm();
      cout << poly_traj.traj_id << " " << dist << endl;
      ofs << dist << endl;
      ofs.flush();
    }

    rnw_msgs::RockingCmd pre_cmd;
    bool cmd_init = false;

    void on_cmd( rnw_msgs::RockingCmdConstPtr const & cmd ){
      if (!cmd_init) {
        pre_cmd = *cmd;
        cmd_init = true;
        return;
      }

      rnw_msgs::RockingCmd const & cur_cmd = *cmd;

      if ( cur_cmd.cmd_idx > pre_cmd.cmd_idx ) {
        // new command arrived
        cout << cur_cmd.step_count << ", " << cur_cmd.grip_state.cone_state.euler_angles.z << endl;
        ofs_amp << cur_cmd.grip_state.cone_state.euler_angles.z << endl;
      }

      pre_cmd = *cmd;

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"gen_paper_data_node");

  ros::NodeHandle nh("~");

  data_logger_t data_logger(nh);

  //ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>("/rnw/poly_traj", 100, &data_logger_t::on_traj, &data_logger );

  ros::Subscriber sub_cmd = nh.subscribe<rnw_msgs::RockingCmd>("/rnw/rocking_cmd", 10000, &data_logger_t::on_cmd, &data_logger );

  ros::spin();

  ros::shutdown();

  return 0;

}