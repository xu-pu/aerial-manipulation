#include "rnw_ros/poly_traj.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>

using namespace std;

using Eigen::Vector3d;

struct data_logger_t {

    quadrotor_msgs::PolynomialTrajectory latest_msg;

    poly_traj_t poly_traj;

    std::ofstream ofs;

    explicit data_logger_t( ros::NodeHandle & nh ) : ofs("/home/sheep/reloc_log.txt") {}

    void on_traj(  quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg  ){
      latest_msg = *msg;
      poly_traj = poly_traj_t(latest_msg);
      cout << poly_traj.traj_id << " " << poly_traj.duration() << endl;
      ofs << poly_traj.duration() << endl;
      ofs.flush();
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"gen_paper_data_node");

  ros::NodeHandle nh("~");

  data_logger_t data_logger(nh);

  ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>("/rnw/poly_traj", 100, &data_logger_t::on_traj, &data_logger );

  ros::spin();

  ros::shutdown();

  return 0;

}