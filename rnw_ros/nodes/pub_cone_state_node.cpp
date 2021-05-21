#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/cone_state_estimator.h"
#include "rnw_ros/TuningConeStateConfig.h"

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  cone_state_estimator_t cone_state_estimator(nh);

  ////////////////////// dynamic_reconfigure param tuning
//  using dyn_cfg_server_t = dynamic_reconfigure::Server<rnw_ros::TuningConeStateConfig>;
//  std::shared_ptr<dyn_cfg_server_t> server;
//  server = std::make_shared<dyn_cfg_server_t>();
//  server->setCallback([&](rnw_ros::TuningConeStateConfig & config, uint32_t level){
//      cone_state_estimator.lpf_ang_vel_x.T = config.lpf_ang_vel_x;
//      cone_state_estimator.lpf_ang_vel_y.T = config.lpf_ang_vel_y;
//      cone_state_estimator.lpf_ang_vel_z.T = config.lpf_ang_vel_z;
//  });
  /////////////////////

  ros::spin();

  ros::shutdown();

  return 0;

}