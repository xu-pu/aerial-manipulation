//
// Created by sheep on 2021/6/28.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Joy.h>
#include <fstream>
#include <rnw_msgs/RnwState.h>
#include <rnw_msgs/WalkingState.h>
#include <nav_msgs/Odometry.h>
#include <rnw_msgs/ConeState.h>
#include <Eigen/Dense>
#include <uav_utils/converters.h>
#include "rnw_ros/rnw_utils.h"

using namespace std;

vector<rnw_msgs::WalkingState> v_rnw_state;

double mass = 0.5;
double xCM = 0.08;
double zCM = 0.05;

void extract_rnw_data( rosbag::Bag & bag ) {

  std::vector<std::string> topics;
  topics.emplace_back("/rnw/walking_state/session_1");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (rosbag::MessageInstance const m: view) {
    rnw_msgs::WalkingState::ConstPtr i = m.instantiate<rnw_msgs::WalkingState>();
    if (i == nullptr) continue;
    v_rnw_state.emplace_back(*i);
  }

}

void gen_csv( string const & name ){

  std::string result_dir = "/home/sheep/";
  stringstream ss; ss << result_dir << "/" << name << ".cont.csv";
  ofstream ofs(ss.str());

  ros::Time start_time = v_rnw_state.front().header.stamp;

  for (auto & i : v_rnw_state) {
    ofs << (i.header.stamp - start_time).toSec() << ","
        << i.cone_state.euler_angles.x << ","
        << i.cone_state.euler_angles.y << ","
        << i.cone_state.euler_angles.z << endl;
  }

  ofs.close();

}

void gen_ke( string const & bag_name ){

  stringstream ss; ss << "/home/sheep/" << bag_name << ".ke.csv";
  ofstream ofs(ss.str());

  ros::Time st = v_rnw_state.front().header.stamp;
  size_t step = 0;
  double peak_ke = 0;
  double ke = 0;

  for (auto & i : v_rnw_state) {
    ofs << (i.header.stamp-st).toSec() << "," << ke << endl;
    peak_ke = std::max<double>(peak_ke, calc_kinetic_energy(i.cone_state,mass,xCM,zCM));
    if ( i.step_count > step ) {
      step = i.step_count;
      ke = peak_ke;
      peak_ke = 0;
    }
  }

  ofs.close();

}

int main( int argc, char** argv ) {

  std::string bag_dir = "/home/sheep/Dropbox/mphil_bags";
  std::string bag_name = "2020-10-11-16-25-05.table.n25t40.bag";

  rosbag::Bag bag;
  stringstream ss; ss << bag_dir << "/" << bag_name;
  bag.open(ss.str());

  extract_rnw_data(bag);

  gen_csv(bag_name);

  gen_ke(bag_name);

  bag.close();

}