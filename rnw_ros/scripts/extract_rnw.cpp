//
// Created by sheep on 2021/6/28.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Joy.h>
#include <fstream>
#include <rnw_msgs/RnwState.h>
#include <nav_msgs/Odometry.h>
#include <rnw_msgs/ConeState.h>
#include <Eigen/Dense>
#include <uav_utils/converters.h>

using namespace std;

ros::Time start_time;
ros::Time end_time;

vector<rnw_msgs::RnwState> v_rnw_state;
vector<rnw_msgs::ConeState> v_cone_state;
vector<nav_msgs::Odometry> v_drone1_odom;
vector<nav_msgs::Odometry> v_drone2_odom;

void extract_rnw_data( rosbag::Bag & bag ) {

  std::vector<std::string> topics;
  topics.emplace_back("/rnw/state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  rnw_msgs::RnwState latch;
  latch.is_walking = false;

  for (rosbag::MessageInstance const m: view) {
    rnw_msgs::RnwState::ConstPtr i = m.instantiate<rnw_msgs::RnwState>();
    if (i == nullptr) continue;

    if ( i->is_walking ) {
      v_rnw_state.emplace_back(*i);
    }

    if ( !latch.is_walking && i->is_walking ) {
      // start
      start_time = i->header.stamp;
    }
    else if ( latch.is_walking && !i->is_walking ) {
      // stopped
      end_time = latch.header.stamp;
      break;
    }

    latch = *i;

  }

  cout << "rnw duration: " << (end_time-start_time).toSec() << ", count: " << v_rnw_state.size() << endl;

}

template<typename T>
void sync_topic( rosbag::Bag & bag, string const & name, std::vector<T> & dst ){

  dst.clear();
  dst.resize(v_rnw_state.size());

  std::vector<std::string> topics;
  topics.emplace_back(name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  size_t idx = 0;

  for (rosbag::MessageInstance const m: view) {
    auto ptr = m.instantiate<T>();
    if ( ptr == nullptr ) continue;
    if ( ptr->header.stamp >= v_rnw_state.at(idx).header.stamp ) {
      dst.at(idx) = *ptr;
      idx++;
    }
    if ( idx >= dst.size() ) {
      break;
    }
  }

  cout << "topic" << name << ", " << idx << " synced" << endl;

}

void sync_all_data( rosbag::Bag & bag ){
  sync_topic<rnw_msgs::ConeState>(bag,"/cone/state",v_cone_state);
  sync_topic<nav_msgs::Odometry>(bag,"/drone1/odom",v_drone1_odom);
  sync_topic<nav_msgs::Odometry>(bag,"/drone2/odom",v_drone2_odom);
}

double calc_angle( rnw_msgs::ConeState const & cone, nav_msgs::Odometry const & odom1, nav_msgs::Odometry const & odom2 ){

  Eigen::Vector3d cp = uav_utils::from_point_msg(cone.tip);
  Eigen::Vector3d d1 = uav_utils::from_point_msg(odom1.pose.pose.position);
  Eigen::Vector3d d2 = uav_utils::from_point_msg(odom2.pose.pose.position);

  Eigen::Vector3d v1 = d1-cp;
  Eigen::Vector3d v2 = d2-cp;

  return std::atan2(v1.cross(v2).norm(), v1.dot(v2));

}


void gen_csv( string const & name ){

  std::string result_dir = "/home/sheep/";
  stringstream ss; ss << result_dir << "/" << name << ".cont.csv";
  ofstream ofs(ss.str());

  for ( size_t i=0; i<v_rnw_state.size(); i++ ) {
    ofs << (v_rnw_state.at(i).header.stamp - start_time).toSec() << ","
        << v_cone_state.at(i).euler_angles.y << ","
        << v_cone_state.at(i).euler_angles.z << ","
        << calc_angle(v_cone_state.at(i),v_drone1_odom.at(i),v_drone2_odom.at(i)) << endl;
  }

  ofs.close();

}


int main( int argc, char** argv ) {

  std::string bag_dir = "/home/sheep/Dropbox/mphil_bags";
  std::string bag_name = "2021-06-27-23-24-28.exp8.ground.heavy.45.90.bag";

  rosbag::Bag bag;
  stringstream ss; ss << bag_dir << "/" << bag_name;
  bag.open(ss.str());

  extract_rnw_data(bag);
  sync_all_data(bag);

  gen_csv(bag_name);

  bag.close();

}