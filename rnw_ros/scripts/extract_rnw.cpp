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

    if ( !latch.is_walking && i->is_walking ) {
      // start
      start_time = i->header.stamp;
    }
    else if ( latch.is_walking && !i->is_walking ) {
      // stopped
      end_time = latch.header.stamp;
      break;
    }

    v_rnw_state.emplace_back(*i);

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

int main( int argc, char** argv ) {

  std::string bag_dir = "/home/sheep/Dropbox/mphil_bags";
  std::string bag_name = "2021-06-27-23-24-28.exp8.ground.heavy.45.90.bag";
  std::string result_dir = "/home/sheep/";

  rosbag::Bag bag;
  stringstream ss; ss << bag_dir << "/" << bag_name;
  bag.open(ss.str());

  extract_rnw_data(bag);
  sync_all_data(bag);

  bag.close();

}