#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"

struct zero_spin_finder_t {

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      update(msg);
    }

    bool found() const {
      return _found;
    }

    rnw_msgs::ConeState take(){
      _found = false;
      return _rst;
    }

private:

    bool finding = false;

    rnw_msgs::ConeState zero_state;

    static constexpr double spin_zero_thresh_deg = 5;

    bool _found = false;

    rnw_msgs::ConeState _rst;

    void update( rnw_msgs::ConeStateConstPtr const & msg ){

      double spin_zero_thresh = spin_zero_thresh_deg * deg2rad;

      if ( finding ) {
        double cur = abs(msg->euler_angles.z);
        double last = abs(zero_state.euler_angles.z);
        if ( !msg->is_point_contact || cur > spin_zero_thresh ) {
          _found = true;
          _rst = zero_state;
          finding = false;
        }
        else if ( cur < last ) {
          zero_state = *msg;
        }
      }
      else if ( abs(msg->euler_angles.z) < spin_zero_thresh ) {
        finding = true;
        zero_state = *msg;
      }
    }

};


struct obj_tracer_t {

    ros::Publisher pub_spin_zero;

    ros::Publisher pub_spin_zero_odom;

    ros::Subscriber sub_cone_state;

    zero_spin_finder_t finder;

    explicit obj_tracer_t( ros::NodeHandle & nh ){

      pub_spin_zero = nh.advertise<rnw_msgs::ConeState>("/cone/zero_spin_states",100);

      pub_spin_zero_odom = nh.advertise<nav_msgs::Odometry>("/cone/zero_spin_odom",100);

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              10,
              &obj_tracer_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      finder.on_cone_state(msg);
      if ( finder.found() ) {
        rnw_msgs::ConeState rst = finder.take();
        pub_spin_zero.publish(rst);
        pub_spin_zero_odom.publish(rst.odom);
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_obj_trace_node");

  ros::NodeHandle nh("~");

  obj_tracer_t node(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}