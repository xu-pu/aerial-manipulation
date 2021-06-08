#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <n3ctrl/ControllerDebug.h>
#include <std_msgs/UInt8.h>
#include "input.h"

struct vector3d_lpf_t {

    Eigen::Vector3d value;

    double T = 1;

    bool initialized = false;

    inline Eigen::Vector3d filter( Eigen::Vector3d const & value_new ){
      if ( !initialized ) {
        initialized = true;
        value = value_new;
      }
      else {
        for ( int i=0; i<3; i++ ) {
          value(i) += (value_new(i)-value(i))/(1+(1/(2.*M_PI*T)));
        }
      }
      return value;
    }

};

struct error_integral_t {

    Parameter_t const & param;

    inline explicit error_integral_t( Parameter_t const & p ) : param(p) {
      reset();
    }

    inline void reset(){
      error_integral.setZero();
      disturbance.setZero();
    }

    inline error_integral_t & update( Eigen::Vector3d const & err ){
      using uav_utils::clamp;
      for ( int i = 0; i < 3; i++ ) {
        error_integral(i) += clamp(err(i),param.disturbance.error_limit(i)) * param.disturbance.integration_ratio;
      }
      return *this;
    }

    /**
     * @param gain - control gains, note that gains are defined in the intermediate frame
     * @param gain_frame - gRe - transformation from error frame to gain frame
     * @return disturbance compensation, unit is acceleration
     */
    inline Eigen::Vector3d output( Eigen::Matrix3d const & gain, Eigen::Matrix3d const & gain_frame ) {

      Eigen::Matrix3d const & gRe = gain_frame; // error frame to gain frame

      Eigen::Vector3d rst = gRe.transpose() * gain * gRe * error_integral;

      Eigen::Vector3d limit_acc = param.disturbance.limit / param.mass;

      for ( int i=0; i<3; i++ ) {
        if (std::fabs(rst(i)) > limit_acc(i)) {
          uav_utils::limit_range(rst(i), limit_acc(i));
          ROS_WARN("Disturbance saturate for axis %u, value=%.3f", i, rst(i));
        }
      }

      disturbance = - param.mass * rst ;

      return rst;

    }

    Eigen::Vector3d error_integral;

    Eigen::Vector3d disturbance;

};

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	double yaw;
	Eigen::Quaterniond q;
	Eigen::Vector3d a;
};

struct Controller_Output_t
{
	static constexpr double VERT_VELO = -1.0;
	static constexpr double VERT_THRU = 1.0;

	// static constexpr double CTRL_YAW_RATE = -1.0; //zxzxzxzx
 //    static constexpr double CTRL_YAW = 1.0;
    static constexpr double CTRL_YAW_RATE = 1.0;
    static constexpr double CTRL_YAW = 0.0;

	double roll;
	double pitch;
	double yaw;
	double thrust;
	double mode; // if mode > 0, thrust = 0~100%;
				// if mode < 0, thrust = -? m/s ~ +? m/s
	double yaw_mode; // if yaw_mode > 0, CTRL_YAW;
				// if yaw_mode < 0, CTRL_YAW_RATE
};

struct SO3_Controller_Output_t
{
	Eigen::Matrix3d Rdes;
	Eigen::Vector3d Fdes;
	double net_force;
};

class Controller
{
public:
	Parameter_t& param;

	ros::Publisher ctrl_pub;
	ros::Publisher pub_dbg_info;
	ros::Publisher pub_disturbance;

	n3ctrl::ControllerDebug dbg_msg;

	Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
	Eigen::Matrix3d Kap;
	double Kyaw;

	error_integral_t vel_err_integral;

	ros::Time controller_last_active;

	// low-pass-filters
	vector3d_lpf_t lpf_acc; // acceleration in world frame
	vector3d_lpf_t lpf_thrust; // specific thrust in world frame

    enum class flight_status_e {
        STOPED      = 0,
        ON_GROUND   = 1,
        IN_AIR      = 2
    } flight_status = flight_status_e::STOPED;

    Controller(Parameter_t&);
	void config_gain(const Parameter_t::Gain& gain);
	void config();
	void update(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	
	void publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp, const ros::Time& extra_stamp);

private:
	bool is_configured;

	// XU PU
public:

    /**
     * Limit attitude angles and magnitude of the trust vector
     * @param cmd - thrust vector
     * @return regulated trust vector
     */
    Eigen::Vector3d regulate_trust_vector(Eigen::Vector3d const & cmd );

    /**
     * Incremental Nonlinear Dynamic Inversion (INDI) Acceleration Controller
     * @param cmd_acc
     * @return specific thrust vector (F/m)
     */
    Eigen::Vector3d acceleration_loop( Eigen::Vector3d const & cmd_acc );

    Eigen::Vector3d command_acceleration_n3ctrl(const Desired_State_t& des, const Odom_Data_t& odom);

    Eigen::Vector3d command_acceleration_sertac(const Desired_State_t& des, const Odom_Data_t& odom);

    void on_flight_status( std_msgs::UInt8ConstPtr const & msg );

    /**
     * Enter disarm stage when reached certain height
     * @param odom
     * @return
     */
    bool can_disarm( const Odom_Data_t& odom ) const;

    /**
     * Disarm when setpoint z is negative
     * @param des
     * @return
     */
    bool is_disarm_cmd( const Desired_State_t& des ) const;

    /**
     * Decide disarm or not
     * @param des
     * @param odom
     * @return
     */
    bool should_disarm(const Desired_State_t &des, const Odom_Data_t &odom ) const;

    Eigen::Vector3d external_force_estimate();

    Eigen::Vector3d f_ext_indi();

    static void disarm();

public:

    ros::Time arm_time;

    double sec_since_motor_armed() const;

};

#endif
