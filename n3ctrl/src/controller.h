#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include "input.h"

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
	// ros::Publisher ctrl_so3_pub;
	ros::Publisher ctrl_so3_attitude_pub;
	ros::Publisher ctrl_so3_thrust_pub;
	ros::Publisher ctrl_vis_pub;
	ros::Publisher ctrl_val_dbg_pub;
	ros::Publisher ctrl_dbg_att_des_pub;
	ros::Publisher ctrl_dbg_att_real_pub;
	ros::Publisher ctrl_dbg_e_a_pub;
	ros::Publisher ctrl_dbg_e_p_pub;
	ros::Publisher ctrl_dbg_e_p_i_pub;
  ros::Publisher ctrl_dbg_e_v_pub;

	Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
	Eigen::Matrix3d Kap;
	double Kyaw;

	Eigen::Vector3d int_e_v;

	Controller(Parameter_t&);
	void config_gain(const Parameter_t::Gain& gain);
	void config();
	void update(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	
	void output_visualization(const Controller_Output_t& u);
	void publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp, const ros::Time& extra_stamp);
	void publish_so3_ctrl(const SO3_Controller_Output_t& u_so3, const ros::Time& stamp);

private:
	bool is_configured;

	// XU PU
public:

  /**
   * Original method in N3Ctrl
   * @param des
   * @param odom
   * @return F_des in world frame
   */
  Eigen::Vector3d calc_desired_force( const Desired_State_t& des,const Odom_Data_t& odom );

  /**
   * Based on mellinger thesis
   * @param des
   * @param odom
   * @return F_des in world frame
   */
  Eigen::Vector3d calc_desired_force_mellinger( const Desired_State_t& des,const Odom_Data_t& odom );

  Eigen::Vector3d regulate_desired_force( Eigen::Vector3d const & cmd );

  Eigen::Vector3d acceleration_loop( Eigen::Vector3d const & F_des, const Imu_Data_t& imu, const Odom_Data_t& odom );

};

#endif
