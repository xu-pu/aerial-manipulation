package quadrotor_msgs;

public interface SwarmOdometry extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/SwarmOdometry";
  static final java.lang.String _DEFINITION = "nav_msgs/Odometry odom\nstring name\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  nav_msgs.Odometry getOdom();
  void setOdom(nav_msgs.Odometry value);
  java.lang.String getName();
  void setName(java.lang.String value);
}
