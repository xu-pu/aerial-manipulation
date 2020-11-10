package quadrotor_msgs;

public interface Replan extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Replan";
  static final java.lang.String _DEFINITION = "#data structure\ngeometry_msgs/Vector3 start_velocity\ngeometry_msgs/Vector3 start_acceleration\nnav_msgs/Path         plan\ngeometry_msgs/Vector3 stop_velocity\ngeometry_msgs/Vector3 stop_acceleration\nfloat64               replan_time\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  geometry_msgs.Vector3 getStartVelocity();
  void setStartVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getStartAcceleration();
  void setStartAcceleration(geometry_msgs.Vector3 value);
  nav_msgs.Path getPlan();
  void setPlan(nav_msgs.Path value);
  geometry_msgs.Vector3 getStopVelocity();
  void setStopVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getStopAcceleration();
  void setStopAcceleration(geometry_msgs.Vector3 value);
  double getReplanTime();
  void setReplanTime(double value);
}
