package quadrotor_msgs;

public interface ReplanCheck extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/ReplanCheck";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Vector3 start_velocity\ngeometry_msgs/Vector3 start_acceleration\ngeometry_msgs/Point[] plan_points\ngeometry_msgs/Point[] check_points\ngeometry_msgs/Vector3 stop_velocity\ngeometry_msgs/Vector3 stop_acceleration\nfloat64               replan_time_length\nfloat64               check_points_time_interval\nfloat64               plan_points_time_interval\nuint32                trajectory_id\nfloat64               replan_to_global_time\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getStartVelocity();
  void setStartVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getStartAcceleration();
  void setStartAcceleration(geometry_msgs.Vector3 value);
  java.util.List<geometry_msgs.Point> getPlanPoints();
  void setPlanPoints(java.util.List<geometry_msgs.Point> value);
  java.util.List<geometry_msgs.Point> getCheckPoints();
  void setCheckPoints(java.util.List<geometry_msgs.Point> value);
  geometry_msgs.Vector3 getStopVelocity();
  void setStopVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getStopAcceleration();
  void setStopAcceleration(geometry_msgs.Vector3 value);
  double getReplanTimeLength();
  void setReplanTimeLength(double value);
  double getCheckPointsTimeInterval();
  void setCheckPointsTimeInterval(double value);
  double getPlanPointsTimeInterval();
  void setPlanPointsTimeInterval(double value);
  int getTrajectoryId();
  void setTrajectoryId(int value);
  double getReplanToGlobalTime();
  void setReplanToGlobalTime(double value);
}
