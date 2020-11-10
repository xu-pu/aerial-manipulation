package quadrotor_msgs;

public interface SwarmInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/SwarmInfo";
  static final java.lang.String _DEFINITION = "quadrotor_msgs/TrajectoryMatrix path\ntime start\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  quadrotor_msgs.TrajectoryMatrix getPath();
  void setPath(quadrotor_msgs.TrajectoryMatrix value);
  org.ros.message.Time getStart();
  void setStart(org.ros.message.Time value);
}
