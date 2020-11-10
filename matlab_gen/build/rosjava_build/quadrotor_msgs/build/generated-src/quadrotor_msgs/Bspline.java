package quadrotor_msgs;

public interface Bspline extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Bspline";
  static final java.lang.String _DEFINITION = "# the action command for trajectory server.\nuint32 ACTION_ADD                  =   1\nuint32 ACTION_ABORT                =   2\nuint32 ACTION_WARN_START           =   3\nuint32 ACTION_WARN_FINAL           =   4\nuint32 ACTION_WARN_IMPOSSIBLE      =   5\nuint32 action\n\nint32 order\nint64 traj_id\nfloat64[] knots\ngeometry_msgs/Point[] pts\ntime start_time\nfloat64 time_extra\nfloat64 replan_to_global_time";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final int ACTION_ADD = 1;
  static final int ACTION_ABORT = 2;
  static final int ACTION_WARN_START = 3;
  static final int ACTION_WARN_FINAL = 4;
  static final int ACTION_WARN_IMPOSSIBLE = 5;
  int getAction();
  void setAction(int value);
  int getOrder();
  void setOrder(int value);
  long getTrajId();
  void setTrajId(long value);
  double[] getKnots();
  void setKnots(double[] value);
  java.util.List<geometry_msgs.Point> getPts();
  void setPts(java.util.List<geometry_msgs.Point> value);
  org.ros.message.Time getStartTime();
  void setStartTime(org.ros.message.Time value);
  double getTimeExtra();
  void setTimeExtra(double value);
  double getReplanToGlobalTime();
  void setReplanToGlobalTime(double value);
}
