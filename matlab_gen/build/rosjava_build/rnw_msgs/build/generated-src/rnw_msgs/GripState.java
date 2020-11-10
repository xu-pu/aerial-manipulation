package rnw_msgs;

public interface GripState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rnw_msgs/GripState";
  static final java.lang.String _DEFINITION = "Header header\nrnw_msgs/ConeState cone_state\nnav_msgs/Odometry uav_odom\ngeometry_msgs/Point flu_T_tcp\ngeometry_msgs/Point grip_point\nfloat64 grip_radius\nfloat64 grip_depth\nbool grip_valid";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  rnw_msgs.ConeState getConeState();
  void setConeState(rnw_msgs.ConeState value);
  nav_msgs.Odometry getUavOdom();
  void setUavOdom(nav_msgs.Odometry value);
  geometry_msgs.Point getFluTTcp();
  void setFluTTcp(geometry_msgs.Point value);
  geometry_msgs.Point getGripPoint();
  void setGripPoint(geometry_msgs.Point value);
  double getGripRadius();
  void setGripRadius(double value);
  double getGripDepth();
  void setGripDepth(double value);
  boolean getGripValid();
  void setGripValid(boolean value);
}
