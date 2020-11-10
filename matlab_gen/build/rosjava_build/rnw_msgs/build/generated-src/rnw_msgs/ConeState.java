package rnw_msgs;

public interface ConeState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rnw_msgs/ConeState";
  static final java.lang.String _DEFINITION = "Header header\nnav_msgs/Odometry odom\n# x-psi-precession\n# y-theta-nutation (tilt)\n# z-phi-spin\n# rad\ngeometry_msgs/Vector3 euler_angles\ngeometry_msgs/Vector3 euler_angles_velocity\nbool is_point_contact\ngeometry_msgs/Point contact_point\ngeometry_msgs/Point disc_center\ngeometry_msgs/Point tip\ngeometry_msgs/Point base\nfloat64 height\nfloat64 radius";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  nav_msgs.Odometry getOdom();
  void setOdom(nav_msgs.Odometry value);
  geometry_msgs.Vector3 getEulerAngles();
  void setEulerAngles(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getEulerAnglesVelocity();
  void setEulerAnglesVelocity(geometry_msgs.Vector3 value);
  boolean getIsPointContact();
  void setIsPointContact(boolean value);
  geometry_msgs.Point getContactPoint();
  void setContactPoint(geometry_msgs.Point value);
  geometry_msgs.Point getDiscCenter();
  void setDiscCenter(geometry_msgs.Point value);
  geometry_msgs.Point getTip();
  void setTip(geometry_msgs.Point value);
  geometry_msgs.Point getBase();
  void setBase(geometry_msgs.Point value);
  double getHeight();
  void setHeight(double value);
  double getRadius();
  void setRadius(double value);
}
