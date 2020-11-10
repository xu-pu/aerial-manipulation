package quadrotor_msgs;

public interface Odometry extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Odometry";
  static final java.lang.String _DEFINITION = "uint8 STATUS_ODOM_VALID=0\nuint8 STATUS_ODOM_INVALID=1\nuint8 STATUS_ODOM_LOOPCLOSURE=2\n\nnav_msgs/Odometry curodom\nnav_msgs/Odometry kfodom\nuint32 kfid\nuint8 status\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte STATUS_ODOM_VALID = 0;
  static final byte STATUS_ODOM_INVALID = 1;
  static final byte STATUS_ODOM_LOOPCLOSURE = 2;
  nav_msgs.Odometry getCurodom();
  void setCurodom(nav_msgs.Odometry value);
  nav_msgs.Odometry getKfodom();
  void setKfodom(nav_msgs.Odometry value);
  int getKfid();
  void setKfid(int value);
  byte getStatus();
  void setStatus(byte value);
}
