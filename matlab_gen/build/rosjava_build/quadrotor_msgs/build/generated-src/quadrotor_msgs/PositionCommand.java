package quadrotor_msgs;

public interface PositionCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/PositionCommand";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Point position\ngeometry_msgs/Vector3 velocity\ngeometry_msgs/Vector3 acceleration\nfloat64 yaw\nfloat64 yaw_dot\nfloat64[3] kx\nfloat64[3] kv \n\nuint32 trajectory_id\n\nuint8 TRAJECTORY_STATUS_EMPTY = 0\nuint8 TRAJECTORY_STATUS_READY = 1\nuint8 TRAJECTORY_STATUS_COMPLETED = 3\nuint8 TRAJECTROY_STATUS_ABORT = 4\nuint8 TRAJECTORY_STATUS_ILLEGAL_START = 5\nuint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6\nuint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7\n\n# Its ID number will start from 1, allowing you comparing it with 0.\nuint8 trajectory_flag\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte TRAJECTORY_STATUS_EMPTY = 0;
  static final byte TRAJECTORY_STATUS_READY = 1;
  static final byte TRAJECTORY_STATUS_COMPLETED = 3;
  static final byte TRAJECTROY_STATUS_ABORT = 4;
  static final byte TRAJECTORY_STATUS_ILLEGAL_START = 5;
  static final byte TRAJECTORY_STATUS_ILLEGAL_FINAL = 6;
  static final byte TRAJECTORY_STATUS_IMPOSSIBLE = 7;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  geometry_msgs.Vector3 getVelocity();
  void setVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getAcceleration();
  void setAcceleration(geometry_msgs.Vector3 value);
  double getYaw();
  void setYaw(double value);
  double getYawDot();
  void setYawDot(double value);
  double[] getKx();
  void setKx(double[] value);
  double[] getKv();
  void setKv(double[] value);
  int getTrajectoryId();
  void setTrajectoryId(int value);
  byte getTrajectoryFlag();
  void setTrajectoryFlag(byte value);
}
