package djiros;

public interface MissionHotpointTask extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHotpointTask";
  static final java.lang.String _DEFINITION = "float64 latitude\nfloat64 longitude\nfloat64 altitude\nfloat64 radius\nfloat32 angular_speed\nuint8 is_clockwise\nuint8 start_point\nuint8 yaw_mode\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getLatitude();
  void setLatitude(double value);
  double getLongitude();
  void setLongitude(double value);
  double getAltitude();
  void setAltitude(double value);
  double getRadius();
  void setRadius(double value);
  float getAngularSpeed();
  void setAngularSpeed(float value);
  byte getIsClockwise();
  void setIsClockwise(byte value);
  byte getStartPoint();
  void setStartPoint(byte value);
  byte getYawMode();
  void setYawMode(byte value);
}
