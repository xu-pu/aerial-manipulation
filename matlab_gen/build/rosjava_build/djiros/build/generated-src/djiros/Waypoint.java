package djiros;

public interface Waypoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/Waypoint";
  static final java.lang.String _DEFINITION = "float64 latitude      # in degree\nfloat64 longitude     # in degree\nfloat32 altitude\nint16 heading         #heading is in [-180,180]\nuint16 staytime       # in second\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getLatitude();
  void setLatitude(double value);
  double getLongitude();
  void setLongitude(double value);
  float getAltitude();
  void setAltitude(float value);
  short getHeading();
  void setHeading(short value);
  short getStaytime();
  void setStaytime(short value);
}
