package quadrotor_msgs;

public interface TrajectoryMatrix extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/TrajectoryMatrix";
  static final java.lang.String _DEFINITION = "#type\nuint8 TYPE_UNKNOWN = 0\nuint8 TYPE_POLY    = 1\nuint8 TYPE_TIME    = 2\n\n#data structure\nHeader    header\nuint8     type\nuint32    id\nstring    name\nuint32[]  format\nfloat64[] data\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte TYPE_UNKNOWN = 0;
  static final byte TYPE_POLY = 1;
  static final byte TYPE_TIME = 2;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getType();
  void setType(byte value);
  int getId();
  void setId(int value);
  java.lang.String getName();
  void setName(java.lang.String value);
  int[] getFormat();
  void setFormat(int[] value);
  double[] getData();
  void setData(double[] value);
}
