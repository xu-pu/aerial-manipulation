package quadrotor_msgs;

public interface StatusData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/StatusData";
  static final java.lang.String _DEFINITION = "Header header\nuint16 loop_rate\nfloat64 voltage\nuint8 seq\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short getLoopRate();
  void setLoopRate(short value);
  double getVoltage();
  void setVoltage(double value);
  byte getSeq();
  void setSeq(byte value);
}
