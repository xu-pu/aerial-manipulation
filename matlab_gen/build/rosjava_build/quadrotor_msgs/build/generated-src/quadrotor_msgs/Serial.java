package quadrotor_msgs;

public interface Serial extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Serial";
  static final java.lang.String _DEFINITION = "# Note: These constants need to be kept in sync with the types\n# defined in include/quadrotor_msgs/comm_types.h\nuint8 SO3_CMD = 115 # \'s\' in base 10\nuint8 TRPY_CMD = 112 # \'p\' in base 10\nuint8 STATUS_DATA = 99 # \'c\' in base 10\nuint8 OUTPUT_DATA = 100 # \'d\' in base 10\nuint8 PPR_OUTPUT_DATA = 116 # \'t\' in base 10\nuint8 PPR_GAINS = 103 # \'g\'\n\nHeader header\nuint8 channel\nuint8 type # One of the types listed above\nuint8[] data\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte SO3_CMD = 115;
  static final byte TRPY_CMD = 112;
  static final byte STATUS_DATA = 99;
  static final byte OUTPUT_DATA = 100;
  static final byte PPR_OUTPUT_DATA = 116;
  static final byte PPR_GAINS = 103;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getChannel();
  void setChannel(byte value);
  byte getType();
  void setType(byte value);
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}
