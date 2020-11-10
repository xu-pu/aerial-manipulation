package djiros;

public interface MFIOSetValueRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MFIOSetValueRequest";
  static final java.lang.String _DEFINITION = "uint8 channel   # 0-7\nuint32 init_on_time_us    # on time for pwm duty cycle in micro-seconds\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getChannel();
  void setChannel(byte value);
  int getInitOnTimeUs();
  void setInitOnTimeUs(int value);
}
