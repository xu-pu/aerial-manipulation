package djiros;

public interface MFIOSetValue$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MFIOSetValue$Service";
  static final java.lang.String _DEFINITION = "uint8 channel   # 0-7\nuint32 init_on_time_us    # on time for pwm duty cycle in micro-seconds\n---\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
