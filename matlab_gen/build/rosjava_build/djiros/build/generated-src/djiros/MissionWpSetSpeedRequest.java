package djiros;

public interface MissionWpSetSpeedRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWpSetSpeedRequest";
  static final java.lang.String _DEFINITION = "float32 speed\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  float getSpeed();
  void setSpeed(float value);
}
