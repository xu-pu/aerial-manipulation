package djiros;

public interface MissionHpUpdateYawRateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpUpdateYawRateRequest";
  static final java.lang.String _DEFINITION = "float32 yaw_rate    # degree/s\nuint8 direction     # 0: counter-clockwise, 1: clockwise\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  float getYawRate();
  void setYawRate(float value);
  byte getDirection();
  void setDirection(byte value);
}
