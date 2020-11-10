package djiros;

public interface MissionHpUpdateRadiusRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpUpdateRadiusRequest";
  static final java.lang.String _DEFINITION = "float32 radius      # in meters\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  float getRadius();
  void setRadius(float value);
}
