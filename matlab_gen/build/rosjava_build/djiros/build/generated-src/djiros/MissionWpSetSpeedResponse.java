package djiros;

public interface MissionWpSetSpeedResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWpSetSpeedResponse";
  static final java.lang.String _DEFINITION = "bool result";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getResult();
  void setResult(boolean value);
}
