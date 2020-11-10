package djiros;

public interface MissionWpGetSpeedResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWpGetSpeedResponse";
  static final java.lang.String _DEFINITION = "float32 speed\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  float getSpeed();
  void setSpeed(float value);
  byte getCmdSet();
  void setCmdSet(byte value);
  byte getCmdId();
  void setCmdId(byte value);
  int getAckData();
  void setAckData(int value);
}
