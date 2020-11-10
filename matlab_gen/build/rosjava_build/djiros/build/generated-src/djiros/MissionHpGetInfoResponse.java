package djiros;

public interface MissionHpGetInfoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpGetInfoResponse";
  static final java.lang.String _DEFINITION = "MissionHotpointTask hotpoint_task\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  djiros.MissionHotpointTask getHotpointTask();
  void setHotpointTask(djiros.MissionHotpointTask value);
  byte getCmdSet();
  void setCmdSet(byte value);
  byte getCmdId();
  void setCmdId(byte value);
  int getAckData();
  void setAckData(int value);
}
