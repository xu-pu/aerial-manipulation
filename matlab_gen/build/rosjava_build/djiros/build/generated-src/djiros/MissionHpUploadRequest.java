package djiros;

public interface MissionHpUploadRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpUploadRequest";
  static final java.lang.String _DEFINITION = "MissionHotpointTask hotpoint_task\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  djiros.MissionHotpointTask getHotpointTask();
  void setHotpointTask(djiros.MissionHotpointTask value);
}
