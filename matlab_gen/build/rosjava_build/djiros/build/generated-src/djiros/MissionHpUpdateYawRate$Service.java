package djiros;

public interface MissionHpUpdateYawRate$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpUpdateYawRate$Service";
  static final java.lang.String _DEFINITION = "float32 yaw_rate    # degree/s\nuint8 direction     # 0: counter-clockwise, 1: clockwise\n---\nbool result\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
