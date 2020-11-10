package djiros;

public interface CameraActionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/CameraActionRequest";
  static final java.lang.String _DEFINITION = "#constant for camera_action\nuint8 CAMERA_ACTION_TAKE_PICTURE = 0\nuint8 CAMERA_ACTION_START_RECORD = 1\nuint8 CAMERA_ACTION_STOP_RECORD  = 2\n\n#request\nuint8 camera_action     # see constants above for possible camera_actions\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte CAMERA_ACTION_TAKE_PICTURE = 0;
  static final byte CAMERA_ACTION_START_RECORD = 1;
  static final byte CAMERA_ACTION_STOP_RECORD = 2;
  byte getCameraAction();
  void setCameraAction(byte value);
}
