package djiros;

public interface Gimbal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/Gimbal";
  static final java.lang.String _DEFINITION = "Header header\nint32 ts        # sec\n# Mode is 1 byte size:\n# Bit #:      | Set to 0:                             | Set to 1:\n# ----------- | ------------------------------------- | -------------------------------------\n# bit 0       | Incremental control,                  | Absolute control,\n#             | the angle reference is the            | the angle reference is\n#             | current Gimbal location               | related to configuration in DJI Go App\n# bit 1       | Gimbal will follow the command in Yaw | Gimbal will maintain position in Yaw\n# bit 2       | Roll invalid bit, the same as bit[1]  | Roll invalid bit, the same as bit[1]\n# bit 3       |Pitch invalid bit, the same as bit[1]  | Pitch invalid bit, the same as bit[1]\n# bit [4:7]   | bit [4:7]: reserved, set to be 0      |\nuint8 mode\nfloat32 pitch   # rads\nfloat32 yaw     # rads\nfloat32 roll    # rads\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getTs();
  void setTs(int value);
  byte getMode();
  void setMode(byte value);
  float getPitch();
  void setPitch(float value);
  float getYaw();
  void setYaw(float value);
  float getRoll();
  void setRoll(float value);
}
