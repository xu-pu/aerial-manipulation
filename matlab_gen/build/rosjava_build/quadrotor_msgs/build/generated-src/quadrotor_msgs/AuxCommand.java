package quadrotor_msgs;

public interface AuxCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/AuxCommand";
  static final java.lang.String _DEFINITION = "float64 current_yaw\nfloat64 kf_correction\nfloat64[2] angle_corrections# Trims for roll, pitch\nbool enable_motors\nbool use_external_yaw\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getCurrentYaw();
  void setCurrentYaw(double value);
  double getKfCorrection();
  void setKfCorrection(double value);
  double[] getAngleCorrections();
  void setAngleCorrections(double[] value);
  boolean getEnableMotors();
  void setEnableMotors(boolean value);
  boolean getUseExternalYaw();
  void setUseExternalYaw(boolean value);
}
