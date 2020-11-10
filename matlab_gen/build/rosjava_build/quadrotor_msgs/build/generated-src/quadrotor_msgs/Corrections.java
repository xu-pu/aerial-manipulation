package quadrotor_msgs;

public interface Corrections extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Corrections";
  static final java.lang.String _DEFINITION = "float64 kf_correction\nfloat64[2] angle_corrections\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getKfCorrection();
  void setKfCorrection(double value);
  double[] getAngleCorrections();
  void setAngleCorrections(double[] value);
}
