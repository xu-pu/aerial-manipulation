Header header

float64 full_thrust

geometry_msgs/Vector3 ref_p
geometry_msgs/Vector3 ref_v
geometry_msgs/Vector3 ref_a

geometry_msgs/Vector3 plant_p
geometry_msgs/Vector3 plant_v
geometry_msgs/Vector3 plant_a

# control errors

geometry_msgs/Vector3 err_p
geometry_msgs/Vector3 err_v
geometry_msgs/Vector3 err_a

# from inner control loops

geometry_msgs/Vector3 cmd_v
geometry_msgs/Vector3 cmd_a

# disturbance

geometry_msgs/Vector3 int_v

# lpf

geometry_msgs/Vector3 lpf_acc
geometry_msgs/Vector3 lpf_thrust