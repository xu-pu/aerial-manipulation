Header header

float64 full_thrust

geometry_msgs/Vector3 ref_p
geometry_msgs/Vector3 ref_v
geometry_msgs/Vector3 ref_a

geometry_msgs/Vector3 plant_p
geometry_msgs/Vector3 plant_v
geometry_msgs/Vector3 plant_a

# from inner control loops

geometry_msgs/Vector3 cmd_v
geometry_msgs/Vector3 cmd_a

# integral terms

geometry_msgs/Vector3 int_v
geometry_msgs/Vector3 int_a

# err = ref - plant, for easy debug

float64 err_p
float64 err_v
float64 err_a