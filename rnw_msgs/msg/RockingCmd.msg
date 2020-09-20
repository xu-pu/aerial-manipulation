Header header
uint8 fsm
uint8 cmd_type
uint32 cmd_idx
# waypoint
geometry_msgs/Point setpoint_uav
geometry_msgs/Point setpoint_apex
# walking
bool is_walking
uint8 walk_idx
float64 tau_deg
uint32 step_count
# grip monitoring
rnw_msgs/GripState grip_state
float64 setpoint_nutation_deg
float64 setpoint_grip_depth
float64 err_grip_depth
float64 err_nutation_deg
