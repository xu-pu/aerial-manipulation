# Aerial Manipulation

This repo contains code for aerial manipulation experiments.



### Run as hardware-in-the-loop (HIL) simulation

1. Connect DJI N3 Autopilot to a PC/Mac with DJI Assistant installed
2. Enter simulation in DJI Assistant
3. `roslaunch rnw_ros sim.launch`



### Run real experiments

1. Open OptiTrack Motive using my config file 
2. `roslaunch rnw_ros ground_station.launch` on ground station i.e. your laptop
3. SSH into the aircraft and `roslaunch rnw_ros real.launch`



### Run software-only simulation



## Preparation



make sure UAV odometry is correct

- fly it using `real.launch`, hover and moving around



make sure `ConeState` is correct

- calibrate the center point and tip point using `roslaunch rnw_ros mocap_calib.launch`
- calibrate `ground_z` by placing a marker on the ground
- run `roslaunch rnw_ros check_cone_state.launch`, check the cone state visually
- check the estimated radius and true radius.



make sure `GripState` is correct

- calibrate `flu_T_tcp` using `roslaunch rnw_ros mocap_calib.launch`
- run `roslaunch rnw_ros check_grip_state.launch`, see does it make sense intuitively.
- move uav along the shaft, see is the estimated `grip_depth` correct, adjust `flu_T_tcp` to make `grip_depth` reflect reality.



make sure r-n-w planning is correct

- make sure `ConeState` and `GripState` is correct, following the instructions above
- run `roslaunch rnw_ros check_rnw_planning.launch` 



## DOC



### Control Flow

`rnw_controller_node` sends trajectory to  `rnw_traj_server_node`

`rnw_traj_server_node` transform the trajectory into feedforward command, send to `n3ctrl_node`

`n3ctrl_node` performs cascade PID control, send attitude and trust command to `djiros_node`



### State Flow

`uart_odom` receive odometry of aircraft and object from the ground station, publish to ROS

`pub_cone_state_node` reads odometry and configuration files, calculate cone state, including contact point, apex point, angular velocity etc, and publish to ROS

`rnw_controller_node` read cone state, and plan trajectories accordingly



