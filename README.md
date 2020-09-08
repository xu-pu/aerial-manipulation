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



## DOC



### Control Flow

`rnw_controller_node` sends trajectory to  `rnw_traj_server_node`

`rnw_traj_server_node` transform the trajectory into feedforward command, send to `n3ctrl_node`

`n3ctrl_node` performs cascade PID control, send attitude and trust command to `djiros_node`



### State Flow

`uart_odom` receive odometry of aircraft and object from the ground station, publish to ROS

`pub_cone_state_node` reads odometry and configuration files, calculate cone state, including contact point, apex point, angular velocity etc, and publish to ROS

`rnw_controller_node` read cone state, and plan trajectories accordingly



