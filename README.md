# Aerial Rock-and-Walk

This directory contains code for the aerial experiments, including:

- `rnw_ros` and `rnw_msgs` are specific to rock-and-walk.
- `uwb_mocap_broadcast` for transmitting mocap information.
- `djiros` and `n3ctrl` for the underlying quadrotor control



__Hardware Requirements__

- DJI N3 Flight Controller
- DJI Manifold 2-G Onboard Computer
- OptiTrack Motion Capture System
- 2 x Nooploop UWB Transmitter
- Logitech F710 Wireless Gamepad
- Custom end-effector for aerial rock-and-walk



__Run as hardware-in-the-loop (HIL) simulation__

1. Connect DJI N3 Autopilot to a PC/Mac with DJI Assistant installed
2. Enter simulation in DJI Assistant
3. `roslaunch rnw_ros sim.launch`



__Run real experiments__

1. Open OptiTrack
2. `roslaunch rnw_ros ground_station.launch` on ground station i.e. your laptop
3. SSH into the aircraft and `roslaunch rnw_ros real.launch`



__Pre-Flight Checklist__

Make sure UAV odometry is correct

- Fly it using `real.launch`, hover and moving around

Make sure `ConeState` is correct

- Calibrate the center point and tip point using `roslaunch rnw_ros mocap_calib.launch`
- Calibrate `ground_z` by placing a marker on the ground
- Run `roslaunch rnw_ros check_cone_state.launch`, check the cone state visually
- Check the estimated radius and the true radius, make sure they mactch.

Make sure `GripState` is correct

- Calibrate `flu_T_tcp` using `roslaunch rnw_ros mocap_calib.launch`
- Run `roslaunch rnw_ros check_grip_state.launch`, see does it make sense intuitively.
- Move the quadrotor along cone's shaft, see is the estimated `grip_depth` correct, adjust `flu_T_tcp` to make `grip_depth` match reality.

Make sure rock-and-walk planning is correct

- make sure `ConeState` and `GripState` are correct following the instructions above
- run `roslaunch rnw_ros check_rnw_planning.launch` 



## Documentation



__Control Flow__

1. `rnw_controller_node` sends trajectory to  `rnw_traj_server_node`

2. `rnw_traj_server_node` transform the trajectory into position command, send to `n3ctrl_node`

3. `n3ctrl_node` performs position control, sends attitude and trust commands to `djiros_node`



__State Flow__

1. OptiTrack sends out mocap for the aircraft and object through Ethernet.
2. `ground_station.launch` receives mocap from Ethernet and sends odometry to the onboard computer through UWB.
3. `uart_odom` receives odometry from the ground station, then publish to ROS

2. `pub_cone_state_node` reads odometry and configuration files, calculate `cone_state`, then publish to ROS

3. `rnw_controller_node` read `cone_state`, and performs rock-and-walk.



__Playback__

`rosbag record -a` is called by default, the `.bag` files can be retrieved after flight.

Inspect them using:

- `playback.launch` will replay the experiments in RViz.
- PlotJuggler
- MATLAB