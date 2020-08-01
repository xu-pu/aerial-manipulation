# Aerial Manipulation

This repo contains code for aerial manipulation experiments.



### Run as hardware-in-the-loop (HIL) simulation

1. Connect DJI N3 Autopilot to a PC/Mac with DJI Assistant installed
2. Enter simulation in DJI Assistant
3. `roslaunch rnw_ros sim.launch`



### Run real experiments

1. Open OptiTrack Motive using my config file 
2. `roslaunch rnw_ros ground_station.launch` on ground station i.e. your laptop
3. SSH into the aircraft
4. `roslaunch djiros djiros.launch` through SSH
5. `roslaunch uart_odom client` through SSH
6. `roslaunch rnw_ros real.launch` through SSH

> TODO merge into one launch file on the aircraft side



### Run software-only simulation



