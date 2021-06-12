[1mdiff --git a/rnw_ros/include/rnw_ros/rnw_planner_v2.h b/rnw_ros/include/rnw_ros/rnw_planner_v2.h[m
[1mindex a691187..ce47cf3 100644[m
[1m--- a/rnw_ros/include/rnw_ros/rnw_planner_v2.h[m
[1m+++ b/rnw_ros/include/rnw_ros/rnw_planner_v2.h[m
[36m@@ -69,7 +69,7 @@[m [mstruct rnw_planner_v2_t {[m
      */[m
     void spin();[m
 [m
[31m-    rnw_command_t rnw_command;[m
[32m+[m[32m    rnw_command_t cmd;[m
 [m
     void start_walking();[m
 [m
[1mdiff --git a/rnw_ros/nodes/rnw_cable_node.cpp b/rnw_ros/nodes/rnw_cable_node.cpp[m
[1mindex 200b2a2..ecc91b3 100644[m
[1m--- a/rnw_ros/nodes/rnw_cable_node.cpp[m
[1m+++ b/rnw_ros/nodes/rnw_cable_node.cpp[m
[36m@@ -150,14 +150,14 @@[m [mstruct cable_rnw_node_t {[m
     void spin(const ros::TimerEvent &event ){[m
       rnw_planner.spin();[m
       if ( !rnw_planner.is_walking ) {[m
[31m-        last_cmd_idx = rnw_planner.rnw_command.cmd_idx;[m
[32m+[m[32m        last_cmd_idx = rnw_planner.cmd.cmd_idx;[m
       }[m
[31m-      else if ( rnw_planner.rnw_command.cmd_idx > last_cmd_idx ) {[m
[31m-        ROS_WARN("[swarm_rnw] new command #%u received!",rnw_planner.rnw_command.cmd_idx);[m
[32m+[m[32m      else if (rnw_planner.cmd.cmd_idx > last_cmd_idx ) {[m
[32m+[m[32m        ROS_WARN("[swarm_rnw] new command #%u received!",rnw_planner.cmd.cmd_idx);[m
         drone.execute_trajectory([m
[31m-                drone.plan(rnw_planner.rnw_command.control_point_setpoint + drone.cable_length * Vector3d::UnitZ())[m
[32m+[m[32m                drone.plan(rnw_planner.cmd.control_point_setpoint + drone.cable_length * Vector3d::UnitZ())[m
         );[m
[31m-        last_cmd_idx = rnw_planner.rnw_command.cmd_idx;[m
[32m+[m[32m        last_cmd_idx = rnw_planner.cmd.cmd_idx;[m
       }[m
     }[m
 [m
[1mdiff --git a/rnw_ros/nodes/swarm_rnw_controller_node.cpp b/rnw_ros/nodes/swarm_rnw_controller_node.cpp[m
[1mindex ee3cff7..5a3a952 100644[m
[1m--- a/rnw_ros/nodes/swarm_rnw_controller_node.cpp[m
[1m+++ b/rnw_ros/nodes/swarm_rnw_controller_node.cpp[m
[36m@@ -255,12 +255,12 @@[m [mstruct rnw_node_t {[m
     void spin(const ros::TimerEvent &event ){[m
       rnw_planner.spin();[m
       if ( !rnw_planner.is_walking ) {[m
[31m-        last_cmd_idx = rnw_planner.rnw_command.cmd_idx;[m
[32m+[m[32m        last_cmd_idx = rnw_planner.cmd.cmd_idx;[m
       }[m
[31m-      else if ( rnw_planner.rnw_command.cmd_idx > last_cmd_idx ) {[m
[31m-        ROS_WARN("[swarm_rnw] new command #%u received!",rnw_planner.rnw_command.cmd_idx);[m
[31m-        execute_rnw_cmd(rnw_planner.rnw_command);[m
[31m-        last_cmd_idx = rnw_planner.rnw_command.cmd_idx;[m
[32m+[m[32m      else if (rnw_planner.cmd.cmd_idx > last_cmd_idx ) {[m
[32m+[m[32m        ROS_WARN("[swarm_rnw] new command #%u received!",rnw_planner.cmd.cmd_idx);[m
[32m+[m[32m        execute_rnw_cmd(rnw_planner.cmd);[m
[32m+[m[32m        last_cmd_idx = rnw_planner.cmd.cmd_idx;[m
       }[m
     }[m
 [m
[1mdiff --git a/rnw_ros/src/rnw_planner_v2.cpp b/rnw_ros/src/rnw_planner_v2.cpp[m
[1mindex 6f5c471..b8c38b9 100644[m
[1m--- a/rnw_ros/src/rnw_planner_v2.cpp[m
[1m+++ b/rnw_ros/src/rnw_planner_v2.cpp[m
[36m@@ -45,7 +45,7 @@[m [mrnw_msgs::RnwState rnw_planner_v2_t::to_rnw_state() const {[m
   msg.header.frame_id = "world";[m
   msg.is_walking = is_walking;[m
   msg.step_count = step_count;[m
[31m-  msg.setpoint = uav_utils::to_point_msg(rnw_command.control_point_setpoint);[m
[32m+[m[32m  msg.setpoint = uav_utils::to_point_msg(cmd.control_point_setpoint);[m
   return msg;[m
 }[m
 [m
[36m@@ -65,7 +65,7 @@[m [mvoid rnw_planner_v2_t::control_loop(){[m
   }[m
 [m
   // avoid transient states[m
[31m-  if ( ros::Time::now() - rnw_command.stamp < ros::Duration(rnw_config.rnw.min_step_interval) ) {[m
[32m+[m[32m  if (ros::Time::now() - cmd.stamp < ros::Duration(rnw_config.rnw.min_step_interval) ) {[m
     return;[m
   }[m
 [m
[36m@@ -93,7 +93,7 @@[m [mvoid rnw_planner_v2_t::control_loop(){[m
 [m
 void rnw_planner_v2_t::plan_cmd_walk(){[m
 [m
[31m-  rnw_command.stamp = ros::Time::now();[m
[32m+[m[32m  cmd.stamp = ros::Time::now();[m
 [m
   // adjust nutation first[m
 [m
[36m@@ -127,9 +127,9 @@[m [mvoid rnw_planner_v2_t::plan_cmd_walk(){[m
   Vector3d next_v = rot * v;[m
   Vector3d setpoint_apex = G + next_v;[m
 [m
[31m-  rnw_command.control_point_setpoint = setpoint_apex;[m
[31m-  rnw_command.heading = precession_regulator.desired_heading;[m
[31m-  rnw_command.cmd_idx++;[m
[32m+[m[32m  cmd.control_point_setpoint = setpoint_apex;[m
[32m+[m[32m  cmd.heading = precession_regulator.desired_heading;[m
[32m+[m[32m  cmd.cmd_idx++;[m
   step_count++;[m
   cmd_fsm = cmd_fsm_e::pending;[m
 [m
