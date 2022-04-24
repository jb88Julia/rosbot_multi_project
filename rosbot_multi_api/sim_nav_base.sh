#!/bin/bash
r1_x="2"
r1_y="0"
r1_z="0"

r2_x="1"
r2_y="3"
r2_z="0"

r3_x="-2"
r3_y="-3"
r3_z="0"


gnome-terminal -- roslaunch rosbot_multi_sim world_launch.launch
#gnome-terminal -- roslaunch rosbot_gazebo maze_world.launch 
sleep 5

gnome-terminal -- roslaunch rosbot_multi_sim main.launch robot1_init_pose:="-x $r1_x -y $r1_y -z $r1_z" robot2_init_pose:="-x $r2_x -y $r2_y -z $r2_z" robot3_init_pose:="-x $r3_x -y $r3_y -z $r3_z"
sleep 2

gnome-terminal -- roslaunch rosbot_multi_nav navigation.launch r1_initial_pose_x:="$r1_x" r1_initial_pose_y:="$r1_y" r1_initial_pose_z:="$r1_z" r2_initial_pose_x:="$r2_x" r2_initial_pose_y:="$r2_y" r2_initial_pose_z:="$r2_z" r3_initial_pose_x:="$r3_x" r3_initial_pose_y:="$r3_y" r3_initial_pose_z:="$r3_z"
