#!/bin/bash

ProjectPath=$(find ~/ -name "QuadRotorBench")
echo $ProjectPath
gnome-terminal --title="Gazebo" --geometry=60x20 -- bash -i -c "cd $ProjectPath/MyLaunch/;roslaunch MyGazebo.launch world_name:=$ProjectPath/MyWorld/MyForest.world;exec bash" &
sleep 5
gnome-terminal --title="PX4" --geometry=60x20 -- bash -i -c "cd ~/px4;sudo ./bin/px4 -s HITL.config;exec bash" &
sleep 5
gnome-terminal --title="FastLIO" --geometry=60x20 -- bash -i -c "roslaunch fast_lio mapping_mid360.launch;exec bash" &
sleep 5
gnome-terminal --title="EgoPlanner" --geometry=60x20 -- bash -i -c "roslaunch ego_planner single_run_in_exp.launch;exec bash" &
sleep 5
gnome-terminal --title="MAVROS" --geometry=60x20 -- bash -i -c "cd $ProjectPath/MyLaunch/;roslaunch MyMAVROS.launch;exec bash" &
sleep 5
gnome-terminal --title="OffboardControl" --geometry=60x20 -- bash -i -c "rosrun ego_planner offboard_control;exec bash" &
sleep 5
gnome-terminal --title="PubGoal" --geometry=60x20 -- bash -i -c "rosrun ego_planner pub_goal;exec bash" &
