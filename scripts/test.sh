#!/bin/sh

xterm  -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../turtlebot_simulator/turtlebot_gazebo/worlds/floor_plan.world " &
sleep 20
xterm  -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../turtlebot_simulator/turtlebot_gazebo/maps/map.yaml " &
sleep 5
xterm  -e "cd $(pwd)/../..; source devel/setup.bash ; roslaunch turtlebot_rviz_launchers view_navigation.launch "

