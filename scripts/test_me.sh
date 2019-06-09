#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/HomeServiceRobot/worlds/pankaj_world.world" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/HomeServiceRobot/maps/map.yaml" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects_node" 