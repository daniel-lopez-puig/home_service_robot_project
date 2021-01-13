#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}
. ../../../../devel/setup.bash
xterm  -e  " source /opt/ros/kinetic/setup.bash; cd ${DIR}; source ../../../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=`rospack find home_service_robot`/worlds/daniel.world " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; cd ${DIR}; source ../../../../devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=`rospack find home_service_robot`/map/map.yaml" & 
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; cd ${DIR}; source ../../../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5