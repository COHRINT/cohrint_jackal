#!/bin/bash

# Source necessary setup files for ROS
source /opt/ros/noetic/setup.bash
source /home/cohrint/catkin_ws/devel/setup.bash

# Export robot name
export ROBOT_NAME=tars

# Launch ros master depending on specified indoor/outdoor/aspen
if [ $1 == 'indoor' ]
then
    roslaunch cohrint_jackal_bringup jackal_indoor.launch
elif [ $1 == 'aspen' ]
then
    roslaunch cohrint_jackal_bringup jackal_aspen.launch
else
    roslaunch cohrint_jackal_bringup jackal_outdoor.launch
fi
