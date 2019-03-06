#!/usr/bin/env bash

export ROS_MASTER_URI=http://localhost:11311
local_info=(`hostname -I`)
local_ip=${local_info[0]}
export ROS_IP=$local_ip
