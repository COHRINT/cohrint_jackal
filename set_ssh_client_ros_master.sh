#!/usr/bin/env bash

client_info=($SSH_CLIENT)
client_ip=${client_info[0]}
export ROS_MASTER_URI=http://$client_ip:11311
local_info=(`hostname -I`)
local_ip=${local_info[0]}
export ROS_IP=$local_ip
