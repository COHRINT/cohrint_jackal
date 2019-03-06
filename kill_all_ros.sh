#!/usr/bin/env bash

# Kills all rosnodes then kills the roscore

rosnode kill -a
res=`ps aux |grep -i rosmaster`
str_arr=($res)
process_num=${str_arr[1]}
echo $process_num
sudo kill $process_num
res=`ps aux |grep -i ros`
str_arr=($res)
process_num=${str_arr[1]}
sudo kill $process_num
