#!/usr/bin/env bash

# Kills all rosnodes then kills the roscore

rosnode kill -a
res=`ps aux |grep -i roscore`
str_arr=($res)
process_num=${str_arr[1]}
echo $process_num
kill $process_num
