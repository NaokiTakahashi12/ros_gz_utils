#!/bin/bash

ros_gz_gazebo_process_ids=$(ps aux | grep "ros_gz" | grep -v grep | awk '{print $2}')

if [ -n "$ros_gz_gazebo_process_ids" ]
then
    kill -9 $ros_gz_gazebo_process_ids
else
    echo "Not found ros_gz process"
fi
