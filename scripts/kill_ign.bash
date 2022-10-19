#!/bin/bash

ign_gazebo_process_ids=$(ps aux | grep "ign gazebo" | grep -v grep | awk '{print $2}')

if [ -n "$ign_gazebo_process_ids" ]
then
    kill -9 $ign_gazebo_process_ids
else
    echo "Not found ignition gazebo process"
fi

