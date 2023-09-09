#!/bin/bash

while true
do
    ros2 launch kaia_bringup publish_urdf.launch.py description:=false model:=$1 gui:=$2 &
    pid=$!
    trap "kill $!; exit" INT
    inotifywait -e modify $1
    kill $pid
done
