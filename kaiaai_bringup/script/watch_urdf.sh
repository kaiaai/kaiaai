#!/bin/bash

while true
do
    ros2 launch kaiaai_bringup publish_urdf.launch.py urdf_path:=$1 gui:=$2 &
    pid=$!
    trap "kill $!; exit" INT
    inotifywait -e modify $1
    kill $pid
done
