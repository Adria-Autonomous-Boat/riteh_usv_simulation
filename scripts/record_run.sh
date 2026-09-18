#!/bin/bash
BAG=~/logs/data_logs/run_$(date +%Y%m%d_%H%M%S)
mkdir -p ~/logs/data_logs
echo "Recording to $BAG"
ros2 bag record \
    /odom \
    /otter/odom \
    /vessel_detection/closest \
    /vessel_detection/markers \
    /channel_nav/paused \
    /channel_nav/next_goal \
    -o "$BAG"
