#!/bin/bash
mkdir -p new_bags
for i in *.bag; do
    echo "$i" || break
    rosbag filter "$i" new_bags/"$i" "topic != '/region/final_waypoints' and topic != '/region/final_path'"
done