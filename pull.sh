#!/bin/bash
git reset --hard HEAD
git pull

dir_path="src/robot/src"

for file in "$dir_path"/*.py
do
    chmod +x "$file"
done

source /opt/ros/noetic/setup.bash
source devel/setup.bash