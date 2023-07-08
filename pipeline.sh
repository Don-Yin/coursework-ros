#!/bin/bash

# # Step 1
# python3 1_plan_path.py
# if [ $? -ne 0 ]; then
#     echo "Python script failed. Exiting..."
#     exit 1
# fi

# Step 2
source setup.sh
if [ $? -ne 0 ]; then
    echo "Setup failed. Exiting..."
    exit 1
fi

# Step 3
source chmod.sh
if [ $? -ne 0 ]; then
    echo "Chmod failed. Exiting..."
    exit 1
fi

# Step 4
gnome-terminal -- bash -c "source roscore.sh; sleep 3; bash"

# Step 5
sleep 3
gnome-terminal -- bash -c "roslaunch robot_moveit demo.launch; sleep 7; bash"

# Step 6
sleep 7
gnome-terminal -- bash -c "source marker.sh; bash"

# Step 7
gnome-terminal -- bash -c "source move.sh; bash"
