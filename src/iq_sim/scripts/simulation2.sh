#!/bin/env bash

# Check if exactly one argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <number of drones>"
    exit 1
fi

# Validate if the argument is an integer
if ! [[ $1 =~ ^[0-9]+$ ]]; then
    echo "Error: Argument must be an integer."
    exit 1
fi

SCRIPT=$(realpath -s "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# Compile nodes
echo 
echo 'Building the project with the new files...'
cd $SCRIPTPATH/../../..
colcon build

# Launch the script main.py for running MDS, plotting the results and guiding the drones
# altitude just creates confusion, let's skip it
echo
echo 'Launching test.py...'
gnome-terminal --tab -- bash -c "ros2 run iq_sim test.py --ros-args -p n_drones:=$1"
echo 'test.py launched!'
sleep 5

# Launch ROS2 node to calculate the distances from the drones' coordinates
echo
echo 'Launching the hub...'
gnome-terminal --tab -- bash -c "ros2 run iq_sim hub.py --ros-args -p n_drones:=$1 -p noise_dist_std:=0.3 "
echo 'hub launched!'
sleep 5 

# Launch the script main.py for running MDS, plotting the results and guiding the drones
echo
echo 'Launching main.py...'
gnome-terminal --tab -- bash -c "ros2 run iq_sim main.py --ros-args -p environment:='test' -p n_drones:=$1 -p altitude:=0.0 -p noise_time_std:=0.0 "
echo 'main.py launched!'
