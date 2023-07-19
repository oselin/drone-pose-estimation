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

# Generate the correct world
echo
python3 $SCRIPTPATH/generate_world.py $1

# Generate the correct .parm files
echo
python3 $SCRIPTPATH/generate_gazebo_parms.py $1

# Generate the correct models
echo
python3 $SCRIPTPATH/generate_models.py $1

ros2 launch iq_sim multi_drone.launch.py

# Launch the ArduCopter sessions
for ((i = 0; i < $1; i++)); do
    drone_idx=$((i + 1))
    echo "Launching Ardupilot session [$drone_idx/$1]"
    gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone$drone_idx -I$i"
done

# Launch the ROS2 Node
echo
#ros2 launch iq_sim MDS