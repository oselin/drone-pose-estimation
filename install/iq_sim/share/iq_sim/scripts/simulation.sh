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

# Clean Gazebo
pkill gz

# Generate the correct world
echo
python3 $SCRIPTPATH/generate_world.py $1

# Generate the correct .parm files
echo
python3 $SCRIPTPATH/generate_gazebo_parms.py $1

# Generate the corrnoiseect models
echo
python3 $SCRIPTPATH/generate_models.py $1

# Go back to ws folder
echo 
echo 'Building the project with the new files..'
cd $SCRIPTPATH/../../..
colcon build

# Launch populated world
echo
gnome-terminal --tab -- bash -c "ros2 launch iq_sim multi-drone.launch.py"

# Launch the ArduCopter sessions
for ((i = 0; i < $1; i++)); do
    drone_idx=$((i + 1))
    echo "Launching Ardupilot session [$drone_idx/$1]"
    gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone$drone_idx -I$i"
done

# Launch mavros
gnome-terminal --tab -- bash -c "ros2 launch iq_sim multi-apm.launch.py $1"

# Launch ROS2 node to calculate the distances from the drones' coordinates
gnome-terminal --tab -- bash -c "ros2 run iq_sim hub.py --ros-args -p n_drones:=$1 -p noise:='none' "


# Launch the ROS2 Node
echo
#ros2 launch iq_sim MDS