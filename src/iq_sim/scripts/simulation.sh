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

# Generate the correct models
echo
python3 $SCRIPTPATH/generate_models.py $1

# Update the Ardupilot vehicleinfo.py
echo
echo "Updating vehicleinfo.py details..."
python3 $SCRIPTPATH/generate_vehicleinfo.py $1

# Go back to ws folder
echo 
echo 'Building the project with the new files...'
cd $SCRIPTPATH/../../..
colcon build

# Launch populated world
echo
gnome-terminal --tab -- bash -c "ros2 launch iq_sim multi-drone.launch.py" &

sleep 20

# Launch the ArduCopter sessions
for ((i = 0; i < $1; i++)); do
    drone_idx=$((i + 1))
    gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone$drone_idx -I$i"
    echo "Launching Ardupilot session [$drone_idx/$1]"
done

sleep 120
# Launch mavros
echo
echo 'Launching an instance of mavros for each node'
gnome-terminal --tab -- bash -c "ros2 launch iq_sim multi-apm.launch.py n_drones:=$1"

# Launch ROS2 node to calculate the distances from the drones' coordinates
echo
echo 'Launching the hub...'
gnome-terminal --tab -- bash -c "ros2 run iq_sim hub.py --ros-args -p n_drones:=$1 " # -p noise:='none' "
echo 'hub launched!'

# Launch the script main.py for running MDS, plotting the results and guiding the drones
echo
echo 'Launching main.py...'
gnome-terminal --tab -- bash -c "ros2 run iq_sim main.py --ros-args -p n_drones:=$1 -p mode:=1 " # file !?
echo 'main.py launched!'