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
    gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone$drone_idx -I$i"
    echo "Launching Ardupilot session [$drone_idx/$1]"
done

# Launch mavros
echo
echo 'Launching an instance of mavros for each node'

# 1. close to working way: launch a mavros instance for each drone 
udp_port_base=14541
for ((i=1;i<=$1;i++)); do
    udp_port_in=$((udp_port_base + i * 10))
    udp_port_out=$((udp_port_in + 4))
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=udp://127.0.0.1:$udp_port_in@$udp_port_out \
        -p target_system_id:=$i \
        -p target_component_id:=1 \
        -p fcu_protocol:=v2.0 \
        -r __ns:=/drone$i \
        --params-file /opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml \
        --params-file /opt/ros/humble/share/mavros/launch/apm_config.yaml &
    # note: no need to define the node name thanks to the ns
    echo "Launching mavros [$drone_idx/$1]"
done
# 2. alternative way (it does not work as the launcher does not know n_drones)
# gnome-terminal --tab -- bash -c "ros2 launch iq_sim multi-apm.launch.py"

# Launch ROS2 node to calculate the distances from the drones' coordinates
gnome-terminal --tab -- bash -c "ros2 run iq_sim hub.py --ros-args -p n_drones:=$1 -p noise:='none' "

#ros2 launch iq_sim MDS # ehehe