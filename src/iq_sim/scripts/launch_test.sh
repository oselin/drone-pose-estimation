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

echo
echo

# Launch test N times, set N here 
N=50
for ((i = 0; i < $N; i++)); do

    idx=$((i + 1))
    echo "Launching test [$idx/$N]"

    gnome-terminal --tab -- bash -c "ros2 run iq_sim test.py --ros-args --params-file install/iq_sim/share/iq_sim/config/config.yaml -p n_drones:=$1 -p seed:=$idx"
    echo '  test.py launched!'
    sleep 5

    # Launch ROS2 node to calculate the distances from the drones' coordinates
    gnome-terminal --tab -- bash -c "ros2 run iq_sim hub.py --ros-args --params-file install/iq_sim/share/iq_sim/config/config.yaml -p n_drones:=$1 -p seed:=$idx"
    echo '  hub launched!'
    sleep 5 

    # Launch the script main.py for running MDS, plotting the results and guiding the drones
    gnome-terminal --tab -- bash -c "ros2 run iq_sim main.py --ros-args --params-file install/iq_sim/share/iq_sim/config/config.yaml -p n_drones:=$1 -p run:='run$idx' -p seed:=$idx" 
    main_pid=$!
    echo '  main.py launched!'

    # something with "wait PID" would be better, but..
    sleep 300
    pkill -f test.py
    pkill -f hub.py
    pkill -f main.py

    echo "Terminated test [$idx/$N]"
    echo

    sleep 5
done


echo 'All tests run successfully!'

