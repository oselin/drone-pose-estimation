# Estimation of drone fleet poistion using MDS algorithm

The project aims to estimate the location of a fleet of drones via Multi-Dimensional Scaling (MDS) algorithm.

The performances are compared with standard trilateration algorithm, solved via numerical approach (Least Square Minimization - LSM)

The project has beed developed as ROS2 package (Distribution: Humble), mainly developed in Python3.

This research was conducted along with the course **Design Methods for Unmanned Vehicles**

Authors:
@[muttigiacomo](https://github.com/muttigiacomo)
@[oselin](https://github.com/oselin),
@[riccardoperiotto](https://github.com/riccardoperiotto),


## Build and run
In order to install the software and make it work, few steps are needed.

In this repository, the script `install.sh` helps the user to install all the requirements needed to run the package


To compile the project, please follow these bash commands

Move into dev folder
~~~bash
cd dev_ws
~~~

Copy this repository as ROS2 package
~~~bash
cp -r path-to-this-repo ~/ros2_humble/src
~~~

To run the simulation using Gazebo, please run
~~~bash
cd ~/ros2_humble/src/iq_sim/scripts/launch_gazebo TYPE_NUMBER_OF_DESIRED_DRONES
~~~

An example can be
~~~bash
cd ~/ros2_humble/src/iq_sim/scripts/launch_gazebo 3
~~~

To run the simulation using the dynamic (numerical) model, please run
~~~bash
cd ~/ros2_humble/src/iq_sim/scripts/launch_test TYPE_NUMBER_OF_DESIRED_DRONES
~~~

An example can be
~~~bash
cd ~/ros2_humble/src/iq_sim/scripts/launch_test 20
~~~