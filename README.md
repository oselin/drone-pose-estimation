# Estimation of drone fleet poistion using MDS algorithm

The project aims to estimate the location of a fleet of drones via Multi-Dimensional Scaling (MDS) algorithm.

The performances are compared with standard trilateration algorithm, solved via numerical approach (Least Square Minimization - LSM)

The project has beed developed as ROS2 package (Distribution: Humble), mainly developed in Python3.

This research was conducted along with the course **Design Methods for Unmanned Vehicles**

Authors:
@[muttigiacomo](https://github.com/muttigiacomo)
@[oselin](https://github.com/oselin),
@[riccardoperiotto](https://github.com/riccardoperiotto),


## Build and install the project
In order to install the software and make it work, few steps are needed.

1) If ROS2 Humble is not installed, please install it by running
~~~bash
cd path/to/this/repository
chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
~~~

2) Install all the tools and requirements needed for running the project, by running
~~~bash
cd path/to/this/repository
chmod +x install_tools.sh
./install_tools.sh
~~~

## Run the project

To run the simulation using Gazebo, please run
~~~bash
cd path/to/this/repository/launch/launch_gazebo TYPE_NUMBER_OF_DESIRED_DRONES
~~~

An example can be
~~~bash
cd path/to/this/repository/launch/launch_gazebo 3
~~~

To run the simulation using the dynamic (numerical) model, please run
~~~bash
cd path/to/this/repository/launch/launch_test TYPE_NUMBER_OF_DESIRED_DRONES
~~~

An example can be
~~~bash
cd path/to/this/repository/launch/launch_test 20
~~~