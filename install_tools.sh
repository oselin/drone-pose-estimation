# Copy ROS2 package to dedicated folder
echo 'Copying project package to ROS2 workspace'
cp -r drone_pose_estimation ~/ros2_humble/src


# Install Gazebo
echo '\n\n\nInstalling Gazebo...'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install gazebo11 libgazebo-dev

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc 


# Install ArduPilot
echo '\n\n\nInstalling ArduPilot'
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Downgrade to version 4.3.0 (working)
git checkout ArduCopter-4.3.0
git submodule update --recursive

Tools/environment_install/install-prereqs-ubuntu.sh -y

echo 'source /home/$USER/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc 


# Install ArduPilot-Gazebo plugin
echo '\n\n\nInstalling ArduPilot-Gazebo Plugin'
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo

mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'export GAZEBO_MODEL_PATH=$HOME/ros2_humble/src/drone_pose_estimation/models:$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc 


# Install GeographicLib
echo '\n\n\nInstalling GeographicLib'
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

reboot

