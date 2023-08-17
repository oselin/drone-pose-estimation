# Copy ROS2 package to dedicated folder
echo 'Copying project package to ROS2 workspace'
cp -r drone_pose_estimation ~/ros2_ws/src/drone_pose_estimation

# Install Python libraries
pip install -U scikit-learn

# Install Gazebo
echo
echo
echo 'Installing Gazebo...'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install gazebo libgazebo-dev

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc 


# Install ArduPilot
echo
echo
echo 'Installing ArduPilot'
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Downgrade to version 4.3.0 (working)
git checkout Copter-4.3.0
git submodule update --recursive

Tools/environment_install/install-prereqs-ubuntu.sh -y

echo 'source /home/$USER/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc 


# Install ArduPilot-Gazebo plugin
echo
echo
echo 'Installing ArduPilot-Gazebo Plugin'
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo

mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'export GAZEBO_MODEL_PATH=$HOME/ros2_ws/src/drone_pose_estimation/models:$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc 


# Install GeographicLib
echo
echo
echo 'Installing GeographicLib'
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

reboot

