
# Install ROS2
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# SETUP SOURCES
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

pip install catkin-pkg==0.3.0 defusedxml
# Install ROS2
sudo apt install ros-humble-desktop
sudo apt install ros-humble-mavros
sudo apt install ros-humble-mavros-extras

# Sourcing the setup script
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo 'source /opt/ros/humble/setup.bash ' >> ~/.bashrc 
echo 'source ~/ros2_humble_ws/install/setup.bash' >> ~/.bashrc 
echo 'source ~/ros2_humble_ws/install/local_setup.bash' >> ~/.bashrc 

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc 

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Resolve dependencies
rosdep install -i --from-path src --rosdistro humble -y

colcon build

# echo 'alias launch_multi="ros2 launch iq_sim multi_drone.launch.py"' >> ~/.bashrc 
echo 'source /home/$USER/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc 
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Install ArduPilot
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Downgrade to version 4.3.0 (working)
git checkout ArduCopter-4.3.0
git submodule update --recursive

Tools/environment_install/install-prereqs-ubuntu.sh -y

# Install Gazebo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install gazebo11 libgazebo-dev

cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo

mkdir build
cd build
cmake ..
make -j4
sudo make install

# echo 'export GAZEBO_MODEL_PATH=$HOME/drone-pose-estimation/src/iq_sim/models:$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc 
# . ~/.bashrc


wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

reboot

