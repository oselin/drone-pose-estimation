
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

# Sourcing the setup script
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo 'source /opt/ros/humble/setup.bash ' >> ~/.bashrc 

echo 'source $HOME/drone-pose-estimation/install/local_setup.bash' >> ~/.bashrc 
# source $HOME/Desktop/ardupilot/Tools/completion/completion.bash

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc 

echo 'export GAZEBO_MODEL_PATH=$HOME/drone-pose-estimation/src/iq_sim/models:$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc 

echo 'alias launch_multi="ros2 launch iq_sim multi_drone.launch.py"' >> ~/.bashrc 
echo 'source /home/$USER/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc 
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

cd ~
sudo apt install git
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot

cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Install Gazebo
sudo apt-get install gazebo11 libgazebo11-dev


reboot

