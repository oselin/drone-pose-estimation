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

sudo apt install python3
sudo apt install python3-pip

# Install ROS2
sudo apt install ros-humble-desktop
sudo apt install ros-humble-mavros
sudo apt install ros-humble-mavros-extras

# Install catkin
pip install -U catkin-pkg

# Install colcon
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install python3-colcon-common-extensions

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Resolve dependencies
rosdep install -i --from-path src --rosdistro humble -y

colcon build

# Sourcing the setup script
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo 'source /opt/ros/humble/setup.bash ' >> ~/.bashrc 
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc 
echo 'source ~/ros2_ws/install/local_setup.bash' >> ~/.bashrc 

reboot
