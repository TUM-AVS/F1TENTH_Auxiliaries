#!/usr/bin/bash
apt install software-properties-common -y
add-apt-repository universe -y
apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt upgrade -y
apt install ros-foxy-desktop python3-argcomplete -y
apt install ros-dev-tools -y
apt install python3-colcon-common-extensions -y
su -l f1tenth -c '
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
'
apt install python3-bloom python3-rosdep fakeroot debhelper dh-python -y
rosdep init
su -l f1tenth -c '
rosdep update --rosdistro="foxy"
'
