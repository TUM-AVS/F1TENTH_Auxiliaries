#!/usr/bin/bash
cd $HOME
mkdir -p f1tenth_ws/src
cd f1tenth_ws
colcon build
cd src
git clone https://github.com/f1tenth/f1tenth_system.git
cd f1tenth_system
git submodule update --init --force --remote
cd $HOME/f1tenth_ws
source /opt/ros/foxy/setup.bash
rosdep update --rosdistro="foxy"
rosdep install --from-paths src -i -y
colcon build
