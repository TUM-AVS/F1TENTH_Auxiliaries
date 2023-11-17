#!/usr/bin/bash
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
source ~/.bashrc
#installing Tutorials and Examples
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone https://github.com/stereolabs/zed-ros2-examples.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ~/.bashrc