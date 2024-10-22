# Instructions to set up an NVIDIA Jetson Orin Nano for all TUM F1TENTH Labs using an SD Card or SSD.
The following instructions where tested and verifies on the following hard- and software on the 31. Jan 2024
- Jetson Orin Nano 8GB Development Kit
- Jetpack 5.1.2
- SD Card (SanDisk Extreme 128GB) and SSD
- ZED 1/2 camera
- Hokuyo UST-10LX
- Hint: Do not use SD Cards if you use the Jetson intensively. It is prone to have corrupted files after some time that can break the entire system.

## General Setup
0. If you want to resetup and SD Card, save all important data from the previous SD Card image.
1. Flash an SD Card with the correct Jetpack Version (currently 5.1.2 [Jetpack 5.1.2 Download](https://developer.nvidia.com/embedded/jetpack-sdk-512)) using, e.g., Balena Etcher: [Balena Etcher Download](https://etcher.balena.io).
2. Insert the SD Card, power up the Jetson and set up the user (username: f1tenth, password: f1tenth).
3. Alternative flash the SSD with the SDK Manager from NVidia: [NVidia SDK Manager](https://developer.nvidia.com/sdk-manager)
4. Update and Upgrade the software:
```
sudo apt-get update && sudo apt-get upgrade
```
5. Install the Wifi driver according to the installation instructions

Clone the repository:
```
git clone https://github.com/RinCat/RTL88x2BU-Linux-Driver.git
```
Navigate inside the directory and build the module:
```
cd RTL88x2BU-Linux-Driver && make clean && make && sudo make install
```
6. Insert the Bluetooth Adapter and Wifi Adapter and check successful connections
7. For usage at TUM in the eduroam Wifi, please log into Eduroam using the official python script you can download [here](https://cat.eduroam.org)
8. Install X11VNC
```
sudo apt-get install x11vnc
```
9. Set the X11 password to f1tenth and save it at the default location
```
x11vnc -storepasswd
```



## Install ROS with the base F1TENTH software stack
1. [Install ROS](https://docs.ros.org/en/foxy/Installation.html)
2. [Install the F1TENTH stack](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#doc-drive-workspace). Use "rosdep update --include-eol-distros"!

> [!IMPORTANT]  
> The udev files (e.g., /etc/udev/rules.d/99-hokuyo.rules) do not exist yet. Running
> ```
> sudo nano /etc/udev/rules.d/99-hokuyo.rules
> ```
> will create the corresponding file.


3. Add two lines to the bashrc file so no car communicates with each other over Wifi and ROS2 Foxy is automatically sourced.
Open the bash script with:
```
sudo nano ~/.bashrc
```
Enter the following commands after the last line, save and then close the file
```
export ROS_LOCALHOST_ONLY=1
export /opt/ros/foxy/setup.bash
```
4. [Install Zerotier](https://www.zerotier.com/download/) and add the car to the correct network by replacing <network_id> with the corresponding parameter
```
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join <network_id>
```
5. Add a second Ethernet profile called "Standard" with default settings.

## Install the required Packages for Deep Learning
1. Install the corresponding ZED SDK from here: [ZED SDK Download](https://www.stereolabs.com/developers/release). The installation instructions are here: [ZED SDK Installation Instructions](https://www.stereolabs.com/docs/installation/jetson)). Enter yes to all options until it wants to download the AI modules. Enter no there.
```
pip3 install onnx
cd Downloads && chmod +x ZED_SDK* && ./ZED_SDK*
```
2. Add the ZED ROS2 Wrapper to the "f1tenth_ws" workspace and install according to the official repository: [ZED ROS2 Wrapper Installation Instructions](https://github.com/stereolabs/zed-ros2-wrapper)
```
cd ~/f1tenth_ws/src/
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
```
Test the ZED2 Wrapper via:
```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```
Replace <camera_model> with the model of the camera that you are using: 'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm'.

3. Increase the Swap Size to 16GB according to this repo: [JetsonHacks Swap Size](https://github.com/JetsonHacksNano/resizeSwapMemory "JetsonHacks Swap Size")
```
sudo nano /etc/systemd/nvzramconfig.sh
```
Change the line:
```
mem=$((("${totalmem}" / 2 / "${NRDEVICES}") * 1024))
```
to:
```
mem=$((("${totalmem}" / "${NRDEVICES}") * 1024 * 2))
```

4. Install Tensorflow for Jetson for Tensorboard: [Install Tensorflow on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html "Install Tensorflow on Jetson")
```
sudo apt-get update
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip
sudo python3 -m pip install --upgrade pip
sudo pip3 install -U testresources setuptools==65.5.0
sudo pip3 install -U numpy==1.22 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 protobuf pybind11 pkgconfig packaging h5py==3.7.0
sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v512 tensorflow==2.12.0+nv23.06
```
5. Install PyTorch with CUDA bindings on Jetson: [Install PyTorch on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html "Install PyTorch on Jetson")
Download the PyTorch wheel from this link: [Download PyTorch wheel](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
Replace <path/to/torch> with the full path including /home/etc. to the wheel file
```
sudo apt-get -y update;
sudo apt-get -y install python3-pip libopenblas-dev
export TORCH_INSTALL=<path/to/torch>
python3 -m pip install --upgrade pip
python3 -m pip install numpy==1.22
python3 -m pip install --no-cache $TORCH_INSTALL
```
6. Verify the functionality of the install by running the following commands in a terminal
```
python3
import torch
print(torch.cuda.is_available())
exit()
```
If the command prints true at the end, Torch can use the GPU.

7. Install Torchvision with [Install Torchvision on Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
Look up the torchvision version corresponding to the PyTorch Version
and insert it as e.g. v0.16.1 in <version> and 0.16.1 for <build_version>. The installation takes quite some time.
```
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch <version> https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=<build_version>
python3 setup.py install --user
cd
```
Test via:
```
python3
import torchvision
exit()
```

## Install Jetson Inference for Object Detection Training and Inference on the Jetson
1. Install the Repository:
```
sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```
2. Export the Path:
```
export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so
```
3. Test the Inference after plugging in the ZED camera into a USB port:
```
cd jetson-inference/build/aarch64/bin
./detectnet /dev/video0
```
4. Test Model Training:
```
cd jetson-inference/python/training/classification/data
wget https://nvidia.box.com/shared/static/o577zd8yp3lmxf5zhm38svrbrv45am3y.gz -O cat_dog.tar.gz
tar xvzf cat_dog.tar.gz
cd jetson-inference/python/training/classification
python3 train.py --model-dir=models/cat_dog data/cat_dog
python3 onnx_export.py --model-dir=models/cat_dog
NET=models/cat_dog
DATASET=data/cat_dog
imagenet --model=$NET/resnet18.onnx --input_blob=input_0 --output_blob=output_0 --labels=$DATASET/labels.txt $DATASET/test/cat/01.jpg cat.jpg
```
5. Install the Jetson Inference ROS Wrapper
```
cd f1tenth_ws/src
sudo apt-get install ros-foxy-vision-msgs
git clone https://github.com/dusty-nv/ros_deep_learning
cd ..
colcon build
```
Test via:
```
source install/setup.bash
ros2 launch ros_deep_learning detectnet.ros2.launch input:=v4l2:///dev/video0 output:=display://0
```

## Install Range_Libc for the Particle Filter
```
sudo pip3 install cython transforms3d
git clone https://github.com/f1tenth/range_libc.git
cd range_libc
mkdir build
cd build
cmake ..
make
cd ~/range_libc/pywrapper
sudo WITH_CUDA=ON python3 setup.py install #for python3.8
```

## F1TENTH_Waterloo Workspace
```
git clone https://github.com/fjahncke/f1tenth_ws_waterloo.git
sudo apt install ros-foxy-tf-transformations ros-foxy-navigation2
cd f1tenth_ws_waterloo
colcon build --symlink-install
source install/setup.bash
export PYTHONPATH=${PYTHONPATH}:/usr/lib/python3.8/site-packages/range_libc-0.1-py3.8-linux-aarch64.egg
colcon build --symlink-install
```
Test the particle filter:
```
cd f1tenth_ws_waterloo
source install/setup.bash
ros2 launch particle_filter localize_launch.py
```
Test the Pure pursuit node in another terminal at the same time:
```
cd f1tenth_ws_waterloo
source install/setup.bash
ros2 launch pure_pursuit pure_pursuit_launch.py
```
