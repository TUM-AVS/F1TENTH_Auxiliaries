# Instructions to set up an NVIDIA Jetson Orin Nano for all TUM F1TENTH Labs using a SD Card.
## General Setup
0. If you want to resetup and SD Card, save all important data from the previous SD Card image.
1. Flash an SD Card with the correct Jetpack Version (currently 5.1.2 [Jetpack 5.1.2 Download](https://developer.nvidia.com/embedded/jetpack-sdk-512)) using, e.g., Balena Etcher: [Balena Etcher Download](https://etcher.balena.io).
2. Insert the SD Card, power up the Jetson and set up the user (username: f1tenth, password: f1tenth).
3. Alternative flash the SSD with the SDK Manager from NVidia: [NVidia SDK Manager](https://developer.nvidia.com/sdk-manager)
4. Update and Upgrade the software:
```
sudo apt-get update && sudo apt-get upgrade
```
7. Install the Wifi driver according to the installation instructions
   1. Clone the repository
```
git clone https://github.com/RinCat/RTL88x2BU-Linux-Driver.git
```
   2. Navigate inside the directory and build the module
```
cd RTL88x2BU-Linux-Driver && make clean && make && sudo make install
```
8. Insert the Bluetooth Adapter and Wifi Adapter and check successful connections
9. For usage at TUM in the eduroam Wifi, please log into Eduroam using the official python script you can download [here](https://cat.eduroam.org)
11. Install X11VNC
```
sudo apt-get install x11vnc
```
12. Set the X11 password to f1tenth and save it at the default location
```
x11vnc -storepasswd
```



## Install ROS with the base F1TENTH software stack
10. [Install ROS](https://docs.ros.org/en/foxy/Installation.html)
11. [Install the F1TENTH stack](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#doc-drive-workspace). Use "rosdep update --include-eol-distros"!
13. Add a line to the bashrc file so no car communicates with each other over Wifi.
14. Open the bash script with:
```
sudo nano ~/.bashrc
```
15. Enter in the last line the following command, save and then close the file
```
export ROS_LOCALHOST_ONLY=1
export /opt/ros/foxy/setup.bash
```
10. [Install Zerotier](https://www.zerotier.com/download/) and add the car to the correct network by replacing <network_id> with the correct parameter
```
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join <network_id>
```

## Install Object Detection Training and Inference on the Jetson
16. Install the corresponding ZED SDK from here: [ZED SDK Download](https://www.stereolabs.com/developers/release). The installation instructions are here: [ZED SDK Installation Instructions](https://www.stereolabs.com/docs/installation/jetson)). Enter yes to all options until it wants to download the AI modules. Enter no there.
```
cd Downloads && chmod +x ZED_SDK* && ./ZED_SDK*
```
18. Add the ZED ROS2 Wrapper to the f1tenth_ws workspace and install according to the official repository: [ZED ROS2 Wrapper Installation Instructions](https://github.com/stereolabs/zed-ros2-wrapper)
19. Increase the Swap Size to 16GB according to this repo: [JetsonHacks Swap Size](https://github.com/JetsonHacksNano/resizeSwapMemory "JetsonHacks Swap Size")
20. Install Tensorflow for Jetson for Tensorboard: [Install Tensorflow on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html "Install Tensorflow on Jetson")
```
sudo apt-get update
sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install python3-pip
sudo python3 -m pip install --upgrade pip
sudo pip3 install -U testresources setuptools==65.5.0
sudo pip3 install -U numpy==1.22 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 protobuf pybind11 pkgconfig packaging h5py==3.7.0
sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v512 tensorflow==2.12.0+nv23.06
```
22. Install PyTorch with CUDA bindings on Jetson: [Install PyTorch on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html "Install PyTorch on Jetson")
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
24. Verify the functionality of the install by running the following commands in a terminal:d
- "python3"
- "import torch"
- "print(torch.cuda.is_available())"
- "exit()"
If the command prints true at the end, Torch can use the GPU.
1. Install Torchvision with [Install Torchvision on Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
Look up the torchvision version corresponding to the PyTorch Version
and insert it as e.g. v0.16.1 in <version> and 0.16.1 for <build_version>. The installation takes quite some time.
```
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch <version> https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=<build_version>
python3 setup.py install --user
```
Test the success with:
```
python3
import torchvision
exit()
```
15. In a terminal: "export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so"
16. python3 train_ssd.py --dataset-type=voc --data=data/cone_dataset --model-dir=models/mobilenetv2 --batch-size=4 --workers=0 --epochs=10
17. python3 onnx_export.py --model-dir=models/mobilenetv2
18. detectnet --model=models/mobilenetv2/ssd-mobilenet.onnx --labels=models/cone_dataset/labels.txt \
          --input-blob=input_0 --output-cvg=scores --output-bbox=boxes \
            /dev/video0
19. sudo pip3 install cython transforms3d
20. git clone https://github.com/f1tenth/range_libc.git
21. cd range_libc
22. mkdir build
23. cd build
24. cmake ..
25. make
25. cd ~/range_libc/pywrapper
26. sudo WITH_CUDA=ON python3 setup.py install #for python3.8
27. git clone https://github.com/fjahncke/f1tenth_ws_waterloo.git
28. sudo apt install ros-foxy-tf-transformations ros-foxy-navigation2
29. cd f1tenth_ws_waterloo
30. colcon build --symlink-install
31. source install/setup.bash
32. export PYTHONPATH=${PYTHONPATH}:/usr/lib/python3.8/site-packages/range_libc-0.1-py3.8-linux-aarch64.egg
33. colcon build --symlink-install
34. source install/setup.bash
35. ros2 launch particle_filter localize_launch.py
36. In another terminal, source and launch the Pure Pursuit:
37. ros2 launch pure_pursuit pure_pursuit_launch.py
