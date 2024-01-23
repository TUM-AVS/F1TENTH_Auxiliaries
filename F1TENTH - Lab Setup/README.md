## Instructions to set up an NVIDIA Jetson Orin Nano for all TUM F1TENTH Labs using a SD Card.
0. If you want to resetup and SD Card, save all important data from the previous SD Card image.
1. Flash an SD Card with the correct Jetpack Version (currently 5.1.2 [Jetpack 5.1.2 Download](https://developer.nvidia.com/embedded/jetpack-sdk-512)) using, e.g., Balena Etcher: [Balena Etcher Download](https://etcher.balena.io).
2. Insert the SD Card, power up the Jetson and set up the user (username: f1tenth, password: f1tenth).
3. Run "sudo apt-get update" and "sudo apt-get upgrade"
4. Install the Wifi driver according to the installation instructions
5. Insert the Bluetooth Adapter and Wifi Adapter and check successful connections
6. [Install ROS](https://docs.ros.org/en/foxy/Installation.html)
7. [Install the F1TENTH stack](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#doc-drive-workspace). Use "rosdep update --include-eol-distros"!
8. [Install Zerotier](https://www.zerotier.com/download/)
9. Add a unique ID to the bashrc file so no car communicates with each other over Wifi. The ID is identical to the chassis number.
10. Open the bash script with "sudo nano ~/.bashrc" and in the last line enter for, e.g., chassis 7: "ROS_DOMAIN_ID=7".
11. Install the corresponding ZED SDK from here: [ZED SDK Download](https://www.stereolabs.com/developers/release). The installation instructions are here: [ZED SDK Installation Instructions](https://www.stereolabs.com/docs/installation/jetson)). Click yes to all options until it wants to download the AI modules. Click no there.
12. Add the ZED ROS2 Wrapper to the f1tenth_ws workspace and install according to the official repository: [ZED ROS2 Wrapper Installation Instructions](https://github.com/stereolabs/zed-ros2-wrapper)
13. Increase the Swap Size to 16GB according to this repo: [JetsonHacks Swap Size](https://github.com/JetsonHacksNano/resizeSwapMemory "JetsonHacks Swap Size")
14. Install PyTorch with CUDA bindings on Jetson: [Install PyTorch on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html "Install PyTorch on Jetson")
15. Verify the functionality of the install by running the following commands in a terminal:
- "python3"
- "import torch"
- "print(torch.cuda.is_available())"
If the command prints true at the end, Torch can use the GPU.
14. Install Tensorflow for Jetson for Tensorboard: [Install Tensorflow on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html "Install Tensorflow on Jetson")
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
