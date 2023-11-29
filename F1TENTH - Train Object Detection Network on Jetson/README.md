## Instructions to train an object detection network directly on an NVIDIA Jetson Orin Nano


1. Increase the Swap Size to 16GB according to this repo: [JetsonHacks Swap Size](https://github.com/JetsonHacksNano/resizeSwapMemory "JetsonHacks Swap Size")
2. Install Tensorflow for Jetson for Tensorboard: [Install Tensorflow on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html "Install Tensorflow on Jetson")
3. Install PyTorch with CUDA bindings on Jetson: [Install PyTorch on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html "Install PyTorch on Jetson")
4. Verify the functionality of the install by running the following commands in a terminal:
- "python3"
- "import torch"
- "print(torch.cuda.is_available())"
If the command prints true at the end, Torch can use the GPU.

5. In a terminal: "export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so"
6. python3 train_ssd.py --dataset-type=voc --data=data/cone_dataset --model-dir=models/mobilenetv2 --batch-size=4 --workers=0 --epochs=10
7. python3 onnx_export.py --model-dir=models/mobilenetv2
8. detectnet --model=models/mobilenetv2/ssd-mobilenet.onnx --labels=models/cone_dataset/labels.txt \
          --input-blob=input_0 --output-cvg=scores --output-bbox=boxes \
            /dev/video0
