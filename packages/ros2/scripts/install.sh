#!/usr/bin/env bash
set -euo pipefail

# Install ros2 and packages

sudo apt-get update && sudo apt-get upgrade -y

sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

PKGS=(
    cmake
    build-essential
    python3-colcon-common-extensions
    libpcl-dev
    libeigen3-dev
    ros-humble-ros-base
    ros-humble-usb-cam
    ros-humble-robot-localization
    ros-humble-ublox-dgnss
    ros-humble-tf2-ros
    ros-humble-nav2-bringup
    ros-humble-rviz2
    ros-humble-rviz-default-plugins
    ros-humble-rviz-common
)

sudo apt-get install -y "${PKGS[@]}"

# Build OpenCV

wget -O opencv.zip https://github.com/opencv/opencv/archive/4.10.0.zip
unzip opencv.zip

wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.10.0.zip
unzip opencv_contrib.zip

mkdir -p build && cd build
cmake ../opencv-4.10.0 \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENCV_EXTRA_MODULES_PATH="../opencv_contrib-4.10.0/modules" \
  -DCMAKE_INSTALL_PREFIX="/usr/local" \
  -DPYTHON2_EXECUTABLE="" \
  -DPYTHON2_INCLUDE_DIR="" \
  -DPYTHON2_LIBRARY="" \
  -DPYTHON2_NUMPY_INCLUDE_DIRS="" \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=OFF \
  -DWITH_CUDA=ON \
  -DWITH_CUDNN=ON \
  -DWITH_CUBLAS=ON \
  -DOPENCV_DNN_CUDA=ON \
  -DENABLE_FAST_MATH=ON \
  -DCUDA_FAST_MATH=ON \
  -DCUDA_ARCH_BIN="8.7" \
  -DCUDA_ARCH_PTX="" \
  -DWITH_NVCUVID=ON \
  -DWITH_NVCUVENC=ON \
  -DBUILD_opencv_cudacodec=ON \
  -DBUILD_opencv_cudaarithm=ON \
  -DBUILD_opencv_cudaimgproc=ON \
  -DBUILD_opencv_cudawarping=ON \
  -DBUILD_opencv_cudafilters=ON \
  -DBUILD_opencv_viz=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DOPENCV_ENABLE_NONFREE=OFF \
  -DWITH_OPENVINO=OFF \
  -DWITH_GTK=ON \
  -DWITH_V4L=ON \
  -DWITH_GSTREAMER=ON
cmake --build . -j$(nproc)

sudo make install
sudo ldconfig

cd ..

rm -rf opencv-4.5.0
sudo rm -rf build
rm -rf opencv_contrib
rm opencv.zip

# Set up workspace

[[ "$(pwd)" == */ros2 ]] || { echo "Error: must be in ros2 workspace (ucf-act/packages/ros2)"; exit 1; }

export ACT_ROS_WS="$(pwd)"
echo "export ACT_ROS_WS=$ACT_ROS_WS" >> ~/.bashrc

source /opt/ros/humble/setup.bash

# Set up bringup package

chmod +x src/bringup/hooks/dotenv.sh

# Install livox driver

git clone https://github.com/DRobinson4105/livox_ros_driver2.git src/livox_ros_driver2
cd src/livox_ros_driver2
./build.sh humble
cd ../..

# Install fast lio

git clone https://github.com/DRobinson4105/FAST_LIO_ROS2.git src/FAST_LIO_ROS2 --recursive

# Install perception models

mkdir -p src/perception/models
wget -O src/perception/models/seg.onnx https://huggingface.co/DavidRobinson05/ddrnet23slim_ucf/resolve/main/seg.onnx
/usr/src/tensorrt/bin/trtexec \
  --onnx=src/perception/models/seg.onnx \
  --saveEngine=src/perception/models/seg.engine \
  --fp16

# Build workspace

chmod +x ./scripts/build.sh
./scripts/build.sh

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ${ACT_ROS_WS}/install/setup.bash" >> ~/.bashrc
