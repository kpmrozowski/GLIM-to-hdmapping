# GLIM-to-HDMapping converter


## Intended use 

This small toolset allows to integrate SLAM solution provided by [GLIM](https://github.com/koide3/glim) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 2 workspace that :
  - submodule to tested revision of GLIM
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Building

Clone the repo
```shell
sudo apt install -y nlohmann-json3-dev clang-14 libomp-14-dev
git clone --recursive https://github.com/kpmrozowski/GLIM-to-hdmapping.git $HOME/GLIM-to-hdmapping
cd $HOME/GLIM-to-hdmapping
CXX=clang++-14 CC=clang-14 colcon build --event-handlers=console_direct+ \
   --cmake-args -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_WITH_CUDA_MULTIARCH=ON \
   -DBUILD_WITH_CUDA=ON -DBUILD_GTSAM_POINTS_GPU=ON -DGTSAM_POINTS_USE_CUDA=ON \
   -DBUILD_WITH_CUDA=ON -DBUILD_TESTS=ON -DBUILD_TESTS_PCL=ON -DBUILD_DEMO=ON \
   -DBUILD_EXAMPLE=ON -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
```

## Example usage - data SLAM:

Prepare recorded bag with estimated odometry:

### In first terminal:

1. download and convert bag fro mros1 to ros2:
```shell
mkdir $HOME/glim-datasets
wget https://zenodo.org/records/6864654/files/livox.bag -O $HOME/glim-datasets/livox.bag
python3 -m pip install rosbags
$HOME/.local/bin/rosbags-convert --src $HOME/glim-datasets/livox.bag --dst $HOME/glim-datasets/livox_bag
```
2. prepare GLIM config file
```shell
wget https://staff.aist.go.jp/k.koide/projects/glim_params/config_versatile.tar.gz -O $HOME/glim-datasets/config_versatile.tar.gz
tar -xf $HOME/glim-datasets/config_versatile.tar.gz -C  $HOME/glim-datasets/
```
3. start odometry:
```shell 
cd $HOME/GLIM-to-hdmapping
. ./install/setup.bash # adjust to used shell
ros2 run glim_ros glim_rosbag --ros-args -p config_path:=$HOME/glim-datasets/config/livox
```

### and in the second record bag:
```shell
ros2 bag record /glim_ros/points /glim_ros/odom -o $HOME/glim-datasets/result-livox_bag
```

### and in the third play bag:
```shell
ros2 bag play $HOME/glim-datasets/livox_bag/
```

after bag finished playing press CTRL+C in all 3 terminals

## Usage - conversion:

```shell
cd $HOME/GLIM-to-hdmapping
. ./install/setup.bash # adjust to used shell
ros2 run glim-to-hdmapping listener $HOME/glim-datasets/result-livox_bag $HOME/glim-datasets/sesja-livox
```
