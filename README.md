# [C3P-VoxelMAP](https://github.com/deptrum/c3p-voxelmap/) converter to [HDMapping](https://github.com/MapsHD/HDMapping)

## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-c3p-voxelmap-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.  

## Intended use 

This small toolset allows to integrate SLAM solution provided by [C3P-VoxelMap](https://github.com/deptrum/c3p-voxelmap/) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 1 workspace that :
  - submodule to tested revision of C3P-VoxelMap
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependencies

```shell
sudo apt install -y nlohmann-json3-dev
```

## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/MapsHD/benchmark-C3P-VoxelMap-to-HDMapping.git --recursive
cd ..
catkin_make
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
rosbag record /cloud_registered /aft_mapped_to_init
```

and start odometry:
```shell 
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch c3p_voxelmap mapping_velodyne.launch
rosbag play *.bag --clock
```

## Usage - conversion:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun c3p-voxelmap-to-hdmapping listener <recorded_bag> <output_dir>
```

## Record the bag file:

```shell
rosbag record /cloud_registered /aft_mapped_to_init
```

## VoxelMap Launch:

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
roslaunch c3p_voxelmap mapping_velodyne.launch
rosbag play {path_to_bag} --clock pp_points/synced2rgb:=/velodyne_points
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
rosrun c3p-voxelmap-to-hdmapping listener <recorded_bag> <output_dir>
```
