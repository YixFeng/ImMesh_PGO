# ImMesh_PGO: An Immediate LiDAR Localization and Meshing Framework with Loop Closure

## TODO List
- Once, I met `corrupted double-linked list` error, most likely due to multiple threads randomly accessing a container at the same time, however no mutex was used.

## 1. Prerequisites
### 1.1 **ROS**
Following this [ROS Installation](http://wiki.ros.org/ROS/Installation) to install ROS and its additional pacakge:<br>
```
sudo apt-get install ros-XXX-cv-bridge ros-XXX-tf ros-XXX-message-filters ros-XXX-image-transport ros-XXX-image-transport*
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-noetic, the command should be:<br>
```
sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters ros-noetic-image-transport*
```
### 1.2. **livox_ros_driver**
Follow this [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

### 1.3 **CGAL** and **OpenGL**
```
sudo apt-get install -y libcgal-dev pcl-tools
sudo apt-get install -y libgl-dev libglm-dev libglfw3-dev libglew-dev libglw1-mesa-dev 
sudo apt-get install -y libcgal-dev libxkbcommon-x11-dev
```

### 1.4 **GTSAM**
Follow this [GTSAM Get Started](https://gtsam.org/get_started/).

## 2. Build ImMesh on ROS:
Clone this repository and catkin_make:
```
cd ~/catkin_ws/src
git clone https://github.com/YixFeng/ImMesh_PGO.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
