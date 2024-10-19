# ImMesh_PGO: An Immediate LiDAR Localization and Meshing Framework with Loop Closure

## 1. Prerequisites
### 1.1 **ROS**
Following this [ROS Installation](http://wiki.ros.org/ROS/Installation) to install ROS and its additional pacakge:<br>
```
sudo apt-get install ros-XXX-cv-bridge ros-XXX-tf ros-XXX-message-filters ros-XXX-image-transport ros-XXX-image-transport*
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-kinetic, the command should be:<br>
```
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport*
```
### 1.2. **livox_ros_driver**
Follow this [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

### 1.3 **CGAL** and **OpenGL**
```
sudo apt-get install -y libcgal-dev pcl-tools
sudo apt-get install -y libgl-dev libglm-dev libglfw3-dev libglew-dev libglw1-mesa-dev 
sudo apt-get install -y libcgal-dev libxkbcommon-x11-dev
```
## 2. Build ImMesh on ROS:
Clone this repository and catkin_make:
```
cd ~/catkin_ws/src
git clone https://github.com/YixFeng/ImMesh_PGO
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Related works

ImMesh is building upon the foundations of our previous SLAM works including  [R3LIVE](https://github.com/hku-mars/r3live), [VoxelMap](https://github.com/hku-mars/VoxelMap), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [R2LIVE](https://github.com/hku-mars/r2live), and [ikd-Tree](https://github.com/hku-mars/ikd-Tree). These works are all available on our GitHub, as listed below:

1. [R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package](https://github.com/hku-mars/r3live)
2. [VoxelMap: An efficient and probabilistic adaptive(coarse-to-fine) voxel mapping method for 3D LiDAR](https://github.com/hku-mars/VoxelMap)
3. [FAST-LIO: A computationally efficient and robust LiDAR-inertial odometry (LIO) package](https://github.com/hku-mars/FAST_LIO)
4. [R2LIVE: a robust, real-time tightly-coupled multi-sensor fusion framework](https://github.com/hku-mars/r2live)
5. [ikd-Tree: an incremental k-d tree designed for robotic applications](https://github.com/hku-mars/ikd-Tree) 
