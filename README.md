# convert_rgb_to_mono_image

A ros package to convert rgb/bgra8 image to monocular image. 

# Requirements
This repository is tested with Ubuntu 20.04 and ROS noetic.

# Setup the repo
Create a ros1 catkin workspace and clone the repo inside the workspace:
```
mkdir -p ros1_ws/src
git clone https://github.com/ArghyaChatterjee/convert_rgb_to_mono_image.git
```
# Build the package
This requires mostly standard ros messages which are already installed at the time of installing ros noetic. No additional packages are needed to be installed.
```
cd ~/ros1_ws
catkin build convert_rgb_to_mono_image
source devel/setup.bash
```

# Launch the conversion node
Modify the `bgra_to_mono.launch` file according to the `topics` and `frame name` that you want to publish the monocular image data. Then launch the node like this:
```bash
roslaunch convert_bgra_to_mono_image bgra_to_mono.launch
```
There is a separate launch file for zed camera. If you want to test it, launch it like this:
```bash
roslaunch tf_imu_to_base zedm_imu_transform.launch
```
<div align="center">
   <img src="media/zed_imu_frame_transformed.gif"/>
</div>

Input Left camera rgb topic echo:
```
$ rostopic echo /zed2/zed_node/left/image_rect_color
header: 
  seq: 4
  stamp: 
    secs: 1735278948
    nsecs: 606446465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
encoding: "bgra8"
is_bigendian: 0
step: 2560
data: [61, 34, 41, 255, 61, 36, 43, 255, 63, 37, 43, 255, 64, 40, 43, 255...]
```
Output Left camera mono topic echo:
```
$ rostopic echo /zed2/zed_node/left/image_rect_gray
header: 
  seq: 4
  stamp: 
    secs: 1735278948
    nsecs: 606446465
  frame_id: "zed2_left_camera_optical_frame"
height: 360
width: 640
encoding: "mono8"
is_bigendian: 0
step: 2560
data: [61, 34, 41, 255, 61, 36, 43, 255, 63, 37, 43, 255, 64, 40, 43, 255...]
```