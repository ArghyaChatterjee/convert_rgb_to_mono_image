# Convert RGB to Monocular Image

A ros package to convert rgb/bgra8 image to monocular image. This is a ros1 package.

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
Modify the `bgra_to_mono.launch` file according to the `topics` and `frame name` that you want to publish the monocular image data of type `mono8` from color image of type `bgra8`. Then launch the node like this:
```bash
roslaunch convert_bgra_to_mono_image bgra_to_mono.launch
```
There is another launch file for zed camera for converting `rgb8` type color image to `mono8` type monocular image. If you want to test it, launch it like this:
```bash
roslaunch convert_rgb_to_mono_image rgb_to_mono.launch
```
<div align="center">
   <img src="media/zed_convert_rgb_to_monocular_image.gif"/>
</div>

## For bgra8 to mono8 conversion: 

Input Right camera rgb topic echo:
```
$ rostopic echo /zedm/zed_node/right/image_rect_color
header: 
  seq: 0
  stamp: 
    secs: 1729878717
    nsecs: 392199039
  frame_id: "zedm_right_camera_optical_frame"
height: 1080
width: 1920
encoding: "bgra8"
is_bigendian: 0
step: 7680
data: [181, 204, 212, 255, 181, ..]
```

Output Right camera mono topic echo:
```
$ rostopic echo /zedm/zed_node/right/image_rect_gray
header: 
  seq: 1044
  stamp: 
    secs: 1729878741
    nsecs: 844533920
  frame_id: "zedm_right_camera_optical_frame"
height: 1080
width: 1920
encoding: "mono8"
is_bigendian: 0
step: 1920
data: [247, 247, 247, 247, 247, ..]
```

## For rgb8 to mono8 conversion:

Input Right camera rgb topic echo:
```
$ rostopic echo /zedm/zed_node/right/image_rect_color
header: 
  seq: 0
  stamp: 
    secs: 1729878717
    nsecs: 392199039
  frame_id: "zedm_right_camera_optical_frame"
height: 1080
width: 1920
encoding: "rgb8"
is_bigendian: 0
step: 7680
data: [181, 204, 212, 255, 181, ..]
```

Output Right camera mono topic echo:
```
$ rostopic echo /zedm/zed_node/right/image_rect_gray
header: 
  seq: 1044
  stamp: 
    secs: 1729878741
    nsecs: 844533920
  frame_id: "zedm_right_camera_optical_frame"
height: 1080
width: 1920
encoding: "mono8"
is_bigendian: 0
step: 1920
data: [247, 247, 247, 247, 247, ..]
```