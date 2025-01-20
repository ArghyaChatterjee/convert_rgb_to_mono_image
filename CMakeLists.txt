cmake_minimum_required(VERSION 3.0.2)
project(convert_rgb_to_mono_image)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## Specify dependencies required by this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES convert_rgb_to_mono_image
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for bgra_to_mono_node
add_executable(bgra_to_mono_node src/bgra_to_mono_node.cpp)
add_dependencies(bgra_to_mono_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bgra_to_mono_node
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for rgb_to_mono_node
add_executable(rgb_to_mono_node src/rgb_to_mono_node.cpp)
add_dependencies(rgb_to_mono_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rgb_to_mono_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS bgra_to_mono_node rgb_to_mono_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
