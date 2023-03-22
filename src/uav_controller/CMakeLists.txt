cmake_minimum_required(VERSION 3.0.2)
project(uav_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
  mavros_msgs
  airsim_ros_pkgs
)


catkin_package(
 # INCLUDE_DIRS include
 LIBRARIES uav_controller
 # CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
)

include_directories(
  # include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  # ${PROJECT_SOURCE_DIR}/include
)


add_executable(uav_controller_node
  src/uav_controller_node.cpp 
  )
target_link_libraries(uav_controller_node 
  ${catkin_LIBRARIES}
  )

catkin_install_python(PROGRAMS 
  scripts/uav_controller.py
  scripts/main_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)