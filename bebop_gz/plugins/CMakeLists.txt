cmake_minimum_required(VERSION 3.10)
project(GazeboPlugins)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(ignition-math6 REQUIRED)
find_package(ignition-common4 REQUIRED)

add_library(SetPosePlugin SHARED SetPosePlugin.cc)
target_link_libraries(SetPosePlugin
  ignition-gazebo6::ignition-gazebo6
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
  ignition-plugin1::ignition-plugin1
  ignition-math6::ignition-math6
  ignition-common4::ignition-common4
)

add_library(RobotPosePublisher SHARED RobotPosePublisher.cc)
target_link_libraries(RobotPosePublisher
  ignition-gazebo6::ignition-gazebo6
  ignition-transport11::ignition-transport11
  ignition-plugin1::ignition-plugin1
  ignition-math6::ignition-math6
  ignition-common4::ignition-common4
)

install(TARGETS SetPosePlugin RobotPosePublisher DESTINATION lib)
