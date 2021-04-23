cmake_minimum_required(VERSION 3.0.2)
project(manipulator_core)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    sensor_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
  FILES
  FloatArray.msg
  ArmCurrentPosition.msg
  ArmJointAngle.msg
  DoserStatus.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_install_python(PROGRAMS scripts/manipulator_1_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/manipulator_2_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


