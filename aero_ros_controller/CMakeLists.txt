cmake_minimum_required(VERSION 2.8.3)
project(aero_ros_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  joint_limits_interface
  angles
  urdf
  aero_startup
#  pluginlib
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    hardware_interface
    transmission_interface
    joint_limits_interface
    angles
    urdf
    aero_startup
#    pluginlib
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME} default_robot_hw_sim
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Libraries
add_library(universal_robot_hw_shm src/universal_robot_hw_shm.cpp )
target_link_libraries(universal_robot_hw_shm ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/aero_ros_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} universal_robot_hw_shm)

## Install
install(TARGETS ${PROJECT_NAME} universal_robot_hw_shm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(FILES robot_hw_sim_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
