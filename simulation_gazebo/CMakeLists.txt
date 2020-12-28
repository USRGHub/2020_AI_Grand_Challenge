cmake_minimum_required(VERSION 2.8.3)
project(simulation_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()

# Build PX4 firmware with gazebo SITL

#add_custom_target(px4_firmware
#  ALL
#  COMMAND make px4_sitl_default
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/Firmware
#)
#add_custom_target(px4_firmware_sitl_gazebo
#  ALL
#  COMMAND make px4_sitl_default sitl_gazebo
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/Firmware
#)
#add_dependencies(px4_firmware_sitl_gazebo px4_firmware)
