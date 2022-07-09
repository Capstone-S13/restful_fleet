cmake_minimum_required(VERSION 3.0.2)
project(restful_fleet_client)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET COMPONENTS
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    actionlib
    move_base_msgs
    )

    if (catkin_FOUND)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES restful_client
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

file(GLOB python_scripts_DIR
    "scripts/*.py"
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


catkin_install_python(PROGRAMS
    scripts/main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

else()
  message("catkin not found so skipping this ROS1 package")
endif()