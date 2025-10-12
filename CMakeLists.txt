cmake_minimum_required(VERSION 3.0.2)
project(wds_snapshot_client)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  pybind11_catkin
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs pybind11_catkin
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Add the Pybind11 module target using pybind11_catkin
## This automatically handles Python detection
pybind11_add_module(py_csv_logger
  src/csv_logger.cpp
  src/csv_logger_py.cpp
)

## Link libraries
target_link_libraries(py_csv_logger PRIVATE
  ${catkin_LIBRARIES}
)

## Set properties
set_target_properties(py_csv_logger PROPERTIES
  PREFIX ""
  SUFFIX ".so"
)

#############
## Install ##
#############

## Mark executable scripts for installation
catkin_install_python(PROGRAMS
  script/snapshot_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install the pybind11 module
install(TARGETS py_csv_logger
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)