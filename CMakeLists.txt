project(visual_odom)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

##############
## Packages ##
##############

## Catkin ##

find_package(catkin REQUIRED COMPONENTS
  roscpp
  viso2_ros
  libviso2
  manager_api
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES visual_odom
  CATKIN_DEPENDS roscpp viso2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Locations ##

include_directories(
  ${libviso2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/
)

## Libraries ##

#add_library(${PROJECT_NAME}
#  src/${PROJECT_NAME}/component_odom.cpp
#)

#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

## Executables ##

add_executable(visual_odom_node
  src/visual_odom_node.cpp
)

target_link_libraries(visual_odom_node
  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
)

