cmake_minimum_required(VERSION 3.0.2)
project(environment_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin 
  REQUIRED 
  COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  
)


find_package(gazebo REQUIRED)

add_service_files(
  FILES
  DoorStatus.srv
  DoorOpen.srv 
  BoxPutDown.srv
  BoxPickUp.srv
  BoxPos.srv
  BoxUpdatePos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES environment_pkg
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
  DEPENDS 
    # system_lib 
    gazebo_ros
    message_runtime
)

###########
## Build ##
###########


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
# include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# for gazebo plugin
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")




add_library(update_box SHARED src/update_box.cpp)
target_link_libraries(update_box ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(update_door SHARED src/update_door.cpp)
target_link_libraries(update_door ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

add_dependencies(update_box ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(update_door ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")