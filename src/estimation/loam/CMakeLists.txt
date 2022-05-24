cmake_minimum_required(VERSION 2.8.3)
project(loam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  )
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
    src/loam.cpp
    src/loam_handle.cpp
    src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  )