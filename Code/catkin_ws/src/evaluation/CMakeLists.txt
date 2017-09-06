cmake_minimum_required(VERSION 2.8.3)
project(evaluation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  pcl_ros
)

find_package(Boost COMPONENTS serialization REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

link_directories(
  ${Boost_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_service_files(
  FILES
  Evaluate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_candidates
  CATKIN_DEPENDS message_runtime cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

add_executable(evaluation_server src/evaluation_server.cpp)
target_link_libraries(evaluation_server ${catkin_LIBRARIES})
add_dependencies(evaluation_server ${evaluation_EXPORTED_TARGETS})



