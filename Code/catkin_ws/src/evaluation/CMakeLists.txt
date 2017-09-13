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
find_package(candidate_locator)

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
  CompareGroundTruthsToProposals.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  candidate_locator
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

add_executable(evaluation_client src/evaluation_client.cpp)
target_link_libraries(evaluation_client ${catkin_LIBRARIES})
add_dependencies(evaluation_client ${evaluation_EXPORTED_TARGETS})

add_executable(transform_publisher src/transform_publisher.cpp)
target_link_libraries(transform_publisher ${catkin_LIBRARIES})
add_dependencies(transform_publisher ${evaluation_EXPORTED_TARGETS})

add_executable(cheater src/cheater.cpp)
target_link_libraries(cheater ${catkin_LIBRARIES})
add_dependencies(cheater ${evaluation_EXPORTED_TARGETS})


