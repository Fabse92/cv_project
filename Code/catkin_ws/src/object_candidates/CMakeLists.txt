cmake_minimum_required(VERSION 2.8.3)
project(object_candidates)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

find_package(Boost COMPONENTS serialization REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

link_directories(
  ${Boost_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_message_files(
  FILES
  ArrayImages.msg
  SnapshotMsg.msg
)

add_service_files(
  FILES
  Snapshot.srv
  Objectcandidates.srv
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

add_executable(object_candidates_node
  src/object_candidates_node.cpp
  src/VOCUS2.cpp
  src/generate_object_candidates.cpp
  src/egbis.cpp
  src/egbis/filter.cpp
  src/egbis/misc.cpp
  src/egbis/segment-graph.cpp
  src/egbis/segment-image.cpp
  src/egbis/disjoint-set.cpp
)

add_dependencies(object_candidates_node ${object_candidates_EXPORTED_TARGETS})

target_link_libraries(object_candidates_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(listener
  src/listener_node.cpp
  src/listener.cpp
)

add_dependencies(listener ${object_candidates_EXPORTED_TARGETS})

target_link_libraries(listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(snapshot_server src/snapshot_server.cpp)
target_link_libraries(snapshot_server ${catkin_LIBRARIES})
add_dependencies(snapshot_server ${object_candidates_EXPORTED_TARGETS})

add_executable(snapshot_client src/snapshot_client.cpp)
target_link_libraries(snapshot_client ${catkin_LIBRARIES})
add_dependencies(snapshot_client ${object_candidates_EXPORTED_TARGETS})

add_executable(object_candidates_server
  src/object_candidates_server.cpp
  src/VOCUS2.cpp
  src/egbis.cpp
  src/egbis/filter.cpp
  src/egbis/misc.cpp
  src/egbis/segment-graph.cpp
  src/egbis/segment-image.cpp
  src/egbis/disjoint-set.cpp
)

add_dependencies(object_candidates_server ${object_candidates_EXPORTED_TARGETS})

target_link_libraries(
  object_candidates_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

