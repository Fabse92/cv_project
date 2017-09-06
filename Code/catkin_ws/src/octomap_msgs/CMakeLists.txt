cmake_minimum_required(VERSION 2.8)
project(octomap_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs sensor_msgs candidate_locator)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES Octomap.msg OctomapWithPose.msg
  )

add_service_files(
  DIRECTORY srv
  FILES GetOctomap.srv BoundingBoxQuery.srv MergeCandidates.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs candidate_locator)

catkin_package(DEPENDS message_runtime geometry_msgs std_msgs candidate_locator
               INCLUDE_DIRS include
)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
