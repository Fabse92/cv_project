#include "ros/ros.h"
#include "object_candidates/Snapshot.h"
#include "object_candidates/ArrayImages.h"
#include "object_candidates/Objectcandidates.h"

#include <sensor_msgs/image_encodings.h>


bool snapshot(object_candidates::Snapshot::Request  &req,
         object_candidates::Snapshot::Response &res)
{
  sensor_msgs::Image rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>("/kinect2/sd/image_color_rect"));  
  res.rgb_image = rgb_image;
  res.depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>("/kinect2/sd/image_depth_rect"));
  
  res.rgb_info = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect2/sd/camera_info"));
  
  ROS_INFO("snapshot produced and given as response");
  
  ros::NodeHandle n;
  ros::ServiceClient object_candidates_client = n.serviceClient<object_candidates::Objectcandidates>("get_object_candidates");
  object_candidates::Objectcandidates srv;
  srv.request.rgb_image = rgb_image;
  
  ROS_INFO("Requesting object candidates");
  if (object_candidates_client.call(srv))
  {
    ROS_INFO("Received object candidates");
    res.candidates = srv.response.candidates;
  }
  else
  {
    ROS_ERROR("Failed to call object candidates service");
  }
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snapshot_server");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("get_snapshot", snapshot);
  
  ROS_INFO("Ready to serve snapshots");
  ros::spin();

  return 0;
}









