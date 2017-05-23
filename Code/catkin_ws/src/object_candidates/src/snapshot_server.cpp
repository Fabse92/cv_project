#include "ros/ros.h"
#include "object_candidates/Snapshot.h"
#include "object_candidates/ArrayImages.h"
#include "object_candidates/Objectcandidates.h"

#include <sensor_msgs/image_encodings.h>


bool snapshot(object_candidates::Snapshot::Request  &req,
         object_candidates::Snapshot::Response &res)
{
  sensor_msgs::Image rgb_image = *(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw"));  
  res.rgb_image = rgb_image;
  res.depth_image = *(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/depth/image_raw"));
  
  res.rgb_info = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/rgb/camera_info"));
  //res.depth_info = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info"));
  
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

/*
void rgbCb(const sensor_msgs::ImageConstPtr& msg)
{
   rgb_image = msg;
}

void depthCb(const sensor_msgs::ImageConstPtr& msg)
{
   depth_image = msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "snapshot_server");
  ros::NodeHandle nh;
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);
  image_transport::Subscriber rgb_sub, depth_sub;

  rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, rgbCb);
  depth_sub = it.subscribe("/camera/depth/image_raw", 1, depthCb);
  
  ros::ServiceServer service = nh.advertiseService("get_snapshot", snapshot);
  
  ROS_INFO("Ready to serve snapshots");
  ros::spin();

  return 0;
}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snapshot_server");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("get_snapshot", snapshot);
  
  ROS_INFO("Ready to serve snapshots");
  ros::spin();

  return 0;
}









