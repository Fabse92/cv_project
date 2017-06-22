#include "ros/ros.h"
#include "candidate_locator/LocateCandidates.h"
#include "candidate_locator/ArrayPointClouds.h"


bool locate_candidates(
	candidate_locator::LocateCandidates::Request  &req,
  candidate_locator::LocateCandidates::Response &res)
{
  ROS_INFO("In locate_candidates");

	return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "candidate_locator_server");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("locate_candidates", locate_candidates);
  
  ROS_INFO("Ready to locate candidates");
  ros::spin();

  return 0;
}