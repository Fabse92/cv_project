#include <ros/ros.h>
#include "candidate_locator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "candidate_locator");

  CandidateLocator candidate_locator;

  ros::spin();

  return 0;
};