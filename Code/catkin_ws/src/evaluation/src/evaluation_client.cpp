#include "ros/ros.h"
#include "evaluation/Evaluate.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluation_client");
  
  ros::NodeHandle nh;
  
  ros::ServiceClient evaluation_client = nh.serviceClient<evaluation::Evaluate>("evaluate");
  evaluation::Evaluate eval_srv;
  eval_srv.request.restart.data = true;

  ROS_INFO("Requesting evaluation");
  if (evaluation_client.call(eval_srv))
  {
    ROS_INFO_STREAM("Evaluation performed!");
  }

  return 0;
}



