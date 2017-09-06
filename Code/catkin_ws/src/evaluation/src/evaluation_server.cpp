#include "ros/ros.h"

#include "evaluation/Evaluate.h"
#include <sensor_msgs/image_encodings.h>


bool evaluate(evaluation::Evaluate::Request  &req,
         evaluation::Evaluate::Response &res)
{

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluation_server");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("evaluate", evaluate);
  
  ROS_INFO("Ready to evaluate");
  ros::spin();

  return 0;
}









