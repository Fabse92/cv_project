#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>

#include <evaluation/Evaluate.h>
#include <evaluation/CompareGroundTruthsToProposals.h>
#include <candidate_locator/ArrayPointClouds.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

#include <boost/foreach.hpp>

bool evaluate(evaluation::Evaluate::Request  &req,
         evaluation::Evaluate::Response &res)
{
  std::vector<std::string> objects;
  objects.push_back("ball");
  objects.push_back("banana");
  objects.push_back("bleach_cleanser");
  objects.push_back("bowl");
  objects.push_back("cracker_box");
  objects.push_back("hammer");
  objects.push_back("mug");
  objects.push_back("pitcher_base");
  objects.push_back("potted_meat_can");
  objects.push_back("power_drill");
  objects.push_back("sugar_box");
  objects.push_back("wood_block");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  
  std::string objects_package_path(ros::package::getPath("objects"));
  std::string fileName;
  candidate_locator::ArrayPointClouds array_pc_msg; 
  
  ros::NodeHandle nh;  
  ros::ServiceClient gms_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState getmodelstate;
  //ros::ServiceClient gms_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  //gazebo_msgs::GetLinkState getmodelstate;
  gms_client.waitForExistence();  
  
  geometry_msgs::TransformStamped transform;
  //transform.header.frame_id = 
  
  BOOST_FOREACH(std::string object, objects){  
    fileName = objects_package_path + "/data/"+object+"/"+object+"_downsampled.pcd";    
    reader.read (fileName, *cloud);  
    
    sensor_msgs::PointCloud2 pc_msg;    
    pcl::toROSMsg<pcl::PointXYZ>(*cloud,pc_msg); 
    
    getmodelstate.request.model_name = object;
    //getmodelstate.request.link_name = object+"_link";
    gms_client.call(getmodelstate);
    
    std::cout << getmodelstate.response.pose << std::endl;
    //std::cout << getmodelstate.response.link_state.pose << std::endl;
    
    transform.transform.translation.x = getmodelstate.response.pose.position.x;
    transform.transform.translation.y = getmodelstate.response.pose.position.y;
    transform.transform.translation.z = getmodelstate.response.pose.position.z;
    transform.transform.rotation = getmodelstate.response.pose.orientation;
    
    //tf2::doTransform (cloud_in, cloud_out, transform);
    tf2::doTransform(pc_msg, pc_msg, transform);
    
    array_pc_msg.data.push_back(pc_msg);
  }
  
  ros::ServiceClient comparison_client = nh.serviceClient<evaluation::CompareGroundTruthsToProposals>("compare_ground_truths_to_propsoals"); 
  comparison_client.waitForExistence();
  
  evaluation::CompareGroundTruthsToProposals comparison_srv;
  comparison_srv.request.ground_truths = array_pc_msg;
  
  ROS_INFO("Requesting to compare ground truths to proposals");
  if (comparison_client.call(comparison_srv))
  {
    ROS_INFO("Received statistics of comparison");
  }
  else
  {
    ROS_ERROR("Failed to compare ground truths to proposals");
  }
  
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









