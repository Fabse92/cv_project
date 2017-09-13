#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <boost/foreach.hpp>


int main(int argc, char** argv){
  ros::init(argc, argv, "cheater_tf_broadcaster");
  ros::NodeHandle nh;  

  ros::ServiceClient gls_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  gls_client.waitForExistence();
 
  tf::TransformBroadcaster br; 
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  while (nh.ok()){
  
    tf::Transform real_base_link, real_camera;
    tf::StampedTransform base_link_camera;
    gazebo_msgs::GetLinkState getlinkstate;
    
    getlinkstate.request.link_name = "p3dx::base_link";
    gls_client.call(getlinkstate);
    
    real_base_link.setOrigin( tf::Vector3(getlinkstate.response.link_state.pose.position.x, getlinkstate.response.link_state.pose.position.y, getlinkstate.response.link_state.pose.position.z) );
    
    real_base_link.setRotation( tf::Quaternion(getlinkstate.response.link_state.pose.orientation.x, getlinkstate.response.link_state.pose.orientation.y, getlinkstate.response.link_state.pose.orientation.z, getlinkstate.response.link_state.pose.orientation.w) );
    
    try{
      listener.lookupTransform("/base_link", "/camera_depth_frame",  
                               ros::Time(0), base_link_camera);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }  
    
    real_camera = real_base_link * base_link_camera;

    br.sendTransform(tf::StampedTransform(real_camera, ros::Time::now(), "/map", "/real_camera_pose"));
    rate.sleep();
  }
  return 0;
};

