#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <boost/foreach.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "ground_truth_tf_broadcaster");
  ros::NodeHandle nh;
  
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

  ros::ServiceClient gms_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState getmodelstate;
  gms_client.waitForExistence();  
 
  std::vector<tf::Transform> transforms;
  
  int idx = 0;
  
  BOOST_FOREACH(std::string object, objects){  
    getmodelstate.request.model_name = object;
    
    gms_client.call(getmodelstate);
    tf::Transform transform;
    //transform.setOrigin( tf::Vector3(1.5, 0, 0));
    //tf::Quaternion rotation;
    //rotation.setRPY(1.5708, -0,0);
    //transform.setRotation(rotation);
    transform.setOrigin( tf::Vector3(getmodelstate.response.pose.position.x, getmodelstate.response.pose.position.y, getmodelstate.response.pose.position.z) );
    transform.setRotation( tf::Quaternion(getmodelstate.response.pose.orientation.x, getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z, getmodelstate.response.pose.orientation.w) );
    
    if (object == "power_drill")
      transform.getOrigin().setZ(transform.getOrigin().getZ() - 0.08); 
    if (object == "cracker_box")
      transform.getOrigin().setZ(transform.getOrigin().getZ() - 0.025);  
    if (object == "hammer"){
      transform.getOrigin().setZ(transform.getOrigin().getZ() + 0.062); 
      transform.getOrigin().setX(transform.getOrigin().getX() - 0.048);
      transform.getOrigin().setY(transform.getOrigin().getY() + 0.025);
    }
    
    transforms.push_back(transform);
    ++idx;
  }

  tf::TransformBroadcaster br; 
  ros::Rate rate(1.0);
  while (nh.ok()){
    int idx = 0;
    BOOST_FOREACH(std::string object, objects){  
      br.sendTransform(tf::StampedTransform(transforms[idx], ros::Time::now(), "/map", object));
      ++idx;
    }
    rate.sleep();
  }
  return 0;
};

