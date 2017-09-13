#include "ros/ros.h"
#include "object_candidates/Snapshot.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "object_candidates/ArrayImages.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snapshot_client");
  
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);
  image_transport::Publisher image_pub = it.advertise("/candidate",1000);
  
  ros::ServiceClient snapshot_client = nh.serviceClient<object_candidates::Snapshot>("get_snapshot");
  object_candidates::Snapshot srv;
  
  ROS_INFO("Requesting snapshot");
  if (snapshot_client.call(srv))
  {
    ROS_INFO("Received snapshot");
    for (unsigned int i = 0; i < srv.response.candidates.data.size(); ++i){
      image_pub.publish(srv.response.candidates.data[i]);
      ros::WallDuration(3).sleep();
    }
  }
  else
  {
    ROS_ERROR("Failed to call snapshot service");
  }

  return 0;
}



