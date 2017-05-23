#include "ros/ros.h"
#include "object_candidates/Snapshot.h"
#include "object_candidates/SnapshotMsg.h"
#include "candidate_locator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snapshot_client");
  
  ros::NodeHandle nh;
  
  // image_transport::ImageTransport it = image_transport::ImageTransport(nh);
  ros::Publisher pub = nh.advertise<object_candidates::SnapshotMsg>("/candidates_snapshot",1000);
  
  ros::ServiceClient snapshot_client = nh.serviceClient<object_candidates::Snapshot>("get_snapshot");
  object_candidates::Snapshot srv;
  
  ROS_INFO("Requesting snapshot");
  if (snapshot_client.call(srv))
  {
    ROS_INFO("Received snapshot");

    object_candidates::SnapshotMsg snapshot;
    snapshot.depth_image = srv.response.depth_image;
    snapshot.rgb_image = srv.response.rgb_image;
    snapshot.rgb_info = srv.response.rgb_info;
    snapshot.candidates = srv.response.candidates;

    pub.publish(snapshot);

    ROS_INFO("Snapshot published.");
  }
  else
  {
    ROS_ERROR("Failed to call snapshot service");
  }

  return 0;
}



