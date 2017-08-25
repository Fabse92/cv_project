#include "ros/ros.h"
#include "candidate_locator.h"
#include "candidate_locator/LocateCandidates.h"

namespace candidate_locator
{
	class CandidateLocatorServer
	{
		private:

		CandidateLocator candidate_locator_;
		ros::Publisher pub_point_clouds_;
		ros::NodeHandle nh_;

		public:

	  CandidateLocatorServer()
	  {
	  	ros::ServiceServer service = nh_.advertiseService("locate_candidates", &candidate_locator::CandidateLocatorServer::locateCandidates, this);
	  	pub_point_clouds_ = nh_.advertise<sensor_msgs::PointCloud2>("/candidate_point_clouds", 1);
	  }

		bool locateCandidates(
			candidate_locator::LocateCandidates::Request  &req,
		  candidate_locator::LocateCandidates::Response &res)
		{
		  res.candidates = candidate_locator_.locateCandidates(
		  	req.depth_image,
		  	req.rgb_image,
		  	req.rgb_info,
		  	req.candidates);

		  if (req.publish)
		  {
			  for (uint i = 0; i < res.candidates.data.size(); i++)
			  {
			  	pub_point_clouds_.publish(res.candidates.data[i]);
			  }
			}

			return true;
		}
	};
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "candidate_locator_server");
  
  candidate_locator::CandidateLocatorServer server;
  
  ROS_INFO("Ready to locate candidates");
  ros::spin();

  return 0;
}