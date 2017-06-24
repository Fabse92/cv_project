#include "ros/ros.h"
#include "candidate_locator.h"
#include "candidate_locator/LocateCandidates.h"

namespace candidate_locator
{
	class CandidateLocatorServer
	{
		private:

		CandidateLocator candidate_locator_;		

		public:

		bool locateCandidates(
			candidate_locator::LocateCandidates::Request  &req,
		  candidate_locator::LocateCandidates::Response &res)
		{
		  res.candidates = candidate_locator_.locateCandidates(
		  	req.depth_image,
		  	req.rgb_image,
		  	req.rgb_info,
		  	req.candidates);

			return true;
		}
	};
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "candidate_locator_server");
  ros::NodeHandle nh;

  candidate_locator::CandidateLocatorServer server;
  
  ros::ServiceServer service = nh.advertiseService("locate_candidates", &candidate_locator::CandidateLocatorServer::locateCandidates, &server);
  
  ROS_INFO("Ready to locate candidates");
  ros::spin();

  return 0;
}