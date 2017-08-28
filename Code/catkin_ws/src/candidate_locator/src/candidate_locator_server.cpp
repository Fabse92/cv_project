#include "ros/ros.h"
#include "candidate_locator.h"
#include "candidate_locator/LocateCandidates.h"

namespace candidate_locator
{
	class CandidateLocatorServer
	{
		private:

		CandidateLocator candidate_locator_;
		ros::NodeHandle nh_;
		ros::Publisher pub_;

		public:

		CandidateLocatorServer()
		{
			pub_ = nh_.advertise<sensor_msgs::PointCloud2>("candidate_point_clouds", 20);
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
			  	pub_.publish(res.candidates.data[i]);
			  }
		  }

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