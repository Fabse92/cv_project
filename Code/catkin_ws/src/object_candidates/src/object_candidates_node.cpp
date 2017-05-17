#include <ros/ros.h>
#include "OBJECT_CANDIDATES.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "OBJECT_CANDIDATES");

	OBJECT_CANDIDATES object_candidates;
	ros::spin();
	return 0;
}
