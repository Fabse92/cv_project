#include <ros/ros.h>
#include "listener.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LISTENER");

	LISTENER listener;
	ros::spin();
	return 0;
}
