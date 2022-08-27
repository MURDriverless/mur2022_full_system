#include "ros/ros.h"
#include <ros/package.h>

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "follower_node");
	ros::NodeHandle nodeHandle("~");
	
	ros::spin();

	return 0;
}