#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include <sensor_msgs/PointCloud2.h>

#include "topic_names.h"

// Declare "member" variables
ros::Publisher raw_lidar_pub;

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "lidar_reader_node");
	ros::NodeHandle nh("~");

	// Initialise a publisher
	raw_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(LIDAR_RAW_TOPIC, 10, false);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}