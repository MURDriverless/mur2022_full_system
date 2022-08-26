#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"

#include "topic_names.h"

// Declare "member" variables
ros::Publisher odom_filtered_pub;
ros::Publisher point_cloud_requested_pub;
ros::Subscriber lidar_raw_sub;
ros::Subscriber point_cloud_requested_sub;


void sendPointCloud(const std_msgs::Bool& msg){}
void useNewCloud(const sensor_msgs::PointCloud2& msg){}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "lego_loam_node");
	ros::NodeHandle nh("~");

	// Initialise a publisher
	odom_filtered_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 1, false);
    point_cloud_requested_pub = nh.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD_SECTION_TOPIC, 1, false);

	// Initialise a subscriber
	point_cloud_requested_sub = nh.subscribe(POINT_CLOUD_SECTION_REQUEST_TOPIC, 1, sendPointCloud);
	lidar_raw_sub = nh.subscribe(LIDAR_RAW_TOPIC, 1, useNewCloud);
	
    // Spin as a single-threaded node
	ros::spin();

	return 0;
}