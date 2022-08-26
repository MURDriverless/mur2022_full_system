#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Core>
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"

#include "topic_names.h"

// Declare "member" variables
ros::Publisher full_cone_pub;
ros::Publisher point_cloud_requester_pub;
ros::Subscriber cones_found_sub;
ros::Subscriber point_cloud_requested_sub;


void foundCones(const std_msgs::Bool& msg) {}

void gotCloudBack(const sensor_msgs::PointCloud2& cloud) {}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "cone_location_node");
	ros::NodeHandle nh("~");
	
    full_cone_pub = nh.advertise<std_msgs::Bool>(CONES_FULL_TOPIC, 1, false);
    point_cloud_requester_pub = nh.advertise<std_msgs::Bool>(POINT_CLOUD_SECTION_REQUEST_TOPIC, 1);

    cones_found_sub = nh.subscribe(CONE_DETECTED_TOPIC, 1, foundCones);
    point_cloud_requested_sub = nh.subscribe(POINT_CLOUD_SECTION_TOPIC, 1, gotCloudBack);

	ros::spin();

	return 0;
}