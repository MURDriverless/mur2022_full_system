#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#include "topic_names.h"

// Declare "member" variables
ros::Publisher cones_pub;
image_transport::Subscriber left_sub;
image_transport::Subscriber right_sub;  

void leftImageCallback(const sensor_msgs::ImageConstPtr& msg){}
void rightImageCallback(const sensor_msgs::ImageConstPtr& msg){}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "mur_stereo_ros_node");
	ros::NodeHandle nh;
    
    image_transport::ImageTransport it(nh);
    left_sub = it.subscribe(LEFT_IMAGE_TOPIC, 1, leftImageCallback);
    right_sub = it.subscribe(RIGHT_IMAGE_TOPIC, 1, rightImageCallback);
    
	// Initialise a publisher
	cones_pub = nh.advertise<std_msgs::Bool>(CONE_DETECTED_TOPIC, 10, false);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}