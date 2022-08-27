#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include "topic_names.h"

// Declare "member" variables
ros::Subscriber control_output_sub;

// Respond to subscriber receiving a message
void sendControlOutputs(const std_msgs::Bool& msg){}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "output_transformation_node");
	ros::NodeHandle nh("~");

	// Initialise a subscriber
	control_output_sub = nh.subscribe(CONTROL_OUTPUT_TOPIC, 1, sendControlOutputs);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}