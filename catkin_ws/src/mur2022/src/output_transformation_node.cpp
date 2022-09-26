#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "topic_names.h"

// Declare "member" variables
ros::Subscriber control_output_sub;
ros::Publisher control_output_pub;

// Respond to subscriber receiving a message
void sendControlOutputs(const geometry_msgs::Twist msg){
	control_output_pub.publish(msg);
}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "output_transformation_node");
	ros::NodeHandle nh("~");

	// Initialise a subscriber
	control_output_sub = nh.subscribe("/husky_velocity_controller/cmd_vel", 1, sendControlOutputs);
	control_output_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}