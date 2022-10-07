#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "topic_names_mur.h"

// Declare "member" variables
ros::Subscriber control_output_sub;
ros::Subscriber safety_sub;
ros::Publisher control_output_pub;

bool safe = true;

// Respond to subscriber receiving a message
void sendControlOutputs(const geometry_msgs::Twist msg){
	if (!safe) {
		control_output_pub.publish(msg);
	}
}

void updateSafety(const std_msgs::Bool msg){
	safe = msg.data;
}



int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "output_transformation_node");
	ros::NodeHandle nh("~");

	// Initialise a subscriber
	safety_sub = nh.subscribe("/safe", 1, updateSafety);
	control_output_sub = nh.subscribe("/mur2022internal/cmd_vel", 1, sendControlOutputs);
	control_output_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}