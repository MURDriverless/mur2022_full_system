#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include "topic_names.h"

ros::Subscriber cmd_sub;

// Respond to subscriber receiving a message
void commandHusky(const geometry_msgs::Twist cmd) {    
    ROS_INFO_STREAM("Husky Control Node");
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "husky_control_node");
	ros::NodeHandle nh("~");
	
    ros::Subscriber template_subscriber = nh.subscribe(CONTROL_OUTPUT_TOPIC, 1, commandHusky);
	

	ros::spin();

	return 0;
}