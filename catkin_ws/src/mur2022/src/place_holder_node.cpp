#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"

#include "topic_names.h"

// Declare "member" variables
ros::Publisher m_template_publisher;
ros::Subscriber m_template_subscriber;

// Respond to subscriber receiving a message
void templateSubscriberCallback(const std_msgs::UInt32& msg){}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "place_holder_node");
	ros::NodeHandle nh("~");

	// Initialise a publisher
	m_template_publisher = nh.advertise<std_msgs::UInt32>("bs_topic", 10, false);

	// Initialise a subscriber
	m_template_subscriber = nh.subscribe("bs_topic", 1, templateSubscriberCallback);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}