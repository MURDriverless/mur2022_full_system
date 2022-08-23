#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"

// Declare "member" variables
ros::Publisher m_template_publisher;
ros::Timer m_timer_for_publishing;

// Respond to timer callback
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	static uint counter = 0;
	counter++;
	// Publish a message
	std_msgs::UInt32 msg;
	msg.data = counter;
	m_template_publisher.publish(msg);
}

// Respond to subscriber receiving a message
void templateSubscriberCallback(const std_msgs::UInt32& msg)
{
	ROS_INFO_STREAM("[TEMPLATE CPP NODE MINIMAL] Message receieved with data = " << msg.data);
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "place_holder_node");
	ros::NodeHandle nodeHandle("~");
	// Initialise a publisher
	m_template_publisher = nodeHandle.advertise<std_msgs::UInt32>("bs_topic", 10, false);
	// Initialise a timer
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(1.0), timerCallbackForPublishing, false);
	// Initialise a subscriber
	ros::Subscriber template_subscriber = nodeHandle.subscribe("bs_topic", 1, templateSubscriberCallback);
	// Spin as a single-threaded node
	ros::spin();

	return 0;
}