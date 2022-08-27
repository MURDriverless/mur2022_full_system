#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "topic_names.h"

// Declare "member" variables
ros::Publisher map_pub;
ros::Publisher path_pub;
ros::Subscriber odom_sub;
ros::Subscriber cones_sub;
ros::Subscriber transition_sub;

// Respond to subscriber receiving a message
void odomUpdate(const nav_msgs::Odometry& msg){}
void conesUpdate(const std_msgs::Bool& msg){}
void transitionReact(const std_msgs::Bool& msg){}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "planner_node");
	ros::NodeHandle nh("~");

	// Initialise a publisher
	map_pub = nh.advertise<std_msgs::Bool>(CONTROL_MAP_TOPIC, 10, false);
	path_pub = nh.advertise<std_msgs::Bool>(CONTROL_PATH_TOPIC, 10, false);


	// Initialise a subscriber
	odom_sub = nh.subscribe(ODOM_TOPIC, 1, odomUpdate);
	cones_sub = nh.subscribe(CONES_FULL_TOPIC, 1, conesUpdate);
	transition_sub = nh.subscribe(CONTROL_TRANSITION_TOPIC, 1, transitionReact);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}