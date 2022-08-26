#include "ros/ros.h"
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include "topic_names.h"

// Declare "member" variables
image_transport::Publisher left_pub;
image_transport::Publisher right_pub;



int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "both_image_aquisition");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    left_pub = it.advertise(LEFT_IMAGE_TOPIC,1);
	right_pub = it.advertise(RIGHT_IMAGE_TOPIC,1);

	ros::spin();

	return 0;
}