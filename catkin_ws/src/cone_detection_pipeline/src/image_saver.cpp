#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


#include "topic_names.h"

// Declare "member" variables
image_transport::Subscriber left_sub;
image_transport::Subscriber right_sub;

// Converting ROS images into OpenCV images, using cv_bridge
namespace cv_bridgeI {

class CvImage
{
public:
  std_msgs::Header header;
  std::string encoding;
  cv::Mat image;
};

typedef boost::shared_ptr<CvImage> CvImagePtr;
typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

}

#define SKIP 30
#define SAVE 60
// CV image pointers
cv_bridgeI::CvImagePtr LeftPtr;
cv_bridgeI::CVImagePtr RightPtr;
int savedCountL = 0;
int savedCountR = 0;
int skipCountL = SKIP;
int skipCountR = SKIP;

void leftImageCallback(const sensor_msgs::ImageConstPtr& msg){
	if skipCountL <= 0 {
	//Convert to opencv image using cv bridge
		LeftPtr = cv_bridge::toCvCopy(msg);
		
	}
}
void rightImageCallback(const sensor_msgs::ImageConstPtr& msg){
	//Convert to opencv image using cv bridge
	RightPtr = cv_bridge::toCvCopy(msg);
	//Set flag to true
	new_right = true;
}

void run_pipeline(const ros::TimerEvent& timer){
	if (new_left && new_right) {
		//GIVE IMAGES TO CLASS
		new_left = false;
		new_right = false;		
		//RUN THE PIPELINE
			
	}	
}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "mur_stereo_ros_node");
	ros::NodeHandle nh;
    
	image_transport::ImageTransport it(nh);
	left_sub = it.subscribe("/CameraLeft/image_raw", 1, leftImageCallback);
	right_sub = it.subscribe("/CameraRight/image_raw", 1, rightImageCallback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.05), run_pipeline);
    
	// Initialise a publisher
	cones_pub = nh.advertise<cone_msg>(CONE_DETECTED_TOPIC, 10, false);

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}
