#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define SKIP 30
#define SAVE 30

// Declare "member" variables
ros::Subscriber left_sub;
ros::Subscriber right_sub;

int skipL = SKIP;
int skipR = SKIP;
int saveL = 0;
int saveR = 0;

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

// CV image pointers
cv_bridgeI::CvImagePtr LeftPtr;
cv_bridgeI::CvImagePtr RightPtr;


void leftImageCallback(const sensor_msgs::ImageConstPtr& msg){
	if (skipL <= 0 && saveL < SAVE) {
		LeftPtr = cv_bridge::toCvCopy(msg);
		cv::imwrite("LeftRosbag/" + std::to_string(saveL) + ".png", LeftPtr->image);
		skipL = SKIP;
		saveL += 1;
	} else {
		skipL -= 1;
	}	
}

void rightImageCallback(const sensor_msgs::ImageConstPtr& msg){
	if (skipR <= 0 && saveR < SAVE) {
		RightPtr = cv_bridge::toCvCopy(msg);
		cv::imwrite("RightRosbag/" + std::to_string(saveR) + ".png", RightPtr->image);
		skipR = SKIP;
		saveR += 1;
	} else {
		skipR -= 1;
	}
}

int main(int argc, char* argv[]) {
	// Initialise the node
	ros::init(argc, argv, "mur_stereo_ros_node");
	ros::NodeHandle nh;
    
	left_sub = nh.subscribe("/CameraLeft/image_raw", 1, leftImageCallback);
	right_sub = nh.subscribe("/CameraRight/image_raw", 1, rightImageCallback);
    

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}
