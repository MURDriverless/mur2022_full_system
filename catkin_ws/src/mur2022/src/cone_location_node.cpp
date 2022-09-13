#include <ros/package.h>
#include <Eigen/Core>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "topic_names.h"

// Declare "member" variables
ros::Publisher full_cone_pub;
ros::Publisher point_cloud_requester_pub;
ros::Subscriber cones_found_sub;
ros::Subscriber point_cloud_requested_sub;

std::vector<float> cones_x;
std::vector<float> cones_y;
std::vector<std::string> color;

typedef struct __point__ {
  float x;
  float y;
  float z;
} point_t;

typedef struct __ori__ {
  float x;
  float y;
  float z;
  float w;
} ori_t;

typedef struct __pose__ {
  point_t point;
  ori_t ori;
} pose_t;

pose_t current_pose;

// Checks if the new cone has already been found
static bool needToAdd(float x, float y);

// Initialises the global varilabes
static void initialiseGlobalVarliables(void);

// Uses current pose and measured position to get the global cone location
static point_t getConeGlobalPosition(float x, float y, float z);

// Publish vectors of cones
static void publishCones(void);

void foundCones(const geometry_msgs::Point& msg) {
	float x, y, z;
	x = msg.x;
	y = msg.y;
	x = msg.z;

	if(needToAdd(x, y)) {
		point_t global_point = getConeGlobalPosition(x, y, z);
		cones_x.push_back(global_point.x);
		cones_y.push_back(global_point.y);
		
		// NEED TO CHANGE THIS
		color.push_back("blue");
		publishCones();
	} else {
		std::cout << "Cone found at: (" << x << ", " << y << ") too close to another cone.";
	}
}

void gotCloudBack(const sensor_msgs::PointCloud2& cloud) {}

void gotCurrentPose(const nav_msgs::Odometry& msg) {
  current_pose.point.x = msg.pose.pose.position.x;
  current_pose.point.y = msg.pose.pose.position.y;
  current_pose.point.x = msg.pose.pose.position.z;

  current_pose.ori.x = msg.pose.pose.orientation.x;
  current_pose.ori.y = msg.pose.pose.orientation.y;
  current_pose.ori.z = msg.pose.pose.orientation.z;
  current_pose.ori.w = msg.pose.pose.orientation.w;
}

int main(int argc, char* argv[]) {
  // Initialise the node
  ros::init(argc, argv, "cone_location_node");
  ros::NodeHandle nh("~");

  full_cone_pub = nh.advertise<std_msgs::Bool>(CONES_FULL_TOPIC, 1, false);
  point_cloud_requester_pub =
      nh.advertise<std_msgs::Bool>(POINT_CLOUD_SECTION_REQUEST_TOPIC, 1);

  cones_found_sub = nh.subscribe(CONE_DETECTED_TOPIC, 1, foundCones);
  point_cloud_requested_sub =
      nh.subscribe(POINT_CLOUD_SECTION_TOPIC, 1, gotCloudBack);

  initialiseGlobalVarliables();

  ros::spin();

  return 0;
}

static void initialiseGlobalVarliables(void) {
  current_pose.point.x = 0;
  current_pose.point.y = 0;
  current_pose.point.z = 0;

  current_pose.ori.x = 0;
  current_pose.ori.y = 0;
  current_pose.ori.z = 0;
  current_pose.ori.w = 0;
}

static point_t getConeGlobalPosition(float x, float y, float z) {}

static bool needToAdd(float x, float y);