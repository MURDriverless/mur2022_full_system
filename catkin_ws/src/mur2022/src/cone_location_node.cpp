#include <ros/package.h>
#include <Eigen/Core>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "topic_names.h"
#include "mur_common/cone_msg.h" 
#include "mur2022/found_cone_msg.h"

#define CONES_DIST_THRESHOLD 1.5

// Declare "member" variables
ros::Publisher full_cones_pub;
ros::Subscriber cones_found_sub;
// ros::Subscriber pose_sub;

tf::TransformListener tf_listener;

std::vector<float> cones_x;
std::vector<float> cones_y;
std::vector<std::string> colour;

// Checks if the new cone has already been found
static bool needToAdd(float x, float y);

// Uses current pose and measured position to get the global cone location
static geometry_msgs::Point getConeGlobalPosition(geometry_msgs::PointStamped local_point);

// Publish vectors of cones
static void publishCones(void);

// Callback for new cones found location;
void foundCones(const mur2022::found_cone_msg& msg);

int main(int argc, char* argv[]) {
  // Initialise the node
  ros::init(argc, argv, "cone_location_node");
  ros::NodeHandle nh("~");

  full_cones_pub = nh.advertise<mur_common::cone_msg>(CONES_FULL_TOPIC, 1, false);

  cones_found_sub = nh.subscribe(CONE_DETECTED_TOPIC, 1, foundCones);

  ros::spin();

  return 0;
}

void foundCones(const mur2022::found_cone_msg& msg) {
  geometry_msgs::PointStamped stamped_point;

  stamped_point = msg.point;

  geometry_msgs::Point global_point = getConeGlobalPosition(stamped_point);
	
  if(needToAdd(global_point.x, global_point.y)) {		
		cones_x.push_back(global_point.x);
		cones_y.push_back(global_point.y);
		
		// NEED TO CHANGE THIS
    colour.push_back(msg.colour);
		publishCones();
	} else {
		std::cout << "Cone found at: (" << global_point.x << ", " << global_point.y << ") too close to another cone.";
	}
}

static geometry_msgs::Point getConeGlobalPosition(geometry_msgs::PointStamped local_point) {
  
  geometry_msgs::PointStamped global_point;

  tf_listener.transformPoint("/map", local_point, global_point);

  return global_point.point;
}

static bool needToAdd(float x, float y) {
  for(int i = 0; i < cones_x.size(); i++) {
    float dist = sqrt(pow(x - cones_x[i], 2) + pow(y - cones_y[i], 2));
    if (dist < CONES_DIST_THRESHOLD) {
      return false;
    }
  }
  return true;
}

static void publishCones(void) {  
  ros::Time current_time = ros::Time::now();
  
  mur_common::cone_msg cones;
  cones.header.frame_id = "/map";
  cones.header.stamp = current_time;

  cones.x = cones_x;
  cones.y = cones_y;
  cones.colour = colour;

  full_cones_pub.publish(cones);
}