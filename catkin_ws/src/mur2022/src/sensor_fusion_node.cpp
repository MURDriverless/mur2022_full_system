#include <ros/package.h>
#include <Eigen/Core>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "topic_names.h"
#include "reference_frames.h"

#include "mur_common/cone_msg.h" 
#include "mur2022/found_cone_msg.h"

#define CONES_DIST_THRESHOLD 1.5f
#define LOOK_AHEAD_DIST 6.5f

#define ORANGE_START_X 0.0f
#define ORANGE_START_LEFT 1.5f
#define ORANCE_START_RIGHT -1.5f

#define ORANGE "orange"
#define BLUE "blue"
#define YELLOW "yellow"

class SensorFusionNode {
  public:
    ros::NodeHandle nh_;
    ros::NodeHandle globalNode;

    ros::Publisher full_cones_pub;
    ros::Publisher full_cones_rviz_pub;
    ros::Subscriber cones_found_sub;
    ros::Subscriber start_run_sub;

    tf::TransformListener* listener;

    std::vector<float> cones_x;
    std::vector<float> cones_y;
    std::vector<std::string> cone_colours;

    bool system_go;

    bool verbose;
    bool rviz;
    std::vector<visualization_msgs::Marker> cones_viz_array;

    // Checks if the new cone has already been found
    bool needToAdd(float x, float y, std::string colour);

    // Uses current pose and measured position to get the global cone location
    geometry_msgs::Point getConeGlobalPosition(geometry_msgs::PointStamped local_point);

    // Publish vectors of cones
    void publishCones(void);
    void publishConesToRviz(float x, float y, std::string colour);
    void addMarkerToArray(float x, float y, std::string colour);
  
    // Callback for new cones found location;
    void foundCones(const mur2022::found_cone_msg& msg);
    void startSystem(const std_msgs::Bool& msg);

    SensorFusionNode(ros::NodeHandle nh, bool use_rviz, bool use_verbose) {
      this->rviz = use_rviz;
      this->verbose = use_verbose;

      this->full_cones_pub = nh.advertise<mur_common::cone_msg>(CONES_FULL_TOPIC, 1, false);
      this->full_cones_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(CONES_RVIZ_TOPIC, 1, false);

      this->cones_found_sub = nh.subscribe(CONE_DETECTED_TOPIC, 10, &SensorFusionNode::foundCones, this);
      this->start_run_sub = nh.subscribe(SYSTEM_START_TOPIC, 1, &SensorFusionNode::startSystem, this); 
      this->listener = new tf::TransformListener();
    }
};


// Declare "member" variables


int main(int argc, char* argv[]) {
  // Initialise the node
  ros::init(argc, argv, "sensor_fusion_node");
  
  ros::NodeHandle nh;

  bool use_rviz = nh.param("use_rviz", true);
  bool use_verbose = nh.param("verbose", true);

  if(use_rviz) {
    std::cout << "Running with rviz" << std::endl;
  } else {
    std::cout << "Running without rviz" << std::endl;
  } 

  SensorFusionNode sensorFN = SensorFusionNode(nh, use_rviz, use_verbose);

  ros::spin();
  
  return 0;
}

void SensorFusionNode::startSystem(const std_msgs::Bool& msg) {
  if(msg.data) {
    cones_x.push_back(ORANGE_START_X);
    cones_y.push_back(ORANGE_START_LEFT);
    cone_colours.push_back(ORANGE);
    publishConesToRviz(ORANGE_START_X, ORANGE_START_LEFT, ORANGE);

    cones_x.push_back(ORANGE_START_X);
    cones_y.push_back(ORANCE_START_RIGHT);
    cone_colours.push_back(ORANGE);

    publishCones();
    publishConesToRviz(ORANGE_START_X, ORANCE_START_RIGHT, ORANGE);

    if(this->verbose) {
      std::cout << "Published the initial orange cones." << std::endl;
    }
    this->system_go = true;
  }
}

void SensorFusionNode::foundCones(const mur2022::found_cone_msg& msg) {
  if(this->system_go){
    if(msg.point.point.x > LOOK_AHEAD_DIST) {
      if(this->verbose) {
        std::cout << "Cone found is too far ahead." << std::endl;
      }
      return;
    }
    
    geometry_msgs::Point global_point = getConeGlobalPosition(msg.point);
    
    if(needToAdd(global_point.x, global_point.y, msg.colour)) {		
      cones_x.push_back(global_point.x);
      cones_y.push_back(global_point.y);
      
      cone_colours.push_back(msg.colour);

      publishCones();
      if(rviz) {
        publishConesToRviz(global_point.x, global_point.y, msg.colour);
      }
    } else {
      if(this->verbose) {
        std::cout << "Cone found at: (" << global_point.x << ", " << global_point.y << ") too close to another cone." << std::endl;
      }		
    }
  }
}

geometry_msgs::Point SensorFusionNode::getConeGlobalPosition(geometry_msgs::PointStamped local_point) {
  
  geometry_msgs::PointStamped global_point;
  listener->waitForTransform("/map", local_point.header.frame_id, local_point.header.stamp, ros::Duration(1.0));
  listener->transformPoint("/map", local_point, global_point);

  return global_point.point;
}

bool SensorFusionNode::needToAdd(float x, float y, std::string colour) {
  for(int i = 0; i < cones_x.size(); i++) {
    float dist = sqrt(pow(x - cones_x[i], 2) + pow(y - cones_y[i], 2));
    if (dist < CONES_DIST_THRESHOLD) {
      if(colour.compare(cone_colours[i])) {
        return false;
      }      
    }
  }
  return true;
}

void SensorFusionNode::publishCones(void) {  
  ros::Time current_time = ros::Time::now();
  
  mur_common::cone_msg cones;
  cones.header.frame_id = GLOBAL_FRAME;
  cones.header.stamp = current_time;

  cones.x = cones_x;
  cones.y = cones_y;
  cones.colour = cone_colours;

  full_cones_pub.publish(cones);
}

void SensorFusionNode::addMarkerToArray(float x, float y, std::string colour) {
  int index = cones_x.size() - 1;

  if(this->verbose) {
    std::cout << "Index: " << index << std::endl;
  }

  visualization_msgs::Marker marker;

  marker.header.frame_id = GLOBAL_FRAME;
  marker.header.stamp = ros::Time::now();
  marker.id = index;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.5;

  if(colour.compare(BLUE) == 0) {
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  } else if(colour.compare(YELLOW) == 0) {
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  } else if(colour.compare(ORANGE) == 0) {
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.65;
    marker.color.b = 0.0;
  } else { // unknown = white
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }

  cones_viz_array.push_back(marker);
}

void SensorFusionNode::publishConesToRviz(float x, float y, std::string colour) {
  
  addMarkerToArray(x, y, colour);  

  visualization_msgs::MarkerArray markerArray;
  markerArray.markers = cones_viz_array;

  full_cones_rviz_pub.publish(markerArray);
}