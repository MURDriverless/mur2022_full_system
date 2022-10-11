#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>

#include "reference_frames.h"
#include "topic_names.h"

// ros::Subscriber lego_loam_odom_sub;
ros::Publisher control_odom_pub;
ros::Subscriber system_start_sub;

ros::Time last_time;
ros::Time this_time;

bool system_go;

void systemGoCheck(std_msgs::Bool& msg) {
  system_go = msg.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "control_pose_publisher_node");

  ros::NodeHandle nh;

  last_time = ros::Time::now();
  this_time = ros::Time::now();
  
  control_odom_pub = nh.advertise<nav_msgs::Odometry>(CONTROL_ODOM_TOPIC, 1);
  system_start_sub = nh.subscribe(SYSTEM_START_TOPIC, 1, &systemGoCheck);

  tf::TransformListener listener;

  ros::Rate rate(20.0);
  while (nh.ok()){
    if(system_go) {
      this_time = ros::Time::now();
  
      double time_diff = this_time.toSec() - last_time.toSec();
      
      listener.waitForTransform(GLOBAL_FRAME, HUSKY_FRAME, this_time, ros::Duration(0.1));
      listener.waitForTransform(HUSKY_FRAME,LEGO_LOAM_CAMERA_FRAME, this_time, ros::Duration(0.1));

      tf::StampedTransform transform;
      listener.lookupTransform(HUSKY_FRAME, GLOBAL_FRAME, this_time, transform);

      geometry_msgs::Twist velocity;
      listener.lookupTwist(HUSKY_FRAME, GLOBAL_FRAME, this_time,this_time - last_time, velocity);

      tf::Vector3 position = transform.getOrigin();
      tf::Quaternion orientation = transform.getRotation();

      nav_msgs::Odometry out_msg;
      out_msg.child_frame_id = HUSKY_FRAME;
      out_msg.header.frame_id = GLOBAL_FRAME;
      out_msg.header.stamp = this_time;
      
      out_msg.pose.pose.position.x = position[0];
      out_msg.pose.pose.position.y = position[1];
      out_msg.pose.pose.position.z = position[2];
      tf::quaternionTFToMsg(orientation, out_msg.pose.pose.orientation);

      out_msg.twist.twist = velocity;

      control_odom_pub.publish(out_msg);

      rate.sleep();
    }
  }
  return 0;
};