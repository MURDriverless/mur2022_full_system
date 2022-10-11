#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "reference_frames.h"
#include "topic_names.h"

// ros::Subscriber lego_loam_odom_sub;
ros::Publisher control_odom_pub;

ros::Time last_time;
ros::Time this_time;

// geometry_msgs::Pose last_pose;
// geometry_msgs::Pose this_pose;

// geometry_msgs::Twist velocity;

// void publishControlOdometry(void){
//   this_time = ros::Time::now();
  
//   double time_diff = this_time.toSec() - last_time.toSec();
  
//   // Get the current pose in '/husky' frame
//   static tf::TransformListener listener;
//   listener.waitForTransform(GLOBAL_FRAME, HUSKY_FRAME, this_time, ros::Duration(0.1));
//   listener.waitForTransform(HUSKY_FRAME,LEGO_LOAM_CAMERA_FRAME, this_time, ros::Duration(0.1));

//   tf::StampedTransform transform;
//   listener.lookupTransform(HUSKY_FRAME, GLOBAL_FRAME, this_time, transform);

//   geometry_msgs::Twist velocity;
//   listener.lookupTwist(HUSKY_FRAME, GLOBAL_FRAME, this_time,this_time - last_time, velocity);

//   tf::Vector3 position = transform.getOrigin();
//   tf::Quaternion orientation = transform.getRotation();

//   nav_msgs::Odometry out_msg;
//   out_msg.child_frame_id = HUSKY_FRAME;
//   out_msg.header.frame_id = GLOBAL_FRAME;
//   out_msg.header.stamp = this_time;
  
//   out_msg.pose.pose.position.x = position[0];
//   out_msg.pose.pose.position.y = position[1];
//   out_msg.pose.pose.position.z = position[2];
//   tf::quaternionTFToMsg(orientation, out_msg.pose.pose.orientation);

//   out_msg.twist.twist = velocity;

//   control_odom_pub.publish(out_msg);
  // transform.get
  // geometry_msgs::PoseStamped lego_loam_camera_pose, lego_loam_husky_pose;
  
  // lego_loam_camera_pose.pose = msg.pose.pose;
  // lego_loam_camera_pose.header.frame_id = msg.child_frame_id;
  // lego_loam_camera_pose.header.stamp = this_time;

  // listener.transformPose(HUSKY_FRAME, lego_loam_camera_pose, lego_loam_husky_pose); 

  // geometry_msgs::PointStamped global_husky_position, lego_loam_husky_position;
  
  // lego_loam_husky_position.header.frame_id = msg.header.frame_id;
  // lego_loam_husky_position.header.stamp = this_time;
  // lego_loam_husky_position.point = lego_loam_husky_pose.pose.position;

  // listener.transformPoint(GLOBAL_FRAME, lego_loam_husky_position, global_husky_position);

  // geometry_msgs::QuaternionStamped global_husky_orientation,lego_loam_husky_orientation;

  // lego_loam_husky_orientation.header.frame_id = msg.header.frame_id;
  // lego_loam_husky_orientation.header.stamp = this_time;
  // lego_loam_husky_orientation.quaternion = lego_loam_husky_pose.pose.orientation;

  // listener.transformQuaternion(GLOBAL_FRAME, lego_loam_husky_orientation, global_husky_orientation);

  // this_pose.position = global_husky_position.point;
  // this_pose.orientation = global_husky_orientation.quaternion;  

  // double roll, pitch, yaw;
  // geometry_msgs::Quaternion geoQuat = global_husky_orientation.quaternion;
  // tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
  //     .getRPY(roll, pitch, yaw);

  // Compute the velocities
  // velocity.linear.x = (this_pose.position.x - last_pose.position.x)/time_diff;
  // velocity.linear.y = (this_pose.position.y - last_pose.position.y)/time_diff;
  // velocity.linear.z = (this_pose.position.z - last_pose.position.z)/time_diff;


  // velocity.angular.x = (this_pose.orientation.x - last_pose.orientation.x)/time_diff;
  // velocity.angular.y = (this_pose.orientation.y - last_pose.orientation.y)/time_diff;
  // velocity.angular.z = (this_pose.orientation.z - last_pose.orientation.z)/time_diff;

  // // Create the message
  // nav_msgs::Odometry out_msg;
  // out_msg.child_frame_id = HUSKY_FRAME;
  // out_msg.header.frame_id = GLOBAL_FRAME;
  // out_msg.header.stamp = this_time;
  
  // out_msg.pose.pose = this_pose;
  // out_msg.twist.twist = velocity;

  // control_odom_pub.publish(out_msg);

  // last_pose = this_pose;
  // last_time = this_time;
// }

int main(int argc, char** argv){
  ros::init(argc, argv, "control_pose_publisher_node");

  ros::NodeHandle nh;

  last_time = ros::Time::now();
  this_time = ros::Time::now();

  // last_pose.orientation.w = 0; 
  // last_pose.orientation.x = 0; 
  // last_pose.orientation.y = 0; 
  // last_pose.orientation.z = 0;

  // last_pose.position.x = 0;
  // last_pose.position.y = 0;
  // last_pose.position.z = 0;

  // this_pose.orientation.w = 0; 
  // this_pose.orientation.x = 0; 
  // this_pose.orientation.y = 0; 
  // this_pose.orientation.z = 0;

  // this_pose.position.x = 0;
  // this_pose.position.y = 0;
  // this_pose.position.z = 0;
  control_odom_pub = nh.advertise<nav_msgs::Odometry>(CONTROL_ODOM_TOPIC, 1);
  tf::TransformListener listener;

  ros::Rate rate(20.0);
  while (nh.ok()){
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
  ros::spin();

  return 0;
};