#include "ros/ros.h"
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
ros::Subscriber lego_loam_running_sub;


ros::Time last_time;
ros::Time this_time;
ros::Time good_time;

bool system_go = false;

void systemGoCheck(const std_msgs::Bool& msg) {
  if (msg.data && !system_go) {
    system_go = true;
  }
}

ros::Time waitForTransforms(tf::TransformListener& listener) {
  bool transforms_good = false;

  while(!transforms_good) {
    try {
      tf::StampedTransform transform;
      std::cout << "Waiting" << std::endl;
      listener.lookupTransform(GLOBAL_FRAME, HUSKY_FRAME, ros::Time::now(),transform);
      std::cout << "Ready!" << std::endl;
      transforms_good = true;
    } catch (tf::LookupException) {
      continue;
    } catch (tf::ConnectivityException) {
      continue;
    }  catch (tf::ExtrapolationException) {
      continue;
    }
  }
  system_go = true;
  return ros::Time::now();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "control_pose_publisher_node");

  ros::NodeHandle nh;

  this_time = ros::Time::now();
  
  control_odom_pub = nh.advertise<nav_msgs::Odometry>(CONTROL_ODOM_TOPIC, 1);
  system_start_sub = nh.subscribe(SYSTEM_START_TOPIC, 1, systemGoCheck);

  tf::TransformListener listener;
  good_time = waitForTransforms(listener);

  ros::Rate rate(10.0);
  while (nh.ok()){
    ros::spinOnce();
    if(system_go) {
      std::cout << "Running code" << std::endl;
      this_time = ros::Time::now();
      if (this_time < good_time) {
        continue;
      }
      
      tf::StampedTransform transform;
      geometry_msgs::Twist velocity;
      try {
        listener.waitForTransform(HUSKY_FRAME, GLOBAL_FRAME, ros::Time::now(), ros::Duration(1.0));
        listener.lookupTransform(GLOBAL_FRAME, HUSKY_FRAME, this_time, transform);

        listener.waitForTransform(HUSKY_FRAME, GLOBAL_FRAME, ros::Time::now(), ros::Duration(1.0));
        listener.lookupTwist(HUSKY_FRAME, GLOBAL_FRAME, this_time,ros::Duration(0.3), velocity);
      } catch (tf::ExtrapolationException) {
        continue;
      }

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
      std::cout << "Pose published" << std::endl;
      rate.sleep();
    }
  }
  std::cout << "Exit Loop!" << std::endl;
  return 0;
}