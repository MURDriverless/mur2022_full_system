#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

EXPERIMENT = "StraightASpeedrun1"

POSE_TOPIC = "/control_odom"
POSE_SAVE = "/home/micah/Documents/mur2022_full_system/ExperimentResults/" + EXPERIMENT + "pose.csv"
CONE_TOPIC = "/mur/slam/cones"
CONE_SAVE = "/home/micah/Documents/mur2022_full_system/ExperimentResults/" + EXPERIMENT + "cones.csv"

def pose_save(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    v_x = data.twist.twist.linear.x
    v_y = data.twist.twist.linear.y
    v_z = data.twist.twist.linear.z
    fP.write(str(x) + "," + str(y) + "," + str(z) + "," + "\n")

def CSVSaver():
    rospy.init_node("csv_saver")

    rospy.Subscriber(POSE_TOPIC, Odometry, pose_save)

    rospy.spin()


if __name__ == '__main__':
    fP = open(POSE_SAVE, "w")
    fP.write("x,y,z\n")
    fC = open(CONE_SAVE, "w")  
    try:
        CSVSaver()
    except rospy.ROSInterruptException:
        fP.close()
        fC.close()
