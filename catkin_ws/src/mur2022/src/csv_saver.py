#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from mur_common.msg import cone_msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    v_yaw = data.twist.twist.angular.z
    fP.write(str(x) + "," + str(y) + "," + str(z) + "," + str(v_x) + "," + str(v_y) + "," + str(v_z) + "," + str(yaw) + "," + str(v_yaw) + "," + str(rospy.Time.now()) + "\n")

def cone_save(data):
    global last
    time = rospy.Time.now()
    for i in range(last, len(data.x)):
        fC.write(str(data.x[i]) + "," + str(data.y[i]) + "," + str(data.colour[i]) + "," + str(time) + "\n")
    last = len(data.x)

def CSVSaver():
    rospy.init_node("csv_saver")

    rospy.Subscriber(POSE_TOPIC, Odometry, pose_save)
    rospy.Subscriber(CONE_TOPIC, cone_msg, cone_save)

    rospy.spin()


if __name__ == '__main__':
    fP = open(POSE_SAVE, "w")
    fP.write("x,y,z,v_x,v_y,v_z,yaw,v_yaw,time\n")
    fC = open(CONE_SAVE, "w")
    fC.write("x,y,colour,time\n")  
    global last
    last = 0
    try:
        CSVSaver()
    except rospy.ROSInterruptException:
        fP.close()
        fC.close()
