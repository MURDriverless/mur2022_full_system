# Import the ROS-Python package
from math import atan2, sin
from math import cos
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from mur2022.msg import found_cone_msg

# Import numpy
import numpy as np

# Import opencv
import cv2

# Import aruco
import cv2.aruco as aruco

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge

class ConeDetector:

    def __init__(self):
        self.detected_cone_pub = rospy.Publisher("/stereo_cone", found_cone_msg, queue_size=20)

        

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "Cone Detection"
    rospy.init_node(node_name)    

    # Spin as a single-threaded node
    rospy.spin()


