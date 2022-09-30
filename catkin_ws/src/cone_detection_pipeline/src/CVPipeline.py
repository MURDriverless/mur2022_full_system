#! /usr/bin/env python

import rospy

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

class CVPipeline:
    def __init__(self):
        self.r_sub = rospy.Subscriber("/CameraRight/image_raw", Image, self.rightInput)
        self.l_sub = rospy.Subscriber("/CameraLeft/image_raw", Image, self.leftInput)
        self.r_new = False
        self.l_new = False

        self.cv_bridge = CvBridge()

        self.r_image = None
        self.l_image = None


    def rightInput(self, msg):
        if not self.r_new:
            self.r_image = self.cv_bridge.imgmsg_to_cv2(msg)
            self.r_new = True
    
    def leftInput(self, msg):
        if not self.l_new:
            self.l_image = self.cv_bridge.imgmsg_to_cv2(msg)
            self.l_new = True

    def pipeline(self, event=None):
        pass


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "CVPipeline"
    rospy.init_node(node_name)
    cv_pipeline = CVPipeline()

    rospy.Timer(rospy.Duration(1.0/10.0), cv_pipeline.pipeline)
    # Spin as a single-threaded node
    rospy.spin()

    # Close any OpenCV windows
    cv.destroyAllWindows()