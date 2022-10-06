import cv2 as cv
import numpy as np
import glob

objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = []
images = glob.glob('Camera\ Calibration\ GRID\ IMAGES/*')

for image in images:
    pass
