import glob

import cv2 as cv
import numpy as np


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)*20
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imagesLeft = glob.glob('Left/*')
for fname in imagesLeft:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,9), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpointsL.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,9), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(5)
cv.destroyAllWindows()

imgpointsR = []
imagesRight = glob.glob('Right/*')
for fname in imagesRight:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,9), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpointsR.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,9), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(5)
cv.destroyAllWindows()

# Camera Calibrations
if not len(imgpointsL) == len(imgpointsR):
    print("Error, not all points found")
    exit()
ret, mtxL, distL, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpointsL, gray.shape[::-1], None, None)
if not ret:
    print("Error, Camera Left Calibration")
    exit()
print("Camera Left Matrix")
print(mtxL)
ret, mtxR, distR, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpointsR, gray.shape[::-1], None, None)
if not ret:
    print("Error, Camera Right Calibration")
    exit()
print("Camera Right Matrix")
print(mtxR)

# Stereo Calibration
ret, mtxL, distL, mtxR, distR, R, T, E, F = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, mtxL, distL, mtxR, distR, gray.shape[::-1])
if not ret:
    print("Error, Stereo Calibration")
    exit()
print("Rotation")
print(R)
print("Translation")
print(T)

RTL = np.concatenate([np.eye(3), [[0],[0],[0]]], axis=-1)
PL = mtxL @ RTL
RTR = np.concatenate([R, T], axis=-1)
PR = mtxR @ RTR

print("Projection Matrices")
print(PL)
print(PR)

imageID = "1.bmp"
leftImage = cv.imread('Left/' + imageID)
rightImage = cv.imread('Right/' + imageID)

leftImage = cv.cvtColor(leftImage, cv.COLOR_BGR2GRAY)
rightImage = cv.cvtColor(rightImage, cv.COLOR_BGR2GRAY)

ret, leftPoints = cv.findChessboardCorners(leftImage, (7,9), None)
leftPoints = cv.undistortImagePoints(leftPoints, mtxL, distL)
ret, rightPoints = cv.findChessboardCorners(rightImage, (7, 9), None)
rightPoints = cv.undistortImagePoints(rightPoints, mtxR, distR)

points4H = cv.triangulatePoints(PL, PR, leftPoints, rightPoints)

print(points4H)
points3D = points4H[0:3,:]/points4H[3,:]
print("Points 3D")
print(points3D)

