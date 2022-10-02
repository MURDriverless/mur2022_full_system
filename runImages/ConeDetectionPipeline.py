
import cv2 as cv
import numpy as np

img_id = 1
right_image = cv.imread("right/"  + str(img_id) + ".png")
left_image = cv.imread("left/" + str(img_id) + ".png")

cv.imshow("right", right_image)
cv.imshow("left", left_image)
