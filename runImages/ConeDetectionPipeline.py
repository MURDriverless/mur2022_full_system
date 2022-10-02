
import cv2 as cv
import numpy as np

# Camera Parameters (Left is camera 1)
mtxL = np.array([[1.56995834e+03, 0.00000000e+00, 9.47610312e+02],
       [0.00000000e+00, 1.56512241e+03, 5.97284955e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dstL = np.array([[-0.14712914,  0.0882084 ,  0.00025096,  0.00611444,  0.16686285]])

mtxR = np.array([[1.49213810e+03, 0.00000000e+00, 9.59806966e+02],
       [0.00000000e+00, 1.49304631e+03, 5.86244421e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dstR = np.array([[-0.14714803,  0.14932018, -0.00110836,  0.00198408, -0.03618555]])

E = np.array([[   6.84078536,  123.4959375 ,   43.2993839 ],
       [-111.77461723,   -1.34782091,   83.09947311],
       [ -41.45509683,  -63.43891801,    6.14428692]])
F = np.array([[ 2.48835145e-06,  4.50607325e-05, -4.54484750e-03],
       [-4.06335427e-05, -4.91488667e-07,  8.62255678e-02],
       [-1.06767735e-03, -7.75005204e-02,  1.00000000e+00]])

# Projection Matrices
PL = np.array([[1.56995834e+03, 0.00000000e+00, 9.47610312e+02, 0.00000000e+00],
       [0.00000000e+00, 1.56512241e+03, 5.97284955e+02, 0.00000000e+00],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00]])

PR = np.array([[ 1.63144924e+03, -7.23732082e+00,  6.97156017e+02, -2.13518374e+05],
       [ 9.55207847e+01,  1.48490878e+03,  5.98992073e+02, -9.81525338e+03],
       [ 1.66971049e-01, -1.36248738e-02,  9.85867654e-01, -1.24082654e+02]])

# Rectified Parameters
R1 = np.array([[ 0.57131619, -0.29823783,  0.7646254 ],
       [ 0.30466724,  0.9421383 ,  0.13983312],
       [-0.7620864 ,  0.15306738,  0.62912216]])
R2 = np.array([[ 0.43443358, -0.28822272,  0.8533435 ],
       [ 0.28068922,  0.94356187,  0.17579691],
       [-0.85585105,  0.16315224,  0.49081598]])
P1 = np.array([[ 1.52908458e+03,  0.00000000e+00, -1.25297165e+03, 0.00000000e+00],
       [ 0.00000000e+00,  1.52908458e+03,  3.34065872e+02, 0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00]])
P2 = np.array([[ 1.52908458e+03,  0.00000000e+00, -1.25297165e+03, -2.21970144e+02],
       [ 0.00000000e+00,  1.52908458e+03,  3.34065872e+02, 0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00]])
Q = np.array([[ 1.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.25297165e+03],
       [ 0.00000000e+00,  1.00000000e+00,  0.00000000e+00, -3.34065872e+02],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.52908458e+03],
       [ 0.00000000e+00,  0.00000000e+00,  6.88869483e+00, -0.00000000e+00]])
roi1 = (0, 0, 190, 1243)
roi2 = (0, 0, 753, 1418)

img_id = 3
right_image = cv.imread("right/"  + str(img_id) + ".png")
left_image = cv.imread("left/" + str(img_id) + ".png")
right_image_og = right_image
left_image_og = left_image
right_image_gray = cv.cvtColor(right_image, cv.COLOR_RGB2GRAY)
left_image_gray = cv.cvtColor(left_image, cv.COLOR_RGB2GRAY)
size = left_image_gray.shape

h,  w = left_image.shape[:2]
newcameramtxL, roi = cv.getOptimalNewCameraMatrix(mtxL, dstL, (w,h), 1, (w,h))
left_image = cv.undistort(left_image, mtxL, dstL, None, newcameramtxL)
x, y, w, h = roi
left_image = left_image[y:y+h, x:x+w]


cv.imshow("right", right_image)
cv.waitKey(1000)
cv.imshow("left", left_image)
cv.waitKey(1000)

confThreshold = 0.75  #Confidence threshold
nmsThreshold = 0.8  #Non-maximum suppression threshold
inpWidth = 832       #Width of network's input image
inpHeight = 832      #Height of network's input image

# Load names of classes
classesFile = "cones.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

# Give the configuration and weight files for the model and load the network using them.
modelConfiguration = "yolov4-tiny-cones.cfg"
modelWeights = "yolov4-tiny-cones_best.weights"

net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)

# Get the names of the output layers
def getOutputsNames(net):
    # Get the names of all the layers in the network
    layersNames = net.getLayerNames()
    # Get the names of the output layers, i.e. the layers with unconnected outputs
    return [layersNames[i - 1] for i in net.getUnconnectedOutLayers()]

# Draw the predicted bounding box
def drawPred(image, classId, conf, left, top, right, bottom):
    # Draw a bounding box.
    cv.rectangle(image, (left, top), (right, bottom), (255, 178, 50), 3)
    
    label = '%.2f' % conf
        
    # Get the label for the class name and its confidence
    if classes:
        assert(classId < len(classes))
        label = '%s:%s' % (classes[classId], label)

    #Display the label at the top of the bounding box
    labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, labelSize[1])
    cv.rectangle(image, (left, top - round(1.5*labelSize[1])), (left + round(1.5*labelSize[0]), top + baseLine), (255, 255, 255), cv.FILLED)
    cv.putText(image, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 1)

# Remove the bounding boxes with low confidence using non-maxima suppression
def postprocess(image, outs):
    imageHeight = image.shape[0]
    imageWidth = image.shape[1]

    # Scan through all the bounding boxes output from the network and keep only the
    # ones with high confidence scores. Assign the box's class label as the class with the highest score.
    classIds = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                center_x = int(detection[0] * imageWidth)
                center_y = int(detection[1] * imageHeight)
                width = int(detection[2] * imageWidth)
                height = int(detection[3] * imageHeight)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])

    # Perform non maximum suppression to eliminate redundant overlapping boxes with
    # lower confidences.
    indices = cv.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    oBoxes = []
    oClasses = []
    for i in indices:
        box = boxes[i]
        oBoxes.append(box)
        oClasses.append(classIds[i])
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        drawPred(image, classIds[i], confidences[i], left, top, left + width, top + height)
    return oBoxes, oClasses

# Create a 4D blob from a frame.
blobR = cv.dnn.blobFromImage(right_image, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)
# Sets the input to the network
net.setInput(blobR)

# Runs the forward pass to get output of the output layers
outsR = net.forward(getOutputsNames(net))

# Remove the bounding boxes with low confidence
boxesR, classR = postprocess(right_image, outsR)

# Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
t, _ = net.getPerfProfile()
label = 'Inference time: %.2f ms' % (t * 1000.0 / cv.getTickFrequency())
cv.putText(right_image, label, (0, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

cv.imshow("Found Cones right", right_image)
cv.waitKey(1000)

# Create a 4D blob from a frame.
blobL = cv.dnn.blobFromImage(left_image, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)
# Sets the input to the network
net.setInput(blobL)

# Runs the forward pass to get output of the output layers
outsL = net.forward(getOutputsNames(net))

# Remove the bounding boxes with low confidence
boxesL, classL = postprocess(left_image, outsL)

# Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
t, _ = net.getPerfProfile()
label = 'Inference time: %.2f ms' % (t * 1000.0 / cv.getTickFrequency())
cv.putText(left_image, label, (0, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

cv.imshow("Found Cones left", left_image)
cv.waitKey(1000)

# TODO: Find Matches between found cones
# PROBLEM: Fundamental matrix is not great - giving bad matches = poor stereo calibration
centerL = []
for i in range(len(boxesL)):
    box = boxesL[i]
    x = box[0] + box[2]/2
    y = box[1] + box[3]/2
    centerL.append([x, y])
centerL = np.array(centerL)
# centerL = cv.undistortImagePoints(np.array(centerL), mtxL, dstL)

centerR = []
for i in range(len(boxesR)):
    box = boxesR[i]
    x = box[0] + box[2]/2
    y = box[1] + box[3]/2
    centerR.append([x, y])
centerR = np.array(centerR)
# centerR = cv.undistortImagePoints(np.array(centerR), mtxR, dstR)

# mapL1, mapL2 = cv.initUndistortRectifyMap(mtxL, dstL, R1, P1, size, cv.CV_32FC1)

# left_image_rect = cv.remap(left_image_og, mapL1, mapL2, cv.INTER_LINEAR, cv.BORDER_CONSTANT)
# cv.imshow("Left_image_rectified", left_image_rect)
# cv.waitKey(0)

# mapR1, mapR2 = cv.initUndistortRectifyMap(mtxR, dstR, R2, P2, size, cv.CV_32FC1)
# right_image_rect = cv.remap(right_image_og, mapR1, mapR2, cv.INTER_LINEAR, cv.BORDER_CONSTANT)
# cv.imshow("Right_image_rectified", right_image_rect)
# cv.waitKey(0)

# Recalculate Fundamental matrix for now 
img1 = left_image_gray
img2 = right_image_gray

sift = cv.SIFT_create()
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)
# FLANN parameters
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)
flann = cv.FlannBasedMatcher(index_params,search_params)
matches = flann.knnMatch(des1,des2,k=2)
pts1 = []
pts2 = []
# ratio test as per Lowe's paper
for i,(m,n) in enumerate(matches):
    if m.distance < 0.8*n.distance:
        pts2.append(kp2[m.trainIdx].pt)
        pts1.append(kp1[m.queryIdx].pt)

pts1 = np.int32(pts1)
pts2 = np.int32(pts2)
Fnew, mask = cv.findFundamentalMat(pts1,pts2,cv.FM_LMEDS)
# We select only inlier points
pts1 = pts1[mask.ravel()==1]
pts2 = pts2[mask.ravel()==1]

def drawlines(img1,img2,lines,pts1,pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r,c = img1.shape
    img1 = cv.cvtColor(img1,cv.COLOR_GRAY2BGR)
    img2 = cv.cvtColor(img2,cv.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv.circle(img2,tuple(pt2),5,color,-1)
    return img1,img2

# Find epilines corresponding to points in right image (second image) and
# drawing its lines on left image

# Find epilines corresponding to points in left image (first image) and
# drawing its lines on right image
lines2 = cv.computeCorrespondEpilines(centerL.reshape(-1,1,2), 1,Fnew)
lines2 = lines2.reshape(-1,3)
img3,img4 = drawlines(img2,img1,lines2,pts2,pts1)
cv.imshow("right image", img3)
cv.waitKey(1000)
print(Fnew)
# TODO: Once Matches have been found compute cone locations
# triangulate points
