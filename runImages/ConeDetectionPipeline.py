
from cmath import inf
import cv2 as cv
import numpy as np

# Camera Parameters (Left is camera 1)
mtxL = np.array([[1.43643943e+03, 0.00000000e+00, 9.20883584e+02], [0.00000000e+00, 1.44182535e+03, 5.90056085e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dstL = np.array([[-1.76601573e-01,  2.47679371e-01, -1.55554300e-04,  3.37828665e-03,  -1.29219184e-01]])
mtxR = np.array([[2.13852629e+03, 0.00000000e+00, 9.42381208e+02],[0.00000000e+00, 2.13179489e+03, 5.87702763e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dstR = np.array([[-0.21975105, -0.18125776,  0.00486067, -0.00421931,  2.51366215]])

E = np.array([[ 0.00048316, -0.45258333, -0.02748465],
 [ 0.45243798, -0.01139074,  0.30094337],
 [ 0.00996433, -0.29959732, -0.01176402]])
F = np.array([[ 5.10707252e-09, -4.76602956e-06,  2.39020957e-03],
 [ 4.79746417e-06, -1.20331506e-07,  2.36887107e-04],
 [-2.59905548e-03, -2.18486384e-03,  1.00000000e+00]])

# Projection Matrices
PL = np.array([[1.43643943e+03, 0.00000000e+00, 9.20883584e+02, 0.00000000e+00],
 [0.00000000e+00, 1.44182535e+03, 5.90056085e+02, 0.00000000e+00],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00]])

PR = np.array([[ 2.13576837e+03, -3.51667421e+01,  9.47962971e+02, -2.14016578e+02],
 [-3.85322851e+00,  2.10702859e+03,  6.71088422e+02,  2.45813662e+02],
 [-2.91575222e-03, -3.93325159e-02,  9.99221923e-01,  4.53314386e-01]])

# Rectified Parameters
img_id = 3
right_image = cv.imread("right/"  + str(img_id) + ".png")
left_image = cv.imread("left/" + str(img_id) + ".png")
right_image_og = right_image
left_image_og = left_image
right_image_gray = cv.cvtColor(right_image, cv.COLOR_RGB2GRAY)
left_image_gray = cv.cvtColor(left_image, cv.COLOR_RGB2GRAY)
size = left_image_gray.shape

h,  w = left_image.shape[:2]


cv.imshow("right", right_image)
cv.waitKey(1000)
cv.imshow("left", left_image)
cv.waitKey(1000)

confThreshold = 0.8  #Confidence threshold
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



# TODO: Find Matches between found cones
# PROBLEM: Fundamental matrix is not great - giving bad matches = poor stereo calibration
centerL = []
for i in range(len(boxesL)):
    box = boxesL[i]
    x = box[0] + box[2]/2
    y = box[1] + box[3]/2
    centerL.append([x, y])
centerL = np.array(centerL)

centerR = []
for i in range(len(boxesR)):
    box = boxesR[i]
    x = box[0] + box[2]/2
    y = box[1] + box[3]/2
    centerR.append([x, y])
centerR = np.array(centerR)
# centerL = cv.undistortImagePoints(np.array(centerL), mtxL, dstL)
linesR = cv.computeCorrespondEpilines(centerL.reshape(-1,1,2), 1, F)
linesL = cv.computeCorrespondEpilines(centerR.reshape(-1,1,2), 2, F)
for point, cl in zip(centerL, classL):
    x, y = point
    if cl == 0:
        color = (255, 0, 0)
    elif cl == 1:
        color = (0, 255, 0)
    elif cl == 2:
        color = (0, 255, 255)
    else:
        color = (0, 0, 0)
    left_image = cv.circle(left_image,(int(x),int(y)),5,color,-1)

for (r, cl) in zip(linesL, classR):
    if cl == 0:
        color = (255, 0, 0)
    elif cl == 1:
        color = (0, 255, 0)
    elif cl == 2:
        color = (0, 255, 255)
    else:
        color = (0, 0, 0)
    r = r[0]
    x0,y0 = map(int, [0, -r[2]/r[1] ])
    x1,y1 = map(int, [w, -(r[2]+r[0]*w)/r[1] ])
    left_image = cv.line(left_image, (x0,y0), (x1,y1), color,1)

cv.imshow("Found Cones left", left_image)
cv.waitKey(1000)
# 
for point, cl in zip(centerR, classR):
    x, y = point
    if cl == 0:
        color = (255, 0, 0)
    elif cl == 1:
        color = (0, 255, 0)
    elif cl == 2:
        color = (0, 255, 255)
    else:
        color = (0, 0, 0)
    right_image = cv.circle(right_image,(int(x),int(y)),5,color,-1)

for (r, cl) in zip(linesR, classL):
    if cl == 0:
        color = (255, 0, 0)
    elif cl == 1:
        color = (0, 255, 0)
    elif cl == 2:
        color = (0, 255, 255)
    else:
        color = (0, 0, 0)
    r = r[0]
    x0,y0 = map(int, [0, -r[2]/r[1] ])
    x1,y1 = map(int, [w, -(r[2]+r[0]*w)/r[1] ])
    right_image = cv.line(right_image, (x0,y0), (x1,y1), color,1)

cv.imshow("Found Cones right", right_image)
cv.waitKey(1000)

for i in range(len(centerR)):
    min_dist = inf
    min_id = None
    for j in range(len(linesR)):
        if classR[i] == classL[j]:
            point = np.array([centerR[i][0], centerR[i][1], 1])
            line = linesR[j][0]
            line = np.array([line[0], line[1], line[2]])
            dist = abs(np.dot(point, line))



# TODO: Once Matches have been found compute cone locations
# triangulate points
