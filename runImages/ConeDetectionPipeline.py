
import cv2 as cv
import numpy as np

img_id = 1
right_image = cv.imread("right/"  + str(img_id) + ".png")
left_image = cv.imread("left/" + str(img_id) + ".png")
right_image_gray = cv.cvtColor(right_image, cv.COLOR_RGB2GRAY)
left_image_gray = cv.cvtColor(left_image, cv.COLOR_RGB2GRAY)

cv.imshow("right", right_image)
cv.waitKey(100)
cv.imshow("left", left_image)
cv.waitKey(100)

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
right_image = cv.cvtColor(right_image, cv.COLOR_RGB2GRAY)
right_image = cv.cvtColor(right_image, cv.COLOR_GRAY2RGB)
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
cv.waitKey(0)

# Create a 4D blob from a frame.
blobL = cv.dnn.blobFromImage(left_image, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)
left_image = cv.cvtColor(left_image, cv.COLOR_RGB2GRAY)
left_image = cv.cvtColor(left_image, cv.COLOR_GRAY2RGB)
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
cv.waitKey(0)

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

# TODO: Find Matches between found cones
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

print(centerL)
print(boxesL)
print(centerR)
print(boxesR)



# TODO: Once Matches have been found compute cone locations
# triangulate points