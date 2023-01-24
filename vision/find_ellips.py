import cv2
import numpy as np
import pickle
import copy

from kmeansinfo import KmeansInfo

kmeans_info = pickle.load(open("kmeans_info.pkl", "rb"))

img = cv2.imread("images/clustered_frame_23_jan.png")
# God please forgive
# Make it better later TODO
blue_masked_img = copy.deepcopy(img)
yellow_masked_img = copy.deepcopy(img)
red_masked_img = copy.deepcopy(img)
black_img = copy.deepcopy(img)


# mask out every color other than yellow
for i in range(len(img)):
    for j in range(len(img[i])):
        # currently these are the values set in the kmeans_train for yellow!
        if yellow_masked_img[i][j][0] == 0 and \
            yellow_masked_img[i][j][1] == 255 and \
            yellow_masked_img[i][j][2] == 255:
            
            yellow_masked_img[i][j] = [0, 0, 0]
        else:
            yellow_masked_img[i][j] = [255, 255, 255]

# mask out every color other than blue
for i in range(len(img)):
    for j in range(len(img[i])):
        # currently these are the values set in the kmeans_train for blue!
        if blue_masked_img[i][j][0] == 255 and \
            blue_masked_img[i][j][1] == 0 and \
            blue_masked_img[i][j][2] == 0:
            
            blue_masked_img[i][j] = [0, 0, 0]
        else:
            blue_masked_img[i][j] = [255, 255, 255]

# mask out every color other than red
for i in range(len(img)):
    for j in range(len(img[i])):
        # currently these are the values set in the kmeans_train for red!
        if red_masked_img[i][j][0] == 0 and \
            red_masked_img[i][j][1] == 0 and \
            red_masked_img[i][j][2] == 255:
            
            red_masked_img[i][j] = [0, 0, 0]
        else:
            red_masked_img[i][j] = [255, 255, 255]

for i in range(len(img)):
    for j in range(len(img[i])):
        black_img[i][j] = [0, 0, 0] # :(

for i in range(len(img)):
    for j in range(len(img[i])):
        # some small changes for image readability
        if yellow_masked_img[i][j][0] == 0:
            img[i][j] = [0, 127, 127]
        if blue_masked_img[i][j][0] == 0:
            img[i][j] = [127, 0, 0]
        if red_masked_img[i][j][0] == 0:
            img[i][j] = [0, 0, 127]


"""
# set our filtering parameters
# initialize parameter settiing using cv2.SimpleBlobDetector
params = cv2.SimpleBlobDetector_Params()

# Set area filtering parameters
params.filterByArea = True
params.minArea = 10

# Set circularity filtering parameters
params.filterByCircularity = True
params.minCircularity = 0.4

# Set convexity filtering parameters
params.filterByConvexity = True
params.minConvexity = 0.2

# Set inertia filtering parameters
params.filterByInertia = True
params.minInertiaRatio = 0.1

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

# Detect blobs.
keypoints = detector.detect(img)

# Draw detected blobs as red circles.
im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0, 0, 255))
"""

# find countours
yellow_masked_img = cv2.cvtColor(yellow_masked_img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(yellow_masked_img, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    try:
        ellipse = cv2.fitEllipse(contour)
    except cv2.error:
        continue

    # draw that into the img
    cv2.ellipse(img, ellipse, (0, 255, 255), 1)


# find countours
blue_masked_img = cv2.cvtColor(blue_masked_img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(blue_masked_img, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    try:
        ellipse = cv2.fitEllipse(contour)
    except cv2.error:
        continue

    # draw that into the img
    cv2.ellipse(img, ellipse, (255, 0, 0), 1)

# find countours
red_masked_img = cv2.cvtColor(red_masked_img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(red_masked_img, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    try:
        ellipse = cv2.fitEllipse(contour)
    except cv2.error:
        continue

    # draw that into the img
    cv2.ellipse(img, ellipse, (0, 0, 255), 1)


# Show keypoints
cv2.imshow("Original", img)
cv2.imshow("Ellipsed", black_img)
#cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

