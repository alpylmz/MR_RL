# https://towardsdatascience.com/image-color-segmentation-by-k-means-clustering-algorithm-5792e563f26e

import cv2
import numpy as np


img = cv2.imread("frame.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

for i in range(len(img)):
    for j in range(len(img[i])):
        if hsv[i][j][2] < 90:
            hsv[i][j] = [0, 0, 0]

cv2.imshow("img", img)
cv2.imshow("hsv", hsv)
cv2.waitKey(0)
cv2.destroyAllWindows()

exit(2)
_ , mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
mask = cv2.erode(mask, np.ones((7, 7), np.uint8))

cv2.imshow("mask", mask)
cv2.imshow("gray", gray)
cv2.waitKey(0)
cv2.destroyAllWindows()

