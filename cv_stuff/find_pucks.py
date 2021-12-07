import numpy as np
import cv2

from four_point_transform import *

# load the image

#  color boundaries [B, G, R] # 158, 192, 212

img = cv2.imread("headcam.jpg", 1)
# img = cv2.imread("trapezoid.png", 1)

if False:#for blue
    lower = np.array([55, 25, 25])
    upper = np.array([100, 50, 50])

if True:#for red
    lower = np.array([30, 30, 85])
    upper = np.array([100, 100, 200])

mask = cv2.inRange(img, lower, upper)
output = cv2.bitwise_and(img, img, mask=mask)

ret,thresh = cv2.threshold(mask, 40, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


if len(contours) != 0:
    # draw in blue the contours that were founded
    cv2.drawContours(output, contours, -1, 255, 3)

    # find the biggest countour (c) by the area
    c = max(contours, key = cv2.contourArea)
    
    x,y,w,h = cv2.boundingRect(c)

    # draw the biggest contour (c) in green
    cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

# show the images
#cv2.drawContours(img, c, -1, (0, 0, 255), 3)
cv2.imshow("Result", np.hstack([img, output]))

cv2.waitKey(0)
