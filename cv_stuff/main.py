import numpy as np
import cv2
from find_pucks import *
from four_point_transform import *
from shuffle_homo_trans import *

# load the image
img = cv2.imread("headcam.jpg", 1)

corner_points = findCorners(img)

warped = four_point_transform(img, corner_points)

cv2.imwrite("warped.jpg", warped)
# Draw corners on the Polygon
# for i in range(len(corner_points)):
#     img1 = cv2.circle(img, corner_points[i], radius=0, color=(0, 0, 255), thickness=10)
#cv2.imshow("Points", img1)

#cv2.waitKey(0)

cv2.imshow("Original", img)
cv2.imshow("Warped", warped)
cv2.waitKey(0)

find_pucks(warped)
