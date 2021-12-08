import cv2
from find_pucks import *
from four_point_transform import *
from shuffle_homo_trans import *
from findRealDistances import *

# load the image
img = cv2.imread("headcam.jpg", 1)

corner_points = findCorners(img)

warped = four_point_transform(img, corner_points)

#cv2.imwrite("warped.jpg", warped)

# cv2.imshow("Original", img)
# cv2.imshow("Warped", warped)
# cv2.waitKey(0)

blue_pucks, red_pucks =find_pucks(warped)
print("blue",blue_pucks)

print("red",red_pucks)
print(warped.shape[0:2])
blue_pucks_real = findDistances(warped.shape[0:2], (8, 2), blue_pucks)

red_pucks_real = findDistances(warped.shape[0:2], (8, 2), red_pucks)

print("real blue",blue_pucks_real)

print("real red",red_pucks_real)
