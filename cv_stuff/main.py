import cv2
from utils import *
#from homography_from_mouse import *


# load the image
img = cv2.imread("./headcam.jpg", 1)
# img = cv2.imread("./headcam_blackboard.jpg", 1)

corner_points = findCorners(img)

# for corner in corner_points:
#     cv2.circle(img, tuple(corner), radius=0, color=(0, 0, 255), thickness=5)
# cv2.imshow("circled", img)
# cv2.waitKey(0)

# warped = homoFromMouse(img)
warped = four_point_transform(img, corner_points)
# cv2.imshow("warped", warped)
# warped = homoFromMouse(img)

blue_pucks, red_pucks = find_pucks(warped)

blue_pucks_real = findDistances(warped.shape[0:2], (2.4384, 0.4064), blue_pucks)

red_pucks_real = findDistances(warped.shape[0:2], (2.4384, 0.4064), red_pucks)

print(blue_pucks_real)
print(red_pucks_real)

