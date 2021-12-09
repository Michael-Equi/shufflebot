import cv2
from utils import *
from homography_from_mouse import *

# load the image
img = cv2.imread("./headcam.jpg", 1)
# img = cv2.imread("./headcam_blackboard.jpg", 1)

corner_points = findCorners(img)

warped = four_point_transform(img, corner_points)

# cv2.imshow("warped", warped)
# warped = homoFromMouse(img)

blue_pucks, red_pucks =find_pucks(warped)

blue_pucks_real = findDistances(warped.shape[0:2], (8, 2), blue_pucks)

red_pucks_real = findDistances(warped.shape[0:2], (8, 2), red_pucks)

print(blue_pucks_real)
print(red_pucks_real)

