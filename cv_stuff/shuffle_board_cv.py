import cv2
import homography_from_mouse
from utils import *
from homography_from_mouse import homoFromMouse
# from homography_from_mouse import *

def get_real_board_state(length, width):
    # load the image
    img = getImage()
    # img = cv2.imread("./headcam_blackboard.jpg", 1)
    median = cv2.medianBlur(img,13)

    
    corner_points = findCorners(median)
    if len(corner_points) == 4:
        # for corner in corner_points:
        #     cv2.circle(img, tuple(corner), radius=0, color=(0, 0, 255), thickness=5)
        # imgS = cv2.resize(img, (960, 540))
        # cv2.imshow("circled", imgS)
        # cv2.waitKey(0)
        warped = four_point_transform(median, corner_points)
        # cv2.imshow("warped", warped)
        # cv2.waitKey(0)
    else:
        warped = homoFromMouse(median)

    blue_pucks, red_pucks = find_pucks(warped)

    # for blue in blue_pucks:
    #     cv2.circle(warped, tuple(blue), radius=0, color=(0, 0, 255), thickness=5)
    # warpedS = cv2.resize(warped, (150, 800))
    # cv2.imshow("circled", warpedS)
    # cv2.waitKey(0)

    # for red in red_pucks:
    #     cv2.circle(warped, tuple(red), radius=0, color=(0, 0, 255), thickness=5)
    # warpedS = cv2.resize(warped, (150, 800))
    # cv2.imshow("circled", warpedS)
    # cv2.waitKey(0)



    blue_pucks_real = findDistances(warped.shape[0:2], (length, width), blue_pucks)

    red_pucks_real = findDistances(warped.shape[0:2], (length, width), red_pucks)
    print(blue_pucks_real, red_pucks_real)
    return blue_pucks_real, red_pucks_real

length = 2.4384
width = 0.4572
get_real_board_state(length, width)
