import cv2
import homography_from_mouse
from utils import *
from homography_from_mouse import homoFromMouse
# from homography_from_mouse import *

def get_real_board_state(length, width, pixel_height=800, pixel_width=150):
    # load the image
    img = getImage()
    
    # median = cv2.medianBlur(img,13)
    corner_points = findCorners(img)
    while len(corner_points) != 4:
        img = getImage()
        corner_points = homoFromMouse(img)
    
    # for corner in corner_points:
    #     cv2.circle(img, tuple(corner), radius=0, color=(0, 0, 255), thickness=10)
    # imgS = cv2.resize(img, (960, 540))
    # cv2.imshow("circled", img)
    # cv2.waitKey(0)
    # print(corner_points)
    warped = four_point_transform(img, corner_points, pixel_height, pixel_width)
    # warpedS = cv2.resize(warped, (150, 800))
    cv2.imshow("warped", warped)
    cv2.waitKey(0)
    
    blue_pucks, red_pucks = find_pucks(warped)
    ### filter pucks too close to shooter
    blue_pucks = filter(lambda x: x[1] < (2. * pixel_height / 3.), blue_pucks)
    red_pucks = filter(lambda x: x[1] < (2. * pixel_height / 3.), red_pucks)


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
    # print(blue_pucks_real, red_pucks_real)
    return blue_pucks_real, red_pucks_real

if __name__=="__main__":
    pixel_height = 800
    pixel_width = 150
    length = 2.4384
    width = 0.4572
    get_real_board_state(length, width)
