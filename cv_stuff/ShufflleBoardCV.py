from utils import *
# from homography_from_mouse import *

def get_real_board_state():
    # load the image
    img = getImage()
    # img = cv2.imread("./headcam_blackboard.jpg", 1)

    corner_points = findCorners(img)

    warped = four_point_transform(img, corner_points)

    # cv2.imshow("warped", warped)
    # warped = homoFromMouse(img)


    # for corner in corner_points:
    #     cv2.circle(img, tuple(corner), radius=0, color=(0, 0, 255), thickness=5)
    # cv2.imshow("circled", img)
    # cv2.waitKey(0)


    blue_pucks, red_pucks = find_pucks(warped)

    blue_pucks_real = findDistances(warped.shape[0:2], (8, 2), blue_pucks)

    red_pucks_real = findDistances(warped.shape[0:2], (8, 2), red_pucks)
    print(blue_pucks_real, red_pucks_real)
    return blue_pucks_real, red_pucks_real
#get_real_board_state()
