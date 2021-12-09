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

    blue_pucks, red_pucks = find_pucks(warped)

    blue_pucks_real = findDistances(warped.shape[0:2], (8, 2), blue_pucks)

    red_pucks_real = findDistances(warped.shape[0:2], (8, 2), red_pucks)

    return blue_pucks_real, red_pucks_real

