import cv2
import numpy as np
from four_point_transform import four_point_transform

# reading the image
img = cv2.imread("trapezoid.png", 1)



def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        event_point = (x, y)
        params.append(event_point)
        print(points)
        cv2.circle(img, event_point, radius=0, color=(0, 0, 255), thickness=20)

# displaying the image
cv2.imshow('image', img)
points = []
# setting mouse handler for the image
# and calling the click_event() function
cv2.setMouseCallback('image', click_event, points)  
cv2.waitKey(0)
cv2.destroyAllWindows()

warped = four_point_transform(img, np.array(points))
cv2.imshow("Original", img)
cv2.imshow("Warped", warped)
cv2.waitKey(0)