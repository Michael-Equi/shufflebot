import cv2
import numpy as np
from utils import *

def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        event_point = [x, y]
        print(event_point)
        params.append(event_point)
        #cv2.circle(img, event_point, radius=0, color=(0, 0, 255), thickness=20)
        

def homoFromMouse(img):
    # displaying the image
    mult = 1
    # imgS = cv2.resize(img, (img.shape[1] / mult, img.shape[0] / mult))
    cv2.imshow('image', img)
    points = []
    # setting mouse handler for the image
    # and calling the click_event() function
    cv2.setMouseCallback('image', click_event, points)  
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return np.array(points) * mult
