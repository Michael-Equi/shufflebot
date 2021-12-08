import numpy as np
import cv2

# load the image

#  color boundaries [B, G, R] # 158, 192, 212


def findCorners(img):
    # img = cv2.imread("orig_headcam.jpg", 1)
    #lower = [100, 130, 130]
    lower = [90, 105, 110]
    upper = [255, 255, 255]

    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(img, lower, upper)
    output = cv2.bitwise_and(img, img, mask=mask)

    ret,thresh = cv2.threshold(mask, 40, 255, 0)
    #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    if len(contours) != 0:
        # draw in blue the contours that were founded
        cv2.drawContours(output, contours, -1, 255, 3)

        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        
        x,y,w,h = cv2.boundingRect(c)
        #print("aspect ratio", h/w)

        if h/w > 1.05 or h/w < .95:
            c = max(filter(lambda x: (x not in c), contours), key = cv2.contourArea)
            if len(contours) != 0:
                x,y,w,h = cv2.boundingRect(c)
                print("aspect ratio", h/w)
        # draw the biggest contour (c) in green
        cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

    # show the images
    #cv2.drawContours(img, c, -1, (0, 0, 255), 3)
    #cv2.imshow("Result", np.hstack([img, output]))
    #cv2.waitKey(0)

    perim = cv2.arcLength(c, True)
    epsilon = 0.02*perim
    approxCorners = cv2.approxPolyDP(c, epsilon, True)
    approxCornersNumber = len(approxCorners)
    print("Number of approximated corners: ", approxCornersNumber)
    # print("Coordinates of approximated corners:\n", approxCorners)
    corner_points = np.array([[x[0][0], x[0][1]] for x in approxCorners])
    # print("input" ,(corner_points))
    return corner_points
    
#corner_points = np.array([[362, 580], [789, 304], [613, 720], [362, 580]])

