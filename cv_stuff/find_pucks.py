# import the necessary packages
import cv2
import numpy as np
# construct the argument parser and parse the arguments

def farFromPoint(x, points):
    epsilon = 5
    for point in points:
        distance = np.sqrt((x[1]-point[1])**2 + (x[0]-point[0])**2)
        #print(distance)
        if distance < epsilon:
            print(distance)
            return False
    else:
        return True

def find_pucks(image):
    # lower = [0,0,0]#[20, 20, 20]
    # upper = [150, 150, 150]
    # lower = np.array(lower, dtype="uint8")
    # upper = np.array(upper, dtype="uint8")
    # mask = cv2.inRange(image, lower, upper)
    # masked0 = cv2.bitwise_and(image, image, mask=mask)
    

    # convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #h,s,v = cv2.split(hsv)
    # red is 0 in range 0 to 360; so half in OpenCV
    # blue is 240 in range 0 to 360; so half in OpenCV
    lower_red = 0
    upper_red = 30 /2
    lower_red2 = 330 /2
    upper_red2 = 360 /2
    lower_blue = 210 /2
    upper_blue = 270 /2

    mask0 = cv2.inRange(hsv, (lower_red, 10, 30), (upper_red, 255,255 ))
    mask1 = cv2.inRange(hsv, (lower_red2, 30, 50), (upper_red2, 255,255 ))
    mask_blue = cv2.inRange(hsv, (lower_blue, 10, 10), (upper_blue, 255,255 ))
    mask_red = cv2.bitwise_or(mask0, mask1)
    # mask = cv2.bitwise_or(mask_blue, mask_red)
    mask = cv2.inRange(hsv, (0, 0, 0), (255, 255, 100))
    #(lower_red, lower_red2, lower_blue), (upper_red, upper_red2, upper_blue))
    masked = cv2.bitwise_and(image, image, mask=mask)
    cv2.imshow('Mask', mask_red)
    cv2.waitKey(0)
    cv2.imshow('Mask1', mask_blue)
    cv2.waitKey(0)
    cv2.imshow('Masked Image', masked)
    cv2.waitKey(0)


    ### Find Blue contours and centroids
    ret,thresh = cv2.threshold(mask_blue, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blue_pucks = []
    if len(contours) != 0:
        # draw in blue the contours that were founded
        # cv2.drawContours(image, contours, -1, 255, 3)
        # cv2.imshow("contours", image)
        # cv2.waitKey(0)

        for c in contours:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            blue_pucks.append((cX, cY))
            # draw the contour and center of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 0, (0, 0, 255), thickness=3)
            cv2.putText(image, "center", (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # show the image

        # print("blue:", blue_pucks)
        # cv2.imshow("Image", image)
        # cv2.waitKey(0)

    ### Find all contours and centroids
    ret,thresh = cv2.threshold(mask, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    all_pucks = []
    if len(contours) != 0:
        # draw in blue the contours that were founded
        # cv2.drawContours(image, contours, -1, 255, 3)
        # cv2.imshow("contours", image)
        # cv2.waitKey(0)

        
        for c in contours:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            all_pucks.append((cX, cY))
            # draw the contour and center of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 0, (0, 0, 255), thickness=3)
            cv2.putText(image, "center", (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # print("all:", all_pucks)
        # show the image
        # cv2.imshow("Image", image)
        # cv2.waitKey(0)

    red_pucks = list(filter((lambda x: farFromPoint(x, blue_pucks)), all_pucks))
    # print("red:" , red_pucks)
    return [blue_pucks, red_pucks]
    