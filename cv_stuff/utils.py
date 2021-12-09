import numpy as np
import cv2
import itertools



### Takes in four points and orders them with first top-left,
### the second entry is the top-right, the third is the
### bottom-right, and the fourth is the bottom-left
def order_points(img, pts):
    # # initialzie a list of coordinates that will be ordered
    # # such that the first entry in the list is the top-left,
    # # the second entry is the top-right, the third is the
    # # bottom-right, and the fourth is the bottom-left
    # rect = np.zeros((4, 2), dtype = "float32")
    # # the top-left point will have the smallest sum, whereas
    # # the bottom-right point will have the largest sum
    # s = pts.sum(axis = 1)
    # rect[0] = pts[np.argmin(s)]
    # rect[2] = pts[np.argmax(s)]
    # # now, compute the difference between the points, the
    # # top-right point will have the smallest difference,
    # # whereas the bottom-left will have the largest difference
    # diff = np.diff(pts, axis = 1)
    # rect[1] = pts[np.argmin(diff)]
    # rect[3] = pts[np.argmax(diff)]

    pts = [x for x in pts]
    pts.sort(key=lambda x: x[1])
    pts_bot = pts[2:]
    pts_bot.sort(key=lambda x: x[0])
    pts_top = pts[:2]
    pts_top.sort(key=lambda x: -x[0])
    pts_top.extend(pts_bot)
    rect = np.array(pts_top, dtype = "float32")

    # # return the ordered coordinates
    # top_left = np.array([0, 0])
    # top_right = np.array([img.shape[1], 0])
    # bottom_left = np.array([0, img.shape[0]])
    # bottom_right = np.array([img.shape[1], img.shape[0]])

    # targets = np.array([top_left, top_right, bottom_right, bottom_left])
    # #print(pts.shape)
    # perms = np.array(list(itertools.permutations(pts)))
    # #print(perms.shape, targets.shape)
    # best_combo = np.argmin(np.sum(np.linalg.norm(perms - targets, axis=1), axis=1))
    # rect = np.array(perms[best_combo], dtype = "float32")
    # return the ordered coordinates
    return rect
	
### Takes in four points and image and returns warped 
### image transformed to top down view
def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(image, pts)
    (tl, tr, br, bl) = rect
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    # print("maskldfjklsd")
    # print("rect:", rect)
    # print(dst)
    # print(M)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    HEIGHT = 800
    WIDTH = 133
    resized = cv2.resize(warped, (WIDTH, HEIGHT))
    # return the warped image
    return resized





### Given image dimensions (pixels), real world dimensions (m), and list of points 
### find distance to each point from bottom right corner in real units
###  (HEIGHT, WIDTH)
def findDistances(imageDimesions, realDimensions, points):
    heightConversion =  float(realDimensions[0]) / float(imageDimesions[0]) #meter / pixel
    widthConversion =  float(realDimensions[1]) / float(imageDimesions[1]) #meter / pixel
    convertedPoints = [(0. , 0.)]*len(points)
    for i in range(len(points)):
        x =  abs(points[i][1] - imageDimesions[0]) * heightConversion
        y = abs(points[i][0] - imageDimesions[1]) * widthConversion
        convertedPoints[i] = (x, y)
    return convertedPoints



### Find Corner Points on a shuffleboard table
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

    # cv2.imshow("Result", output)
    # cv2.waitKey(0)

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
        # draw the biggest contour (c) in green
        cache = output.copy()
        cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
        #cv2.drawContours(img, c, -1, (0, 0, 255), 3)
        cv2.imshow("Result", output)
        cv2.waitKey(10)
        #check if the correct contour
        user_in = raw_input("Is this correct? y or n:\n")
        #if h/w > 1.05 or h/w < .95:
        while user_in != "y":
            output = cache.copy()
            contours = filter(lambda x: (x not in c), contours)
            c = max(contours, key = cv2.contourArea)
            if len(contours) != 0:
                x,y,w,h = cv2.boundingRect(c)
                #print("aspect ratio", h/w)
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)
            #cv2.drawContours(img, c, -1, (0, 0, 255), 3)
            cv2.imshow("Result", output)
            cv2.waitKey(10)
            user_in = raw_input("Is this correct? y or n:\n")
    # show the images
    # cv2.drawContours(img, c, -1, (0, 0, 255), 3)
    # cv2.imshow("Result", np.hstack([img, output]))
    # cv2.waitKey(0)
   

    perim = cv2.arcLength(c, True)
    epsilon = 0.02*perim
    approxCorners = cv2.approxPolyDP(c, epsilon, True)
    # print(approxCorners)
    # approxCornersNumber = len(approxCorners)
    # print("Number of approximated corners: ", approxCornersNumber)
    # print("Coordinates of approximated corners:\n", approxCorners)

    # corner_points = np.zeros(len(approxCorners), dtype='f,f')
    # for i in range(len(approxCorners)):
    #     corner_points[i] = [approxCorners[i][0][0], approxCorners[i][0][1]]
    corner_points = np.array([[x[0][0], x[0][1]] for x in approxCorners])
    # print("input" ,(corner_points))
    return corner_points







### Helper for find_pucks
def farFromPoint(x, points, epsilon=5):
    for point in points:
        distance = np.sqrt((x[1]-point[1])**2 + (x[0]-point[0])**2)
        #print(distance)
        if distance < epsilon:
            #print(distance)
            return False
    else:
        return True


### Find centroids of pucks in a warped image of shuffleboard table
def find_pucks(image):
    cv2.imshow("image", image)
    cv2.waitKey(0)
    # convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
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
    # cv2.imshow('Mask', mask_red)
    # cv2.waitKey(0)
    # cv2.imshow('Mask1', mask_blue)
    # cv2.waitKey(0)
    # cv2.imshow('Masked Image', masked)
    # cv2.waitKey(0)


    ### Find Blue contours and centroids
    ret,thresh = cv2.threshold(mask_blue, 40, 255, 0)
    #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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
    #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
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

    red_pucks = list(filter((lambda x: farFromPoint(x, blue_pucks, epsilon=5)), all_pucks))
    # print("red:" , red_pucks)
    return [blue_pucks, red_pucks]
    
def getImage():
    img = cv2.imread("headcam.jpg", 1)
    return img