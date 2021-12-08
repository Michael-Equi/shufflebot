# import the necessary packages
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import cv2
import numpy as np
# construct the argument parser and parse the arguments



def run_clustering(image, clusters):
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
    upper_red = 60 /2
    lower_red2 = 300 /2
    upper_red2 = 360 /2
    lower_blue = 180 /2
    upper_blue = 270 /2

    mask0 = cv2.inRange(hsv, (lower_red, 0, 0), (upper_red, 255,255 ))
    mask1 = cv2.inRange(hsv, (lower_red2, 0, 0), (upper_red2, 255,255 ))
    mask2 = cv2.inRange(hsv, (lower_blue, 0, 0), (upper_blue, 255,255 ))
    mask = cv2.bitwise_or(mask0, cv2.bitwise_or(mask1, mask2))
    #(lower_red, lower_red2, lower_blue), (upper_red, upper_red2, upper_blue))
    masked = cv2.bitwise_and(hsv, hsv, mask=mask)

    
    cv2.imshow('Masked Image', masked)
    cv2.waitKey(0)
    #image = cv2.cvtColor(masked, cv2.COLOR_HSV2RGB)
    # show our image
    # plt.figure()
    # plt.axis("off")
    # plt.imshow(masked)

    print(masked.shape)
    # reshape the image to be a list of pixels
    masked = masked.reshape((masked.shape[0] * masked.shape[1], 3))

    # cluster the pixel intensities
    clt = KMeans(n_clusters = clusters)
    clt.fit(masked)

    # segmented = clt.cluster_centers_[clt.labels_]
    # segmented_image = segmented.reshape((image.shape))
    segmented = np.zeros((len(clt.labels_), 3))
    for i in range(len(clt.labels_)):
        #[B G R]
        if clt.labels_[i] == 0:
            segmented[i] = [0,0,0]
        if clt.labels_[i] == 1:
            segmented[i] = [0,0,255]
        if clt.labels_[i] == 2:
            segmented[i] = [255, 0, 0]
        if clt.labels_[i] == 3:
            segmented[i] = [0,255,0]
    segmented_image = segmented.reshape((image.shape))
    cv2.imshow("segmented", segmented_image)
    cv2.waitKey(0)