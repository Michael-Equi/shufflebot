# import the necessary packages
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import cv2
import numpy as np
# construct the argument parser and parse the arguments



def run_clustering(image, clusters):
    # lower = [0,0,0]#[20, 20, 20]
    # upper = [200, 200, 200]#[130, 130, 130]
    # lower = np.array(lower, dtype="uint8")
    # upper = np.array(upper, dtype="uint8")
    # extract alpha channel
    alpha = image[:,:,3]

    # extract bgr channels
    bgr = image[:,:,0:3]

    # convert to HSV
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    # red is 0 in range 0 to 360; so half in OpenCV
    # blue is 240 in range 0 to 360; so half in OpenCV
    lower_red = 0
    blue = 120

    mask = cv2.inRange(image, lower, upper)
    masked = cv2.bitwise_and(image, image, mask=mask)
    image = cv2.cvtColor(masked, cv2.COLOR_BGR2RGB)
    # show our image
    plt.figure()
    plt.axis("off")
    plt.imshow(masked)

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