import numpy as np
import cv2
from find_pucks import *
from shuffle_homo_trans import *

warped = cv2.imread("warped.jpg", cv2.IMREAD_UNCHANGED)

find_pucks(warped)