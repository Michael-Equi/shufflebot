import numpy as np
import cv2
from utils import *

warped = cv2.imread("warped.jpg", cv2.IMREAD_UNCHANGED)

find_pucks(warped)