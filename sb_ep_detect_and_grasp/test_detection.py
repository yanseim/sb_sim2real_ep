#!/bin/python3
from operator import is_not
import rospy
import numpy as np
import cv2
# from utils import ARUCO_DICT
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import sys

from redmarkerdetection import *    # image processing by cython

from scipy.spatial.transform import Rotation as R



def main():
    load_template()
    cv_image = cv2.imread("image.png")
    (h, w, channel) = cv_image.shape

    cv2.imwrite('image.png',cv_image)

    seg_papram = np.array([0,15,125,180,46,80],dtype="uint8")

    id_list,tvec_list,rvec_list,idx_chosen_to_pub = marker_detection(cv_image,seg_papram)

    cv2.imshow('frame', cv_image)
    cv2.waitKey(3000)



if __name__ == '__main__':
    main()
