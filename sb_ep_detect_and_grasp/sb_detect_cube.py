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

def pose_aruco_2_ros(rvec, tvec):
    aruco_pose_msg = Pose()
    aruco_pose_msg.position.x = tvec[0]
    aruco_pose_msg.position.y = tvec[1]
    aruco_pose_msg.position.z = tvec[2]
    rot = R.from_rotvec(np.reshape(rvec,(3,)))
    quat = rot.as_quat()
    aruco_pose_msg.orientation.x = quat[0]
    aruco_pose_msg.orientation.y = quat[1]
    aruco_pose_msg.orientation.z = quat[2]
    aruco_pose_msg.orientation.w = quat[3]
    return aruco_pose_msg

class arucoPose:
    def __init__(self):
        self.aruco_pose_pub = rospy.Publisher("aruco_pose", Pose)
        
        self.aruco_sink1_pub = rospy.Publisher("aruco_sink1", Pose)
        self.aruco_sink2_pub = rospy.Publisher("aruco_sink2", Pose)
        self.aruco_sink3_pub = rospy.Publisher("aruco_sink3", Pose)

        self.aruco_pose_image_pub = rospy.Publisher("aruco_pose_image", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageColorCallback)

        self.MARKER_SIZE = 0.045 # [m]
        self.ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.MTX = np.array([[617.3054000792732, 0.0, 424.0], 
                        [0.0, 617.3054000792732, 240.0], 
                        [0.0, 0.0, 1.0]])# camera intrinsic parameters!!
        self.DIST = np.array([0., 0., 0., 0., 0.])

        self.id_list = []
        self.tvec_list = []
        self.rvec_list = []

        load_template()

    def imageColorCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)
        
        (h, w, channel) = cv_image.shape

        seg_papram = np.array([0,15,125,180,46,80],dtype="uint8")

        id_list,tvec_list,rvec_list,idx_chosen_to_pub = marker_detection(cv_image,seg_papram)

        cv2.imshow('frame', cv_image)
        cv2.waitKey(1)

# =============================================================
        # # publish pose to rostopic
        # target_detected = False
        # sink1_detected = False
        # sink2_detected = False
        # sink3_detected = False
        # field_top = -0.15

        # min_y = 1
        # for i in range(len(id_list)):
        #     if id_list[i] == 0 or id_list[i] == 1 or id_list[i] == 2 or id_list[i] == 6  or id_list[i] == 7 and rvec_list[i][1]>-0.5:
        #         if min_y>rvec_list[i][1]:
        #             min_y = rvec_list[i][1]

        # id_list_chosen = []
        # tvec_list_chosen = []
        # rvec_list_chosen = []
        # for i in range(len(id_list)):
        #     if id_list[i] == 0 or id_list[i] == 1 or id_list[i] == 2 or id_list[i] == 6  or id_list[i] == 7 and target_detected == False:
        #         # choose cube to publish
        #         aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])
        #         if aruco_pose_msg.position.x < 0.5 and aruco_pose_msg.position.x > -0.5 and \
        #             aruco_pose_msg.position.y < 0.02 and aruco_pose_msg.position.y > field_top and \
        #             aruco_pose_msg.position.z < 1 and \
        #             aruco_pose_msg.position.y < min_y+0.04:
        #             id_list_chosen.append(id_list[i])
        #             tvec_list_chosen.append(tvec_list[i])
        #             rvec_list_chosen.append(rvec_list[i])


        # norm_epsilon = 2
        # while len(id_list_chosen)>1:
        #     for i in range(len(id_list_chosen)):
        #         rot = R.from_rotvec(np.reshape(rvec_list_chosen[i],(3,)))
        #         R1 = rot.as_matrix()
        #         Ry1 = np.array([[0,0,-1],[0,1,0],[1,0,0]])
        #         Ry2 = np.array([[0,0,1],[0,1,0],[-1,0,0]])
        #         for k in range(len(id_list_chosen)):
        #             min_norm = 10
        #             if i != k:
        #                 R2 = R.from_rotvec(np.reshape(rvec_list_chosen[k],(3,))).as_matrix()
        #                 if min_norm > min(np.linalg.norm(Ry1@R1-R2),np.linalg.norm(Ry2@R1-R2)):
        #                     min_norm = min(np.linalg.norm(Ry1@R1-R2),np.linalg.norm(Ry2@R1-R2))
        #                     k_min = k
        #         print("min_norm",min_norm)
        #         if min_norm<norm_epsilon:
        #             delete_one = i if np.linalg.norm(R1-np.eye(3))>np.linalg.norm(R.from_rotvec(np.reshape(rvec_list_chosen[k],(3,))).as_matrix()-np.eye(3)) else k
        #             del id_list_chosen[delete_one]
        #             del tvec_list_chosen[delete_one]
        #             del rvec_list_chosen[delete_one]
        #             break
        
        # if target_detected ==False:
        #     aruco_pose_msg = pose_aruco_2_ros(rvec_list_chosen[0],tvec_list_chosen[0])
        #     self.aruco_pose_pub.publish(aruco_pose_msg)
        #     target_detected = True
        #     print("id : ",id_list_chosen[0]," : ",aruco_pose_msg)

        # for i in range(len(id_list)):
        #     aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])
        #     if id_list[i] == 3 and sink1_detected == False:
        #         if aruco_pose_msg.position.y > field_top :
        #             self.aruco_sink1_pub.publish(aruco_pose_msg)
        #             sink1_detected = True
        #     if id_list[i] == 4 and sink2_detected == False:
        #         if aruco_pose_msg.position.y > field_top :
        #             self.aruco_sink2_pub.publish(aruco_pose_msg)
        #             sink2_detected = True
        #     if id_list[i] == 5 and sink3_detected == False:
        #         if aruco_pose_msg.position.y > field_top :
        #             self.aruco_sink3_pub.publish(aruco_pose_msg)
        #             sink3_detected = True

# =====================================================================yuanlaide
        # publish pose to rostopic
        target_detected = False
        sink1_detected = False
        sink2_detected = False
        sink3_detected = False
        field_top = -0.15

        if idx_chosen_to_pub==-1:
            for i in range(len(id_list)):
                aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])

                if id_list[i] == 0 or id_list[i] == 1 or id_list[i] == 2 or id_list[i] == 6  or id_list[i] == 7 and target_detected == False:
                    # choose cube to publish
                    if aruco_pose_msg.position.y < 0.02 and aruco_pose_msg.position.y > field_top:
                        self.aruco_pose_pub.publish(aruco_pose_msg)
                        target_detected = True
                        print("id : ",id_list[i]," : ",aruco_pose_msg)

                if id_list[i] == 3 and sink1_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink1_pub.publish(aruco_pose_msg)
                        sink1_detected = True
                if id_list[i] == 4 and sink2_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink2_pub.publish(aruco_pose_msg)
                        sink2_detected = True
                if id_list[i] == 5 and sink3_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink3_pub.publish(aruco_pose_msg)
                        sink3_detected = True
        else:
            for i in range(len(id_list)):
                aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])

                if id_list[i] == 3 and sink1_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink1_pub.publish(aruco_pose_msg)
                        sink1_detected = True
                if id_list[i] == 4 and sink2_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink2_pub.publish(aruco_pose_msg)
                        sink2_detected = True
                if id_list[i] == 5 and sink3_detected == False:
                    if aruco_pose_msg.position.y > field_top :
                        self.aruco_sink3_pub.publish(aruco_pose_msg)
                        sink3_detected = True
            if target_detected == False:
                aruco_pose_msg = pose_aruco_2_ros(rvec_list[idx_chosen_to_pub],tvec_list[idx_chosen_to_pub])
                self.aruco_pose_pub.publish(aruco_pose_msg)
                target_detected = True
                print("id : ",id_list[idx_chosen_to_pub]," : ",aruco_pose_msg)


def main():
    ap = arucoPose()
    rospy.init_node('aruco_pose_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destoryAllWindows()


if __name__ == '__main__':
    main()
