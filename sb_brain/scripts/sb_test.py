#!/bin/python3

import rospy
import numpy as np
import cv2
import math
import os
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose,Point
from nav_msgs.msg import Odometry
import tf
import tf2_ros
import tf2_geometry_msgs
import time
import roslib
import sys

from sb_ep_detect_and_grasp.srv import grasp_place

from scipy.spatial.transform import Rotation as R

cube_positions = [[1.00058174,0.09497403,3.39986682,-0.00076221,-0.0013086464023217559,-0.0012807715684175491,0.9999980330467224],
   [1.90030038356781,
   0.08630374819040298,
   3.099881172180176,
   -0.0009147212258540094,
   -0.0007257394026964903,
   -0.004962262697517872,
   0.999987006187439],
[1.5008326768875122,
   0.08552967756986618,
   0.998009443283081,
   -0.0018889335915446281,
   0.02178667113184929,
   0.0013308123452588916,
   0.9997599720954895],
[3.999114990234375,
   0.09183745086193085,
   1.8005446195602417,
   0.0003158680337946862,
   0.004514407832175493,
   0.0019894992001354694,
   0.999987781047821],
[5.099486827850342,
   0.0978584736585617,
   0.3004781901836395,
   0.0014465743442997336,
   0.0008184657199308276,
   0.0041693891398608685,
   0.999989926815033]]


class Brain(object):
    def __init__(self):
        rospy.loginfo("initing!!!!!!!!")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_broadcaster =tf2_ros.TransformBroadcaster()

        rospy.Subscriber('/pose/cube_1',Pose,self.cube1_pose_callback)

    def cube1_pose_callback(self,msg):
        if not self.tf_buffer.can_transform("odom","map",rospy.Time.now()):
            rospy.logwarn("can not find world tf!!cube1 callback")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = msg
        pose_stamped = tf2_geometry_msgs.PoseStamped(pose_stamped.header,pose_stamped.pose)
        pose_stamped_in_map = self.tf_buffer.transform(pose_stamped,"map")
        self.cube1_pose_stamped = pose_stamped_in_map

        # publish tf for cube1
        cube1_transform_stamp = tf2_ros.TransformStamped()
        cube1_transform_stamp.header.stamp = rospy.Time.now()
        cube1_transform_stamp.header.frame_id = "map"
        cube1_transform_stamp.child_frame_id = "cube1"
        cube1_transform_stamp.transform.translation.x = pose_stamped_in_map.pose.position.x
        cube1_transform_stamp.transform.translation.y = pose_stamped_in_map.pose.position.y
        cube1_transform_stamp.transform.translation.z = pose_stamped_in_map.pose.position.z
        cube1_transform_stamp.transform.rotation.x = pose_stamped_in_map.pose.orientation.x
        cube1_transform_stamp.transform.rotation.y = pose_stamped_in_map.pose.orientation.y
        cube1_transform_stamp.transform.rotation.z = pose_stamped_in_map.pose.orientation.z
        cube1_transform_stamp.transform.rotation.w = pose_stamped_in_map.pose.orientation.w

        self.tf_broadcaster.sendTransform(cube1_transform_stamp)
        rospy.loginfo("cube1 tf has been published!")

    def main(self):
        time.sleep(0.5)
        rate = rospy.Rate(30)
        self.publish_nav_goal()
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('brain_node')
    brain = Brain()

    rospy.sleep(0.5)
    while 1:
        if not brain.tf_buffer.can_transform("odom","map",rospy.Time.now()):
            rospy.logwarn("fuck!!!!!!!")

