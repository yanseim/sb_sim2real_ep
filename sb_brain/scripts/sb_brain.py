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
    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_broadcaster =tf2_ros.TransformBroadcaster()

        self.cube1_pose_stamped = None
        self.finished = False

        self.current_loc = [0.0,0.0]

        self.rate=rospy.Rate(10)

        # publish the world coordinate
        self.publsih_world_tf()
        self.goal_reached_check = False
        self.current_goal = -1
        self.reach_goal_state = -1

        # self.publish_world_tf()
        rospy.Subscriber('/pose/cube_1',Pose,self.cube1_pose_callback)
        rospy.Subscriber('/op/odom',Odometry,self.keyLocCheck)
        rospy.Subscriber
        self.nav_goal_publisher = rospy.Publisher('/move_base_simple/goal',PoseStamped)

    # def publish_world_tf(self):
    #     trans = [4.2,0,3.5]
    #     rot_matrix = np.array([0,-1,0],[0,0,1],[-1,0,0])
    #     trans_odom2world = np.zeros(4)
    #     trans_odom2world[0:3,0:3] = rot_matrix
    #     trans_odom2world[0:3,3] = np.reshape(np.array(trans),(3,1))
    #     trans_odom2world[3,3] = 1

    #     trans_world2odom = np.linalg.inv(trans_odom2world)
    #     quat = R.from_matrix(trans_world2odom[:3,:3]).as_quat()
    #     world_transform_stamp = tf2_ros.TransformStamped()
    #     world_transform_stamp.header.stamp = rospy.Time.now()
    #     world_transform_stamp.header.frame_id = "odom"
    #     world_transform_stamp.header.child_frame_id = "world"
    #     world_transform_stamp.transform.translation.x = trans_world2odom[0,3]
    #     world_transform_stamp.transform.translation.y = trans_world2odom[1,3]
    #     world_transform_stamp.transform.translation.z = trans_world2odom[2,3]
    #     world_transform_stamp.transform.rotation.x = quat[0]
    #     world_transform_stamp.transform.rotation.y = quat[1]
    #     world_transform_stamp.transform.rotation.z = quat[2]
    #     world_transform_stamp.transform.rotation.w = quat[3]

    #     self.tf_static_broadcaster.sendTransform(world_transform_stamp)
    #     rospy.loginfo("world tf has been published!")

    def cube1_pose_callback(self,msg):
        # if not self.tf_buffer.can_transform("world","map",rospy.Time.now()):
        #     rospy.logwarn("can not find world tf")
        #     return None

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose = msg
        pose_stamped = tf2_geometry_msgs.PoseStamped(pose_stamped.header,pose_stamped.pose)
        pose_stamped_in_map = tf2_ros.transform(pose_stamped,"map")
        self.cube1_pose_stamped = pose_stamped_in_map

        # publish tf for cube1
        cube1_transform_stamp = tf2_ros.TransformStamped()
        cube1_transform_stamp.header.stamp = rospy.Time.now()
        cube1_transform_stamp.header.frame_id = "map"
        cube1_transform_stamp.header.child_frame_id = "cube1"
        cube1_transform_stamp.transform.translation.x = pose_stamped_in_map.pose.position.x
        cube1_transform_stamp.transform.translation.y = pose_stamped_in_map.pose.position.y
        cube1_transform_stamp.transform.translation.z = pose_stamped_in_map.pose.position.z
        cube1_transform_stamp.transform.rotation.x = pose_stamped_in_map.pose.orientation.x
        cube1_transform_stamp.transform.rotation.y = pose_stamped_in_map.pose.orientation.y
        cube1_transform_stamp.transform.rotation.z = pose_stamped_in_map.pose.orientation.z
        cube1_transform_stamp.transform.rotation.w = pose_stamped_in_map.pose.orientation.w

        self.tf_broadcaster.sendTransform(cube1_transform_stamp)
        rospy.loginfo("cube1 tf has been published!")

    def nav_goal_pub(self,goal_pose):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose = goal_pose
        self.nav_goal_publisher(goal)

    def pub_nav_goal_of_cube1(self):
        cube1_goal_pose = Pose()
        cube1_goal_pose.pose.position.x = cube_positions[0][0]
        cube1_goal_pose.pose.position.y = cube_positions[0][1]
        cube1_goal_pose.pose.position.z = cube_positions[0][2]
        quat = R.from_euler("zyx",[math.pi,0,0]).as_quat()
        cube1_goal_pose.pose.orientation.x = quat[0]
        cube1_goal_pose.pose.orientation.y = quat[1]
        cube1_goal_pose.pose.orientation.z = quat[2]
        cube1_goal_pose.pose.orientation.w = quat[3]
        self.nav_goal_pub(cube1_goal_pose)

    def check_goal_state(self,msg):
        if self.cube1_pose_stamped is None:
            rospy.logwarn("cube1 tf has not been published!!")
            return None
        
        if self.reach_goal_state == 0:
            if len(msg.status_list)==0 or \
                (len(msg.status_list)>0 and len(msg.status_list)==3):
                self.reach_goal_state = 1
        elif self.reach_goal_state == 1:
            rospy.loginfo("reached location:%d" % self.current_goal)
            if self.current_goal == -2:
                self.current_goal=0
                self.last_goal = self.current_goal
            elif self.current_goal != 0 and self.current_goal !=1:
                self.current_goal = -1
            else:
                self.current_goal = (self.last_goal+1)%6
                self.last_goal = self.current_goal
            
            self.goal_location = cube_positions[self.current_goal]
            loc = self.goal_location
            goal_map = copy.copy(self.cube1_pose_stamped)
            goal_map.pose.position.x = loc[0]
            goal_map.pose.position.y = loc[1]
            goal_map.pose.position.z = loc[2]
            goal_map.pose.orientation.x = loc[3]
            goal_map.pose.orientation.y = loc[4]
            goal_map.pose.orientation.z = loc[5]
            goal_map.pose.orientation.w = loc[6]

            self.nav_goal_publisher(goal_map)
            rospy.loginfo("navigation to location: %d" %self.current_goal)
            self.reach_goal_state = 2

        elif self.reach_goal_state==2:
            if len(msg.status_list)>0 and msg.status_list[0].status ==1:
                self.reach_goal_state=0

    def publish_nav_goal(self):
        if self.cube1_pose_stamped is None:
            rospy.logwarn("cube1 pose stamped has not been published!!")
            return None
        goal_map = copy.copy(self.cube1_pose_stamped)
        goal_map.pose.position.x+=0.6
        goal_map.pose.position.z=0.0
        quat = R.from_euler('z',np.pi).as_quat()
        goal_map.pose.orientation.x = quat[0]
        goal_map.pose.orientation.y = quat[1]
        goal_map.pose.orientation.z = quat[2]
        goal_map.pose.orientation.w = quat[3]

        self.nav_goal_pub(goal_map)       

    # def startNavigation(self,msg):
    #     start = self.cur_key_loc
    #     goal = msg.data
    #     route
        
    # def prodecure(self):
    #     place1 = 


    def main(self):
        time.sleep(0.5)
        rate = rospy.Rate(30)
        self.publish_nav_goal()
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('brain_node')
        brain = Brain()
        brain.main()
    except rospy.ROSInterruptException:
        pass
