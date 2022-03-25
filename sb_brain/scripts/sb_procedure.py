#!/bin/python3

import rospy
import numpy as np
import cv2
import math
import os
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
import tf
import tf2_ros
import tf2_geometry_msgs
import time
import roslib
import sys

from sb_ep_detect_and_grasp.srv import grasp_place

from scipy.spatial.transform import Rotation as R

# cube1~5 and target1~3 in world frame
cube_positions = [[1.00058174, 0.09497403, 3.39986682, -0.00076221, -0.0013086464023217559, -0.0012807715684175491,
                   0.9999980330467224],
                  [1.90030038356781, 0.08630374819040298, 3.099881172180176, -0.0009147212258540094,
                   -0.0007257394026964903, -0.004962262697517872, 0.999987006187439],
                  [1.5008326768875122, 0.08552967756986618, 0.998009443283081, -0.0018889335915446281,
                   0.02178667113184929, 0.0013308123452588916, 0.9997599720954895],
                  [3.999114990234375, 0.09183745086193085, 1.8005446195602417, 0.0003158680337946862,
                   0.004514407832175493, 0.0019894992001354694, 0.999987781047821],
                  [5.099486827850342, 0.0978584736585617, 0.3004781901836395, 0.0014465743442997336,
                   0.0008184657199308276, 0.0041693891398608685, 0.999989926815033],
                  [2.3251845836639404, 0.08210877216421068, 1.8196847438812256, 0, 0, 0, -1],
                  [2.4501845836639404, 0.08210877216421068, 1.8196847438812256, 0, 0, 0, -1],
                  [2.5751845836639404, 0.08210877216421068, 1.8196847438812256, 0, 0, 0, -1]]



EXCHANGE_POSE = [1.6803152561187744, 1.7498154163360597, 0.08210877216420992, 0, 0, 0, 1]
EXCHANGE_SEE_RELA_GOALS = [-1.5, -0.3, 0]
EXCHANGE_PLACE_RELA_GOALS = [-0.5, 0, 0]
EXCHANGE_OFFSET = 0.12  # distance between adjacent letter

GOAL_POS_THRE = 0.1
GOAL_ANGLE_THRE = 0.1
CMD_VEL_THRE = 0.05


class Brain(object):
    def __init__(self):
        rospy.loginfo("initing!!!!!!!!")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.cube1_pose = None
        self.finished = False

        self.current_pos = [0.0, 0.0]
        self.state = 'initial'
        self.rate = rospy.Rate(30)
        # publish the world coordinate
        self.publish_world_tf()

        # calculate all 8 key positions in map pose
        self.key_position_in_map_pose = []
        for i in range(len(cube_positions)):
            self.key_position_in_map_pose.append(self.calculate_pose_in_map_frame(cube_positions[i]))

        self.goal_reached_check = False
        self.current_goal = -1
        self.reach_goal_state = -1
        self.cmd_vel = None

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber('/ep/odom', Odometry, self.keyLocCheck)
        # rospy.Subscriber

        rospy.wait_for_service('place_')
        self.place_cli = rospy.ServiceProxy('place_', grasp_place)

        self.nav_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped)

    def publish_world_tf(self):
        rospy.loginfo("world tf is publishing!")
        trans = [4.2, 0, 3.5]
        rot_matrix = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
        trans_map2world = np.zeros((4, 4))
        trans_map2world[0:3, 0:3] = rot_matrix
        trans_map2world[0:3, 3] = np.array(trans)
        trans_map2world[3, 3] = 1

        trans_world2map = np.linalg.inv(trans_map2world)
        quat = R.from_matrix(trans_world2map[:3, :3]).as_quat()
        world_transform_stamp = tf2_ros.TransformStamped()
        world_transform_stamp.header.stamp = rospy.Time.now()
        world_transform_stamp.header.frame_id = "map"
        world_transform_stamp.child_frame_id = "world"
        world_transform_stamp.transform.translation.x = trans_world2map[0, 3]
        world_transform_stamp.transform.translation.y = trans_world2map[1, 3]
        world_transform_stamp.transform.translation.z = trans_world2map[2, 3]
        world_transform_stamp.transform.rotation.x = quat[0]
        world_transform_stamp.transform.rotation.y = quat[1]
        world_transform_stamp.transform.rotation.z = quat[2]
        world_transform_stamp.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(world_transform_stamp)
        rospy.loginfo("world tf has been published!")

    def calculate_pose_in_map_frame(self, pose_in_world_list):
        if not self.tf_buffer.can_transform("map", "world", time=rospy.Time(), timeout=rospy.Duration(1 / 10)):
            rospy.logwarn("can not find world tf!")
            return []
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position.x = pose_in_world_list[0]
        pose_stamped.pose.position.y = pose_in_world_list[1]
        pose_stamped.pose.position.z = pose_in_world_list[2]
        pose_stamped.pose.orientation.x = pose_in_world_list[3]
        pose_stamped.pose.orientation.y = pose_in_world_list[4]
        pose_stamped.pose.orientation.z = pose_in_world_list[5]
        pose_stamped.pose.orientation.w = pose_in_world_list[6]
        pose_stamped = tf2_geometry_msgs.PoseStamped(pose_stamped.header, pose_stamped.pose)
        pose_stamped_in_map = self.tf_buffer.transform(pose_stamped, "map")
        print('pose_stamped_in_map.pose', pose_stamped_in_map.pose)
        return pose_stamped_in_map.pose

    def nav_goal_pub(self, goal_pose):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose = goal_pose
        self.nav_goal_publisher.publish(goal)
        rospy.loginfo("i have published cube goal---------------")
        # rospy.loginfo(goal_map)
        self.current_goal = goal.pose

    def publish_nav_goal(self, cube_idx, box_idx, place_or_get):

        if place_or_get == 'place':
            # load BOX position then publish, to be finished
            goal_map = copy.copy(self.key_position_in_map_pose[box_idx+4])
            # BOX bias and orientation are same
            if box_idx == 1:
                goal_map.position.x -= 0.5
                goal_map.position.y -= 0.1
                goal_map.position.z = 0.0
                quat = R.from_euler('z', 0).as_quat()
                goal_map.orientation.x = quat[0]
                goal_map.orientation.y = quat[1]
                goal_map.orientation.z = quat[2]
                goal_map.orientation.w = quat[3]
            if box_idx == 2:
                goal_map.position.x -= 0.5
                goal_map.position.y -= 0.0
                goal_map.position.z = 0.0
                quat = R.from_euler('z', 0).as_quat()
                goal_map.orientation.x = quat[0]
                goal_map.orientation.y = quat[1]
                goal_map.orientation.z = quat[2]
                goal_map.orientation.w = quat[3]
            if box_idx == 3:
                goal_map.position.x -= 0.6
                goal_map.position.y -= 0.0
                goal_map.position.z = 0.0
                quat = R.from_euler('z', 0).as_quat()
                goal_map.orientation.x = quat[0]
                goal_map.orientation.y = quat[1]
                goal_map.orientation.z = quat[2]
                goal_map.orientation.w = quat[3]

        if place_or_get == 'get':
            goal_map = copy.copy(self.key_position_in_map_pose[cube_idx - 1])
            # set bias and orientation separately
            if cube_idx == 1:
                goal_map.position.x += 0.6
                goal_map.position.z = 0.0
                quat = R.from_euler('z', np.pi).as_quat()
                goal_map.orientation.x = quat[0]
                goal_map.orientation.y = quat[1]
                goal_map.orientation.z = quat[2]
                goal_map.orientation.w = quat[3]

            if cube_idx == 2:
                goal_map.position.x += 0.0
                goal_map.position.y += 0.6
                goal_map.position.z = 0.0
                quat = R.from_euler('z', -0.5*np.pi).as_quat()
                goal_map.orientation.x = quat[0]
                goal_map.orientation.y = quat[1]
                goal_map.orientation.z = quat[2]
                goal_map.orientation.w = quat[3]

            if cube_idx == 3:
                goal_map.position.x -= 0.6
                goal_map.position.z = 0.0
                quat = R.from_euler('z', 0*np.pi).as_quat()
                goal_map.orientation.x = quat[0]
                goal_map.orientation.y = quat[1]
                goal_map.orientation.z = quat[2]
                goal_map.orientation.w = quat[3]

        self.nav_goal_pub(goal_map)

    def keyLocCheck(self, msg):
        curr = msg.pose.pose
        current_pos = np.array([curr.position.x, curr.position.y])
        current_quat = np.array([curr.orientation.x, curr.orientation.y, curr.orientation.z, curr.orientation.w])

        goal = self.current_goal
        goal_pos = np.array([goal.position.x, goal.position.y])
        goal_quat = np.array([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])

        position_error = np.linalg.norm(current_pos - goal_pos)
        angle_error = np.abs((R.from_quat(goal_quat) * R.from_quat(current_quat).inv()).as_euler('ZYX')[0])

        # debug
        rospy.logdebug("xy: " + str(position_error) + "    , angle:" + str(angle_error))
        if (position_error < GOAL_POS_THRE and angle_error < GOAL_ANGLE_THRE) and np.linalg.norm(
                self.cmd_vel) < CMD_VEL_THRE:
            rospy.loginfo("Navigation: reach the navigation goal.")
            self.reach_goal_state = True
            self.current_goal = None

    def cmd_vel_callback(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def grasp(self):
        rospy.wait_for_service('grasp_')
        try:
            self.grasp_cli = rospy.ServiceProxy('grasp_', grasp_place)
            resp1 = cli('grasp', 0)
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # def cube1_pose_callback(self,msg):
    #     if not self.tf_buffer.can_transform("world","camera_aligned_depth_to_color_frame_correct",rospy.Time.now()):
    #         rospy.logwarn("can not find world tf!!cube1 callback")
    #         return None

    #     pose_stamped = PoseStamped()
    #     pose_stamped.header.stamp = rospy.Time.now()
    #     pose_stamped.header.frame_id = "world"
    #     pose_stamped.pose = msg
    #     pose_stamped = tf2_geometry_msgs.PoseStamped(pose_stamped.header,pose_stamped.pose)
    #     pose_stamped_in_map = self.tf_buffer.transform(pose_stamped,"map")
    #     self.cube1_pose = pose_stamped_in_map.pose

    #     # publish tf for cube1
    #     cube1_transform_stamp = tf2_ros.TransformStamped()
    #     cube1_transform_stamp.header.stamp = rospy.Time.now()
    #     cube1_transform_stamp.header.frame_id = "map"
    #     cube1_transform_stamp.child_frame_id = "cube1"
    #     cube1_transform_stamp.transform.translation.x = pose_stamped_in_map.pose.position.x
    #     cube1_transform_stamp.transform.translation.y = pose_stamped_in_map.pose.position.y
    #     cube1_transform_stamp.transform.translation.z = pose_stamped_in_map.pose.position.z
    #     cube1_transform_stamp.transform.rotation.x = pose_stamped_in_map.pose.orientation.x
    #     cube1_transform_stamp.transform.rotation.y = pose_stamped_in_map.pose.orientation.y
    #     cube1_transform_stamp.transform.rotation.z = pose_stamped_in_map.pose.orientation.z
    #     cube1_transform_stamp.transform.rotation.w = pose_stamped_in_map.pose.orientation.w

    #     self.tf_broadcaster.sendTransform(cube1_transform_stamp)
    #     rospy.loginfo("cube1 tf has been published!")

    # def pub_nav_goal_of_cube1(self):
    #     cube1_goal_pose = Pose()
    #     cube1_goal_pose.pose.position.x = cube_positions[0][0]+0.5
    #     cube1_goal_pose.pose.position.y = cube_positions[0][1]
    #     cube1_goal_pose.pose.position.z = cube_positions[0][2]
    #     quat = R.from_euler("zyx",[math.pi,0,0]).as_quat()
    #     cube1_goal_pose.pose.orientation.x = quat[0]
    #     cube1_goal_pose.pose.orientation.y = quat[1]
    #     cube1_goal_pose.pose.orientation.z = quat[2]
    #     cube1_goal_pose.pose.orientation.w = quat[3]
    #     self.nav_goal_pub(cube1_goal_pose)

    # def check_goal_state(self,msg):
    #     if self.cube1_pose is None:
    #         rospy.logwarn("cube1 tf has not been published!!")
    #         return None

    #     if self.reach_goal_state == 0:
    #         if len(msg.status_list)==0 or \
    #             (len(msg.status_list)>0 and len(msg.status_list)==3):
    #             self.reach_goal_state = 1
    #     elif self.reach_goal_state == 1:
    #         rospy.loginfo("reached location:%d" % self.current_goal)
    #         if self.current_goal == -2:
    #             self.current_goal=0
    #             self.last_goal = self.current_goal
    #         elif self.current_goal != 0 and self.current_goal !=1:
    #             self.current_goal = -1
    #         else:
    #             self.current_goal = (self.last_goal+1)%6
    #             self.last_goal = self.current_goal

    #         self.goal_location = cube_positions[self.current_goal]
    #         loc = self.goal_location
    #         goal_map = copy.copy(self.cube1_pose)
    #         goal_map.pose.position.x = loc[0]
    #         goal_map.pose.position.y = loc[1]
    #         goal_map.pose.position.z = loc[2]
    #         goal_map.pose.orientation.x = loc[3]
    #         goal_map.pose.orientation.y = loc[4]
    #         goal_map.pose.orientation.z = loc[5]
    #         goal_map.pose.orientation.w = loc[6]

    #         self.nav_goal_publisher(goal_map)
    #         rospy.loginfo("navigation to location: %d" %self.current_goal)
    #         self.reach_goal_state = 2

    #     elif self.reach_goal_state==2:
    #         if len(msg.status_list)>0 and msg.status_list[0].status ==1:
    #             self.reach_goal_state=0


def main():
    time.sleep(1)
    rospy.init_node('brain')
    brain = Brain()
    time.sleep(0.5)

    rate = rospy.Rate(10)
    exc_idx = 1
    gotten_cube = False
    while not rospy.is_shutdown():
        # rospy.loginfo(brain.state)
        if brain.state == 'initial':
            brain.publish_nav_goal(1,0,'get')
            brain.state = 'navigation'


        if brain.state == 'navigation':
            if brain.reach_goal_state == True and brain.current_goal == None:
                # detect the marker should execute here
                if gotten_cube == True:
                    brain.state = 'place_cube'
                else:
                    brain.state = 'grasp_cube'

                brain.reach_goal_state == False # reset flag and navigation is finished

        if brain.state == 'grasp_cube':
            # ret = brain.grasp()
            ret = True
            if ret == True:
                gotten_cube = True

                brain.publish_nav_goal(0,exc_idx,'place') # go to place the cube
                brain.state = "navigation"

        if brain.state == 'place_cube':
            # place_success = brain.place()
            place_success = True
            if place_success == True:
                gotten_cube = False

                if exc_idx == 3:
                    brain.state = 'OFF'
                else:
                    exc_idx += 1 #excuting next cube
                    # if detection is needed to set cube_idx in publish_nav_goal()
                    # brain.detectlist2goal()
                    # here exc_idx == cube_idx i.e. getting cube 123 sequentially
                    brain.publish_nav_goal(exc_idx, 0, 'get')
                    brain.state = 'navigation'

        rate.sleep()


if __name__ == "__main__":
    main()