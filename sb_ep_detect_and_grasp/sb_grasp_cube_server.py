#!/bin/python3

import rospy
import numpy as np
import cv2

import os
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, Pose, Point

import tf
import tf2_ros
import tf2_geometry_msgs

import roslib
import sys

from sb_ep_detect_and_grasp.srv import grasp_place,grasp_placeRequest,grasp_placeResponse

from scipy.spatial.transform import Rotation as R


class graspAruco:
    def __init__(self):
        self.base_move_position_pub = rospy.Publisher("cmd_position", Twist)
        self.base_move_vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose)
        self.image_sub = rospy.Subscriber("/aruco_pose", Pose, self.get_pos_ang_in_base_callback, queue_size = 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pos_2_base = None
        self.ang_2_base = None
        self.ros_rate = 30

        self.server = rospy.Service('grasp_', grasp_place, self.server_callback)

        self.count=0
        
        self.grasp_success = False
        # self.place_success = False
        self.vel_cmd=None
        self.base_vel = 0.11

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        print("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        print("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def move_arm(self):
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.2       # TODO
        move_arm_msg.position.y = -0.02
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def reset_arm(self):            
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        print("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)
        rospy.sleep(0.1)
        self.arm_position_pub.publish(reset_arm_msg)   

    def forward_zero(self):
        zero_cmd = Twist()
        zero_cmd.linear.x = 0.0
        zero_cmd.linear.y = 0.0
        zero_cmd.linear.z = 0.0
        zero_cmd.angular.x = 0.0
        zero_cmd.angular.y = 0.0
        zero_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(zero_cmd)
        rospy.sleep(0.05)

    def move_base_x(self, x_move):
        move_base_msg_x = Twist()
        move_base_msg_x.linear.x = x_move
        move_base_msg_x.linear.y = 0.0
        move_base_msg_x.linear.z = 0.0
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        print("move the base in x direction")
        self.base_move_position_pub.publish(move_base_msg_x)

    def get_pos_ang_in_base_callback(self,ore_2_cam):
       
        ore_2_cam_tf_stamped = tf2_geometry_msgs.PoseStamped()
        ore_2_cam_tf_stamped.header.stamp = rospy.Time.now()
        ore_2_cam_tf_stamped.header.frame_id ="camera_aligned_depth_to_color_frame_correct"
        ore_2_cam_tf_stamped.pose = ore_2_cam
        ore_2_base_stamped = self.tf_buffer.transform(ore_2_cam_tf_stamped, "base_link")

        ore_2_base = ore_2_base_stamped.pose
        pos_2_base = np.array([ore_2_base.position.x,ore_2_base.position.y,ore_2_base.position.z])
        quat_2_base = np.array([ore_2_cam.orientation.x,ore_2_cam.orientation.y,ore_2_cam.orientation.z,ore_2_cam.orientation.w])
        angle_2_base = R.from_quat(quat_2_base).as_euler("YXZ")[0]
        
        def ang_ref(angle):
            if angle>np.pi/2:
                angle-=np.pi
            elif angle<-np.pi/2:
                angle+=np.pi
            return angle
        angle_2_base = ang_ref(angle_2_base)

        self.pos_2_base = pos_2_base
        self.angle_2_base = angle_2_base
        return self.pos_2_base,self.angle_2_base


    def e_refine(self,e,thre):
        return 0 if np.abs(e)<thre else e
        

    def controller(self, pos_2_base, angle_2_base,desired_pos,desired_ang=0):
        vel_cmd = np.zeros((3,))
        if pos_2_base.all()==None:
            return None

        if self.count%1000==0:
            print("pos_2_base",pos_2_base)
            print("angle_2_base",angle_2_base)

        distance_in_x = pos_2_base[0]-desired_pos[0]
        distance_in_y = pos_2_base[1]-desired_pos[1]
        distance_in_ang = angle_2_base-desired_ang

        vel_cmd[0] = 4*self.e_refine(distance_in_x,0.01)
        vel_cmd[1] = 4*self.e_refine(distance_in_y,0.01)
        vel_cmd[2] = -4*self.e_refine(distance_in_ang,10*np.pi/180)

        vel_cmd = np.clip(np.abs(vel_cmd),[0.11,0.11,0.01],[0.5,0.5,0.5])*np.sign(vel_cmd)

        self.vel_cmd = vel_cmd

        return vel_cmd


    def grasp_ore(self,target_id):
        rate = rospy.Rate(self.ros_rate)
        target_pos = [0.27,0,0]
        target_ang=0
        gama_x = 0.01
        gama_y = 0.01
        gama_w = 10*np.pi/180
        while not self.grasp_success:
            # print("in the loop")
            self.count+=1
            distance_in_x = self.pos_2_base[0]-target_pos[0]
            distance_in_y = self.pos_2_base[1]-target_pos[1]
            distance_in_ang = self.angle_2_base-target_ang
            if (abs(distance_in_x) <= gama_x) and (abs(distance_in_y) <= gama_y) and \
                (abs(distance_in_ang)<gama_w) and self.grasp_success==False:
                self.forward_zero()
                rospy.sleep(0.1)
                print("===== start to grasp ====")
                self.move_arm()
                rospy.sleep(1)
                self.close_gripper()
                # self.forward_zero()
                rospy.sleep(0.2)
                self.reset_arm()
                # rospy.sleep(1)
                # self.forward_zero()
                print("===== finish graspping ====")

                self.grasp_success = True
                
            else:
                control_input = self.controller(self.pos_2_base,self.angle_2_base,target_pos)
                vel_cmd_pub = Twist()
                vel_cmd_pub.linear.x = control_input[0]
                vel_cmd_pub.linear.y = control_input[1]
                vel_cmd_pub.linear.z = 0.0
                vel_cmd_pub.angular.x = 0.0
                vel_cmd_pub.angular.y = 0.0
                vel_cmd_pub.angular.z = control_input[2]
                self.base_move_vel_pub.publish(vel_cmd_pub)
        # self.move_base_x(-0.1)
        self.grasp_success=False
        # rospy.sleep(10)
        return True



    def server_callback(self,req):
        if req.task=='grasp':
            success = self.grasp_ore(req.id)
        elif req.task=='place':
            pass
        return success
           

    

def main():

    rospy.init_node('grasp_aruco_node', anonymous=True)
    ap = graspAruco()
    print("=====init=====")
    ap.reset_arm()
    rospy.sleep(1)
    print("=====reset arm at beginning=====")
    ap.open_gripper()
    rospy.sleep(1)
    print("=====open gripper at beginning=====")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        ap.forward_zero()
        # ap.reset_arm()


if __name__=="__main__":
    main()
    