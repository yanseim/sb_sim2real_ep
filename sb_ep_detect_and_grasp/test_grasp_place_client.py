#!/bin/python3

from __future__ import print_function

import sys
import rospy
from sb_ep_detect_and_grasp.srv import grasp_place

def test_grasp_client(x):
    # grasp, uncomment this=============================
    rospy.wait_for_service('grasp_')
    try:
        cli = rospy.ServiceProxy('grasp_', grasp_place)
        resp1 = cli('grasp',0)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # place, uncomment this===============================
    rospy.wait_for_service('place_')
    try:
        cli = rospy.ServiceProxy('place_', grasp_place)
        resp1 = cli('place',1)# the 2nd parameter: 0,1,2 means B,O,X repectively
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    test_grasp_client(0)