#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, PointStamped, Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
import tf 
import tf2_ros
import tf2_geometry_msgs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal, MoveBaseFeedback
from vision_marker_msg.msg import MarkerPose
import actionlib
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal, PickAndPlaceFeedback, PickAndPlaceResult

import time
import numpy as np 
from scipy.spatial.transform import Rotation as sciR
import copy


# hyperparameters
GOAL_POS_TOLERANCE = 0.1
GOAL_ANGLE_TOLERANCE = 0.1
CMD_VEL_TOLERANCE = 0.11

SEE_MARKERS_HEIGHT_THRES = 0.2

# poses (position + quaternion) in /map frame: 5 cubes & exchange station O
CUBE_POSES = [[0.10013318061828613, 3.1994182586669924, 0.09497402608394549, -0.4996039721005702, 0.49963184693447443, 0.501674832514571, -0.49908541454320016],
              [0.4001188278198242, 2.29969961643219, 0.08630374819040239, -0.5016544041300345, 0.4974178808351977, 0.5032948647549162, -0.4976068626583561],
              [2.501990556716919, 2.699167323112488, 0.08552967756986503, 0.5091634486392052, -0.48870758985285856, -0.4892657110991376, 0.5123831945881719],
              [1.6994553804397583, 0.20088500976562518, 0.09183745086193043, 0.5014142788574969, -0.4988893702254536, -0.49658400299152033, 0.5030879100224974],
              [3.1995218098163605, -0.8994868278503416, 0.09785847365856121, 0.4990427888699, -0.5023937122898315, -0.4967777488056687, 0.5017656036650626]]

EXCHANGE_POSE = [1.6803152561187744, 1.7498154163360597, 0.08210877216420992, 0, 0, 0, 1]


# navigation_relative_goal (relative x, relative y (to the object), absolute angle)
CUBE_NAV_RELA_GOALS = [[0.5, 0, np.pi],
                       [-0.4, 0, 0],
                       [-0.5, 0, 0],
                       [0.5, 0, np.pi],
                       [-0.5, 0, 0]]

EXCHANGE_SEE_RELA_GOALS = [-1.5, -0.3, 0]
EXCHANGE_PLACE_RELA_GOALS = [-0.5, 0, 0]
EXCHANGE_OFFSET = 0.12 # distance between adjacent letter



# ---------------------------------------------------------------------------------------------------------
class System(object):
    # ----------------------------------------------------------------
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.cubes_pose_stamped = [None, None, None, None, None]
        self.exchange_posestamped = None
        self.current_base_posestamped = None
        self.current_goal_posestamped = None
        self.cmd_vel = None
        self.current_marker_poses = None
        self.reached_goal = True
        

        # assuming the poses of the cubes are fixed in the current task
        self.cube_poses = CUBE_POSES
        self.exchange_pose = EXCHANGE_POSE
        self.setCubesPoseStampeds()
        self.setExchangePoseStampeds()

        rospy.Subscriber("/cmd_vel", Twist, self.cmdVelCb)
        rospy.Subscriber("/aruco_pose", MarkerPose, self.markerPoseCb)
        # self.base_move_pos_pub = rospy.Publisher("cmd_position", Twist, queue_size=1)

        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

        self.pick_and_place_client = actionlib.SimpleActionClient('pick_and_place', PickAndPlaceAction)
        self.pick_and_place_client.wait_for_server()

        # self.nav_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)


    # ----------------------------------------------------------------
    def setCubesPoseStampeds(self):
        for i in range(5):
            posestamped = PoseStamped()
            posestamped.header.stamp = rospy.Time.now()
            posestamped.header.frame_id = "map"
            posestamped.pose.position.x = self.cube_poses[i][0]
            posestamped.pose.position.y = self.cube_poses[i][1]
            posestamped.pose.position.z = self.cube_poses[i][2]
            posestamped.pose.orientation.x = self.cube_poses[i][3]
            posestamped.pose.orientation.y = self.cube_poses[i][4]
            posestamped.pose.orientation.z = self.cube_poses[i][5]
            posestamped.pose.orientation.w = self.cube_poses[i][6]
            self.cubes_pose_stamped[i] = posestamped

    
    # ----------------------------------------------------------------
    def setExchangePoseStampeds(self):
        posestamped = PoseStamped()
        posestamped.header.stamp = rospy.Time.now()
        posestamped.header.frame_id = "map"
        posestamped.pose.position.x = self.exchange_pose[0]
        posestamped.pose.position.y = self.exchange_pose[1]
        posestamped.pose.position.z = self.exchange_pose[2]
        posestamped.pose.orientation.x = self.exchange_pose[3]
        posestamped.pose.orientation.y = self.exchange_pose[4]
        posestamped.pose.orientation.z = self.exchange_pose[5]
        posestamped.pose.orientation.w = self.exchange_pose[6]
        self.exchange_posestamped = posestamped


    # ----------------------------------------------------------------
    def cmdVelCb(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    
    # ----------------------------------------------------------------
    def markerPoseCb(self, msg):
        self.current_marker_poses = msg


    # ----------------------------------------------------------------
    def publishCubesAndExchangeStationTF(self):
        # publish TF for cube
        for cube_id in range(1, 6):
            pose_stamped_inmap = self.cubes_pose_stamped[cube_id - 1]

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "map"
            transform_stamped.child_frame_id = "cube_" + str(cube_id)
            transform_stamped.transform.translation.x = pose_stamped_inmap.pose.position.x
            transform_stamped.transform.translation.y = pose_stamped_inmap.pose.position.y
            transform_stamped.transform.translation.z = pose_stamped_inmap.pose.position.z 
            transform_stamped.transform.rotation.x = pose_stamped_inmap.pose.orientation.x
            transform_stamped.transform.rotation.y = pose_stamped_inmap.pose.orientation.y
            transform_stamped.transform.rotation.z = pose_stamped_inmap.pose.orientation.z
            transform_stamped.transform.rotation.w = pose_stamped_inmap.pose.orientation.w

            self.tf_broadcaster.sendTransform(transform_stamped)

        # publish TF for exchange_station
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "exchange_station"
        transform_stamped.transform.translation.x = self.exchange_posestamped.pose.position.x
        transform_stamped.transform.translation.y = self.exchange_posestamped.pose.position.y
        transform_stamped.transform.translation.z = self.exchange_posestamped.pose.position.z
        transform_stamped.transform.rotation.x = self.exchange_posestamped.pose.orientation.x
        transform_stamped.transform.rotation.y = self.exchange_posestamped.pose.orientation.y
        transform_stamped.transform.rotation.z = self.exchange_posestamped.pose.orientation.z
        transform_stamped.transform.rotation.w = self.exchange_posestamped.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform_stamped)
        

    # ----------------------------------------------------------------
    def sendNavActionGoal(self, goal_map): # goal: PoseStamped() in map frame
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal_map

        self.nav_client.send_goal(move_base_goal, feedback_cb=self.navFeedbackCb)
        self.current_goal_posestamped = goal_map
        self.reached_goal = False

    
    # ----------------------------------------------------------------
    def navFeedbackCb(self, msg):
        curr = msg.base_position.pose
        current_pos = np.array([curr.position.x, curr.position.y])
        current_quat = np.array([curr.orientation.x, curr.orientation.y, curr.orientation.z, curr.orientation.w])
        # current_angle = sciR.from_quat(current_quat).as_euler('ZYX')[0]
        goal = self.current_goal_posestamped.pose
        goal_pos = np.array([goal.position.x, goal.position.y])
        goal_quat = np.array([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])
        # goal_angle = sciR.from_quat(goal_quat).as_euler('ZYX')[0]

        position_error = np.linalg.norm(current_pos - goal_pos)
        angle_error = np.abs((sciR.from_quat(goal_quat) * sciR.from_quat(current_quat).inv()).as_euler('ZYX')[0])

        # debug
        rospy.logdebug("xy error: " + str(position_error) + ", angle error: " + str(angle_error) )

        if (position_error < GOAL_POS_TOLERANCE and angle_error < GOAL_ANGLE_TOLERANCE) \
                and np.linalg.norm(self.cmd_vel) < CMD_VEL_TOLERANCE:
            self.nav_client.cancel_goal()
            rospy.loginfo("Navigation: reach the navigation goal.")
            self.reached_goal = True


    # ----------------------------------------------------------------
    def getGoalForCube(self, cube_id):
        while self.cubes_pose_stamped[cube_id] is None:
            pass
        goal = copy.deepcopy(self.cubes_pose_stamped[cube_id])
        goal.pose.position.x += CUBE_NAV_RELA_GOALS[cube_id][0]
        goal.pose.position.y += CUBE_NAV_RELA_GOALS[cube_id][1]
        goal.pose.position.z = 0.0
        quat = sciR.from_euler('z', CUBE_NAV_RELA_GOALS[cube_id][2]).as_quat()
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        return goal

    
    # ----------------------------------------------------------------
    def getGoalForExchangeStation(self, m_or_p, target_id=None):
        while self.exchange_posestamped is None:
            pass
        goal = copy.deepcopy(self.exchange_posestamped)
        goal.pose.position.z = 0.0
        if m_or_p == 'see_marker':
            goal.pose.position.x += EXCHANGE_SEE_RELA_GOALS[0]
            goal.pose.position.y += EXCHANGE_SEE_RELA_GOALS[1]
            quat = sciR.from_euler('z', EXCHANGE_SEE_RELA_GOALS[2]).as_quat()
        elif m_or_p == 'place_cube':
            goal.pose.position.x += EXCHANGE_PLACE_RELA_GOALS[0]
            goal.pose.position.y += EXCHANGE_PLACE_RELA_GOALS[1]
            # different target letter 'BOX'
            if target_id == 5: 
                goal.pose.position.y += EXCHANGE_OFFSET
            elif target_id == 7:
                goal.pose.position.y += -EXCHANGE_OFFSET
            quat = sciR.from_euler('z', EXCHANGE_PLACE_RELA_GOALS[2]).as_quat()
        else:
            rospy.logwarn("Invalid m_or_p, should be either 'see_marker' or 'place_cube'.")
        
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        return goal


    # ----------------------------------------------------------------
    # read the cube ids in the current image. used for getting the target cube ids when 'see_markers'.
    def getTargetCubeIDs(self):
        target_cube_ids = []
        str_ids = ""
        marker_poses = copy.deepcopy(self.current_marker_poses)
        ids = marker_poses.marker_ids
        poses = marker_poses.marker_poses
        for i, id in enumerate(ids):
            if id <= 4 and poses[i].position.y < -SEE_MARKERS_HEIGHT_THRES:
                target_cube_ids.append(id)
                str_ids += (str(id + 1) + " ")

        rospy.loginfo("Get the target cube IDs (1~5): " + str_ids)
        return target_cube_ids


    # ----------------------------------------------------------------
    def pickCube(self, target_id):
        goal = PickAndPlaceGoal(task='pick', target_id=target_id)
        self.pick_and_place_client.send_goal(goal)
        self.pick_and_place_client.wait_for_result()

        return (self.pick_and_place_client.get_result()).success

    
    # ----------------------------------------------------------------
    def placeCube(self, target_id):
        goal = PickAndPlaceGoal(task='place', target_id=target_id)
        self.pick_and_place_client.send_goal(goal)
        self.pick_and_place_client.wait_for_result()

        return (self.pick_and_place_client.get_result()).success

    
    # ----------------------------------------------------------------
    # avoid setting cube 1 as the first target
    def resortTarget(self, target_cube, target_box):
        def swap(a, b):
            return b, a

        if target_cube[0] == 0:
            target_cube[0], target_cube[1] = swap(target_cube[0], target_cube[1])
            target_box[0], target_box[1] = swap(target_box[0], target_box[1])
        
        return target_cube, target_box


    # ----------------------------------------------------------------
    def main(self):
        rate = rospy.Rate(10)
        
        state = 'initial'
        # target_cube_list = [1]
        target_cube_list = []
        target_box_list = [5, 6, 7]
        current_target_cube_idx = 0

        while not rospy.is_shutdown():

            self.publishCubesAndExchangeStationTF()

            if state == 'initial':
                self.sendNavActionGoal(self.getGoalForExchangeStation('see_marker'))
                state = 'move_exchange_see'

            if state == 'move_exchange_see':
                if self.reached_goal == True:
                    state = 'reach_exchange_see'
                    rospy.loginfo("Reach exchange state to see the markers.")
                
            if state == 'reach_exchange_see':
                target_cube_list = self.getTargetCubeIDs() # see the target ids
                target_cube_list, target_box_list = self.resortTarget(target_cube_list, target_box_list) # avoid setting cube 1 as the first target
                self.sendNavActionGoal(self.getGoalForCube(target_cube_list[current_target_cube_idx]))
                state = 'move_cube'

            if state == 'move_cube':
                if self.reached_goal == True:
                    state = 'reach_cube'
                    rospy.loginfo("Reach cube " + str(target_cube_list[current_target_cube_idx]))

            if state == 'reach_cube':
                success = self.pickCube(target_cube_list[current_target_cube_idx])
                if success:
                    rospy.loginfo("Picked the cube.")
                    state = 'picked_cube'
                else: # fail to pick the cube, move back to navigation goal to restart the pick process
                    self.sendNavActionGoal(self.getGoalForCube(target_cube_list[current_target_cube_idx]))
                    state = 'move_cube'

            if state == 'picked_cube':
                self.sendNavActionGoal(self.getGoalForExchangeStation('place_cube', target_id=target_box_list[current_target_cube_idx]))
                state = 'move_exchange_place'

            if state == 'move_exchange_place':
                if self.reached_goal == True:
                    state = 'reach_exchange_place'
                    rospy.loginfo("Reach exchange state to place the cube.")

            if state == 'reach_exchange_place':
                success = self.placeCube(target_box_list[current_target_cube_idx])
                if success:
                    rospy.loginfo("Placed the cube into the exchange station.")
                    state = 'placed_cube'
                else: # fail to place the cube, move back to navigation goal to restart the placing process
                    self.sendNavActionGoal(self.getGoalForExchangeStation('place_cube', target_id=target_box_list[current_target_cube_idx]))
                    state = 'move_exchange_place'

            if state == 'placed_cube':
                current_target_cube_idx += 1
                if current_target_cube_idx < len(target_cube_list):
                    self.sendNavActionGoal(self.getGoalForCube(target_cube_list[current_target_cube_idx]))
                    state = 'move_cube'
                else:
                    state = 'finished'
                    rospy.loginfo("Finish task.")


            rate.sleep()





# -----------------------------------------------------------------------------------
if __name__ == '__main__':

    time.sleep(3)

    try:
        rospy.init_node("system_node", log_level=rospy.INFO)
        time.sleep(1.0) # wait for the node initializing

        system = System()
        system.main()

    except rospy.ROSInterruptException:
        pass
    

    


