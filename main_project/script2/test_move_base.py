#!/usr/bin/env python3

import rospy
import actionlib
import time
import tf
import math
#from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Twist
from actionlib_msgs.msg import GoalStatus
from math import sqrt, radians, pi, degrees
from tf.transformations import quaternion_from_euler

class LeaderMBF:
    def __init__(self):
        rospy.init_node('leader_move_base_flex_node', anonymous=True)
        
        self.client = actionlib.SimpleActionClient('/mir_leader/move_base', MoveBaseAction)
        rospy.loginfo("Aspettando il server di Move Base Flex...")
        self.client.wait_for_server()
        rospy.loginfo("Server MBF trovato!")

        self.current_x = None
        self.current_y = None
        self.current_yaw = None  # Nuovo valore per l'orientamento


    def send_goal(self, x, y, yaw_degrees, pos_tolerance=0.3333, yaw_tolerance=radians(280.33)):
    #def send_goal(self, x, y, yaw_degrees=0, pos_tolerance=0.5):

        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Converte yaw (in gradi) in quaternione
        yaw = radians(yaw_degrees)
        quat = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.client.send_goal(goal)


if __name__ == '__main__':
    leader = LeaderMBF()
    
    # Definizione dei 4 punti per formare un quadrato (puoi modificare a piacere)
    square_path = [
        (49, 47.8, -90),   # Punto 2
        (54.8, 46, 180),      #3
        (52.6, 38, 90),    # 4  
        (46.5, 38.7, 0)   # Punto 1
        #(38.5, 40.5, -180),    # Punto 2
        #(38.5, 47.5, 90),   # Punto 3
        #(45.5, 47.5, 0),   # Punto 4
        #(45.5, 40.5, -90)     # Punto 1 (ritorno punto di partenza)
    ]
    
    leader.client.wait_for_server()
    leader.client.send_goal(leader.send_goal(square_path[0][0], square_path[0][1], square_path[0][2]))

    
    # daje


