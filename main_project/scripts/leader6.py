#!/usr/bin/env python

import rospy
import actionlib
import tf
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class LeaderMBF:
    def __init__(self):
        rospy.init_node('leader_move_base_flex_node', anonymous=True)

        # Action Client per Move Base Flex
        self.client = actionlib.SimpleActionClient('/mir_leader/move_base_flex/move_base', MoveBaseAction)
        rospy.loginfo("Aspettando il server di Move Base Flex...")
        self.client.wait_for_server()
        rospy.loginfo("Server MBF trovato!")

        # Publisher della posizione del leader
        self.leader_pose_pub = rospy.Publisher('/mir_leader/mir_pose_simple', PoseStamped, queue_size=10)

        # Sottoscrizione all'odometria del leader
        self.odom_sub = rospy.Subscriber('/mir_leader/odom', Odometry, self.odom_callback)
        
        self.leader_pose = PoseStamped()

    def odom_callback(self, msg):
        """ Callback per ottenere la posizione attuale del leader e pubblicarla """
        self.leader_pose.header.stamp = rospy.Time.now()
        self.leader_pose.header.frame_id = "map"
        self.leader_pose.pose.position.x = msg.pose.pose.position.x
        self.leader_pose.pose.position.y = msg.pose.pose.position.y
        self.leader_pose.pose.orientation = msg.pose.pose.orientation

        # Pubblica la posizione del leader
        self.leader_pose_pub.publish(self.leader_pose)

    def send_goal(self, x, y):
        """ Invia un goal a Move Base Flex """
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()

        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Nessuna rotazione

        rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        result = self.client.get_result()
        rospy.loginfo(f"Goal completato, risultato: {result}")

if __name__ == '__main__':
    leader = LeaderMBF()
    leader.send_goal(40.0, 40.0)  # Cambia le coordinate se necessario
    rospy.spin()  # Mantieni il nodo attivo per la pubblicazione della posizione

