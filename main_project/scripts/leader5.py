#!/usr/bin/env python

import rospy
import actionlib
import time
import tf
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from math import sqrt, radians
from tf.transformations import quaternion_from_euler


class LeaderMBF:
    def __init__(self):
        rospy.init_node('leader_move_base_flex_node', anonymous=True)
        
        self.client = actionlib.SimpleActionClient('/mir_leader/move_base_flex/move_base', MoveBaseAction)
        rospy.loginfo("Aspettando il server di Move Base Flex...")
        self.client.wait_for_server()
        rospy.loginfo("Server MBF trovato!")

        self.current_x = None
        self.current_y = None
        self.current_yaw = None  # Nuovo valore per l'orientamento
        
        rospy.Subscriber("/mir_leader/mir_pose_simple", PoseStamped, self.pose_callback)
    
    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        # Converti la quaternione in yaw (orientamento in radianti)
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)

        rospy.loginfo(f"Aggiornata posizione: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw:.2f} rad")

    def is_goal_reached(self, x, y, yaw, pos_tolerance=1, yaw_tolerance=radians(10)):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        # Distanza dal goal
        distance = sqrt((self.current_x - x) ** 2 + (self.current_y - y) ** 2)

        # Differenza di orientamento (normalizzata tra -pi e pi)
        yaw_diff = abs(self.current_yaw - yaw)
        yaw_diff = min(yaw_diff, abs(yaw_diff - 2 * 3.14159))  # Gestisce il wrap-around di pi

        rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {yaw_diff:.2f} rad (tolleranza: {yaw_tolerance:.2f})")

        return distance <= pos_tolerance and yaw_diff <= yaw_tolerance

    def send_goal(self, x, y, yaw_degrees=0, pos_tolerance=1, yaw_tolerance=radians(10)):
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

        rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}, yaw={yaw_degrees}° (toll. pos: {pos_tolerance}, toll. yaw: {yaw_tolerance})")
        self.client.send_goal(goal)

        rate = rospy.Rate(10)
        start_time = time.time()
        timeout = 60  # Tempo massimo in secondi per il raggiungimento del goal

        while not rospy.is_shutdown():
            state = self.client.get_state()
            rospy.loginfo(f"Stato attuale di MBF: {state}")

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("MBF segnala che il goal è stato raggiunto!")
                self.client.cancel_goal()
                break

            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("MBF ha fallito il goal, lo annullo.")
                self.client.cancel_goal()
                break

            if self.is_goal_reached(x, y, yaw, pos_tolerance, yaw_tolerance):
                rospy.loginfo("Goal raggiunto entro le tolleranze!")
                self.client.cancel_goal()
                break

            if time.time() - start_time > timeout:
                rospy.logwarn("Timeout raggiunto, annullo il goal!")
                self.client.cancel_goal()
                break

            rate.sleep()

        # **Controllo del risultato**
        result = self.client.get_result()
        if result:
            rospy.loginfo(f"Goal completato, risultato: {result}")
        else:
            rospy.logwarn("Nessun risultato ricevuto da MBF, il goal potrebbe non essere stato completato correttamente.")


if __name__ == '__main__':
    leader = LeaderMBF()
    leader.send_goal(38.0, 38.0, yaw_degrees=0, pos_tolerance=1, yaw_tolerance=radians(10))


# import rospy
# import actionlib
# import time
# from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import PoseStamped
# from actionlib_msgs.msg import GoalStatus
# from math import sqrt

# class LeaderMBF:
#     def __init__(self):
#         rospy.init_node('leader_move_base_flex_node', anonymous=True)
        
#         self.client = actionlib.SimpleActionClient('/mir_leader/move_base_flex/move_base', MoveBaseAction)
#         rospy.loginfo("Aspettando il server di Move Base Flex...")
#         self.client.wait_for_server()
#         rospy.loginfo("Server MBF trovato!")
        
#         self.current_x = None
#         self.current_y = None
#         rospy.Subscriber("/mir_leader/mir_pose_simple", PoseStamped, self.pose_callback)
    
#     def pose_callback(self, msg):
#         self.current_x = msg.pose.position.x
#         self.current_y = msg.pose.position.y
#         rospy.loginfo(f"Aggiornata posizione: x={self.current_x}, y={self.current_y}")

#     def is_goal_reached(self, x, y, tolerance=1):
#         if self.current_x is None or self.current_y is None:
#             return False
        
#         distance = sqrt((self.current_x - x) ** 2 + (self.current_y - y) ** 2)
        
#         rospy.loginfo(f"Distanza attuale dal goal: {distance} (tolleranza: {tolerance})")
        
#         return distance <= tolerance
    
#     def send_goal(self, x, y, tolerance=1):
#         goal = MoveBaseGoal()
#         goal.target_pose = PoseStamped()
#         goal.target_pose.header.stamp = rospy.Time.now()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.pose.position.x = x
#         goal.target_pose.pose.position.y = y
#         goal.target_pose.pose.orientation.w = 1.0

#         rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}, tolleranza={tolerance}")
#         self.client.send_goal(goal)
        
#         rate = rospy.Rate(10)
#         start_time = time.time()
#         timeout = 60  # Tempo massimo in secondi per il raggiungimento del goal

#         while not rospy.is_shutdown():
#             state = self.client.get_state()
#             rospy.loginfo(f"Stato attuale di MBF: {state}")

#             if state == GoalStatus.SUCCEEDED:
#                 rospy.loginfo("MBF segnala che il goal è stato raggiunto!")
#                 self.client.cancel_goal()
#                 break

#             if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
#                 rospy.logwarn("MBF ha fallito il goal, lo annullo.")
#                 self.client.cancel_goal()
#                 break

#             if self.is_goal_reached(x, y, tolerance):
#                 rospy.loginfo("Goal raggiunto entro la tolleranza!")
#                 self.client.cancel_goal()
#                 break

#             if time.time() - start_time > timeout:
#                 rospy.logwarn("Timeout raggiunto, annullo il goal!")
#                 self.client.cancel_goal()
#                 break

#             rate.sleep()

#         # **4. Controllo del risultato**
#         result = self.client.get_result()
#         if result:
#             rospy.loginfo(f"Goal completato, risultato: {result}")
#         else:
#             rospy.logwarn("Nessun risultato ricevuto da MBF, il goal potrebbe non essere stato completato correttamente.")

# if __name__ == '__main__':
#     leader = LeaderMBF()
#     leader.send_goal(25.0, 20.0, tolerance=1)





