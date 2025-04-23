#!/usr/bin/env python

import rospy
import actionlib
import time
import tf
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Twist
from actionlib_msgs.msg import GoalStatus
from math import sqrt, radians, pi
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
        
        self.cmd_vel_pub = rospy.Publisher("/mir_leader/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        
        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.pose_callback)
    
    def pose_callback(self, msg):
        self.current_x = msg.position.x
        self.current_y = msg.position.y

        # Converti la quaternione in yaw (orientamento in radianti)
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)

        rospy.loginfo(f"Aggiornata posizione: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw:.2f} rad")

    def is_goal_reached(self, x, y, yaw, pos_tolerance=0.2, yaw_tolerance=radians(180)):
    #def is_goal_reached(self, x, y, yaw, pos_tolerance=0.2):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        # Distanza dal goal
        distance = sqrt((self.current_x - x) ** 2 + (self.current_y - y) ** 2)

        # Differenza di orientamento (normalizzata tra -pi e pi)
        yaw_diff = abs(self.current_yaw - yaw)
        yaw_diff = min(yaw_diff, 2 * pi - yaw_diff)  # Wrap-around corretto
        
        rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {yaw_diff:.2f} rad (tolleranza: {yaw_tolerance:.2f})")
        #rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {yaw_diff:.2f})")

        return distance <= pos_tolerance and yaw_diff <= yaw_tolerance
        #return distance <= pos_tolerance 

    def send_goal(self, x, y, yaw_degrees=0, pos_tolerance=0.2, yaw_tolerance=radians(180)):
    #def send_goal(self, x, y, yaw_degrees=0, pos_tolerance=0.2):

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
        #rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}, yaw={yaw_degrees}° (toll. pos: {pos_tolerance})")
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
            #if self.is_goal_reached(x, y, yaw, pos_tolerance):
                rospy.loginfo("Goal raggiunto entro le tolleranze!")
                self.client.cancel_goal()  # Cancella il goal
                rospy.loginfo("Arresto del movimento.")
                                
                # Fermiamo il robot
                stop_cmd = Twist()  # Crea un messaggio vuoto di tipo Twist (comando di arresto)
                self.cmd_vel_pub.publish(stop_cmd)  # Pubblica il comando per fermare il robot
                
                # Stampa la posizione e l'orientamento finale
                rospy.loginfo(f"Posizione finale: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw:.2f} rad")

                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal completato, il robot si ferma.")
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    break  # Fermo il ciclo                    
                break  # Fermo il ciclo e arresto il robot

            if time.time() - start_time > timeout:
                rospy.logwarn("Timeout raggiunto, annullo il goal!")
                self.client.cancel_goal()
                break
            
            rate.sleep()

    def follow_path(self, path):
        for i, point in enumerate(path):
            x, y = point[:2]
            yaw = point[2] if len(point) > 2 else 0
            rospy.loginfo(f"\n[GOAL {i+1}/{len(path)}] Raggiungendo punto: x={x}, y={y}, yaw={yaw}°")
            self.send_goal(x, y, yaw_degrees=yaw)
            rospy.sleep(3)  # breve pausa tra i goal (opzionale)



if __name__ == '__main__':
    leader = LeaderMBF()
    
    # Definizione dei 4 punti per formare un quadrato (puoi modificare a piacere)
    square_path = [
        (39.0, 44.0, 0),     # Punto 1
        (39.0, 47.0, 90),    # Punto 2
        (42.0, 47.0, 180),   # Punto 3
        (42.0, 44.0, -90),   # Punto 4
        (39.0, 44.0, 0),     # Punto 1 (ritorno punto di partenza)

    ]

    leader.follow_path(square_path)
