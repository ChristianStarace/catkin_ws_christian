#!/usr/bin/env python3

import rospy
import actionlib
import time
import tf
import math
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Twist
from actionlib_msgs.msg import GoalStatus
from math import sqrt, radians, pi, degrees
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
        
        self.cmd_vel_pub = rospy.Publisher("/mir_leader/mobile_base_controller/cmd_vel", Twist, queue_size=100)
        
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

        rospy.loginfo(f"Aggiornata posizione: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.2f}°")
        #rospy.loginfo(f"Aggiornata posizione: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw:.2f} rad")

    def is_goal_reached(self, x, y, yaw, pos_tolerance=0.3333, yaw_tolerance=radians(280.33)):
    #def is_goal_reached(self, x, y, yaw, pos_tolerance=0.5):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        # Distanza dal goal
        distance = sqrt((self.current_x - x) ** 2 + (self.current_y - y) ** 2)

        # Differenza di orientamento (normalizzata tra -pi e pi)
        yaw_diff = abs(self.current_yaw - yaw)
        yaw_diff = min(yaw_diff, 2 * pi - yaw_diff)  # Wrap-around corretto
        
        rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {math.degrees(yaw_diff):.2f}° (tolleranza: {math.degrees(yaw_tolerance):.2f}°)")
        #rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {yaw_diff:.2f} rad (tolleranza: {yaw_tolerance:.2f})")
        #rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {yaw_diff:.2f})")

        return distance <= pos_tolerance and yaw_diff <= yaw_tolerance
        #return distance <= pos_tolerance 

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

        rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}, yaw={yaw_degrees}° (toll. pos: {pos_tolerance}, toll. yaw: {degrees(yaw_tolerance):.2f}°)")
        #rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}, yaw={yaw_degrees}° (toll. pos: {pos_tolerance})")
        self.client.send_goal(goal)

        rate = rospy.Rate(10)
        start_time = time.time()
        timeout = 45  # Tempo massimo in secondi per il raggiungimento del goal

        while not rospy.is_shutdown():
            state = self.client.get_state()
            #rospy.loginfo(f"Stato attuale di MBF: {state}")

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("MBF segnala che il goal è stato raggiunto!")
                #
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                rospy.sleep(0.5)  # Piccola pausa per evitare preemption del goal successivo         
                break  # Fermo il ciclo           
                # 
                # rospy.sleep(0.5)  # Piccola pausa per evitare preemption del goal successivo
                # break

            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("MBF ha fallito il goal, lo annullo.")
                self.client.cancel_goal()
                rospy.sleep(0.5)  # Piccola pausa per evitare preemption del goal successivo
                break

            if self.is_goal_reached(x, y, yaw, pos_tolerance, yaw_tolerance):
            #if self.is_goal_reached(x, y, yaw, pos_tolerance):
                GREEN = "\033[92m"
                RESET = "\033[0m"
                rospy.loginfo(f"Goal: x={x}, y={y}, yaw={degrees(yaw):.2f}° raggiunto entro le tolleranze!")
                self.client.cancel_goal()  # Cancella il goal
                GREEN = "\033[92m"
                RESET = "\033[0m"
                rospy.loginfo("Arresto del movimento.")
                                
                # Fermiamo il robot
                stop_cmd = Twist()  # Crea un messaggio vuoto di tipo Twist (comando di arresto)
                self.cmd_vel_pub.publish(stop_cmd)  # Pubblica il comando per fermare il robot
                
                # Stampa la posizione e l'orientamento finale
                GREEN = "\033[92m"
                RESET = "\033[0m"
                rospy.loginfo(f"{GREEN}Posizione finale: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={degrees(self.current_yaw):.2f}°{RESET}")

                # if self.client.get_state() == GoalStatus.SUCCEEDED:
                #     rospy.loginfo(f"Goal: x={x}, y={y}, yaw={degrees(yaw):.2f}° completato, il robot si ferma.")
                #     stop_cmd = Twist()
                #     self.cmd_vel_pub.publish(stop_cmd)
                #     rospy.sleep(0.5)  # Piccola pausa per evitare preemption del goal successivo         
                #     break  # Fermo il ciclo      
                rospy.sleep(0.5)  # Piccola pausa per evitare preemption del goal successivo              
                break  # Fermo il ciclo e arresto il robot 

            if time.time() - start_time > timeout:
                rospy.logwarn("Timeout raggiunto, annullo il goal!")
                self.client.cancel_goal()
                rospy.sleep(1)  # Piccola pausa per evitare preemption del goal successivo         
                break
            
            rate.sleep()

    def follow_path(self, path):
        for i, point in enumerate(path):
            x, y = point[:2]
            yaw = point[2] if len(point) > 2 else 0
            rospy.loginfo(f"\n[GOAL {i+1}/{len(path)}] Raggiungendo punto: x={x}, y={y}, yaw={degrees(yaw):.2f}°")
            self.send_goal(x, y, yaw_degrees=yaw)
            rospy.sleep(1)  # breve pausa tra i goal (opzionale)


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
    
    repetitions = 1  # Numero di ripetizioni del quadrato

    for i in range(repetitions):
        rospy.loginfo(f"\n{('='*20)}\nEsecuzione quadrato numero {i+1}/{repetitions}\n{('='*20)}")
        leader.follow_path(square_path)
        rospy.sleep(2)  # pausa opzionale tra un quadrato e l'altro

    
    # daje


