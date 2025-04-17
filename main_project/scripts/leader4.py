#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose, Twist
from mbf_msgs.msg import MoveBaseActionResult
from tf.transformations import euler_from_quaternion

class Leader:
    def __init__(self):
        rospy.init_node('leader_move_base_flex_node')
        
        self.pose_leader = None  # Inizializza come None
        self.target_position = None
        
        # Subscriber per la posizione del leader
        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.pose_callback)

        # Publisher per il controllo del leader
        self.cmd_vel_pub = rospy.Publisher("/mir_leader/mobile_base_controller/cmd_vel", Twist, queue_size=10)

    def pose_callback(self, msg):
        #rospy.loginfo("Ricevuta nuova posizione leader!")
        self.pose_leader = msg  # Adesso self.pose_leader è un oggetto Pose, non PoseStamped

    def compute_distance(self):
        if self.pose_leader is None or self.target_position is None:
            rospy.logwarn("Dati posizione non disponibili.")
            return None
        
        dx = self.target_position[0] - self.pose_leader.position.x
        dy = self.target_position[1] - self.pose_leader.position.y
        return math.sqrt(dx**2 + dy**2)

    def compute_target_angle(self, x_next, y_next):
        if self.pose_leader is None:
            rospy.logwarn("Aspettando dati sulla posizione del leader...")
            return None  # Evita errori se i dati non sono ancora disponibili

        dx = x_next - self.pose_leader.position.x
        dy = y_next - self.pose_leader.position.y
        return math.atan2(dy, dx)

    def control_speed(self):
        if self.pose_leader is None:
            rospy.logwarn("Posizione leader non disponibile.")
            return  # Non possiamo calcolare senza la posizione attuale

        distance = self.compute_distance()
        if distance is None:
            return

        target_angle = self.compute_target_angle(self.target_position[0], self.target_position[1])
        if target_angle is None:
            return

        # Estrai l'orientamento attuale del leader
        orientation_q = self.pose_leader.orientation
        _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calcola l'errore angolare
        angle_error = target_angle - current_yaw
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi  # Normalizza l'errore tra -pi e pi

        # Log per monitorare i valori
        rospy.loginfo(f"Distanza al target: {distance}")
        rospy.loginfo(f"Errore angolare: {angle_error} rad")

        # Creazione del messaggio Twist
        cmd = Twist()
        cmd.linear.x = min(0.5, distance)  # Limitiamo la velocità lineare
        cmd.angular.z = max(-0.5, min(0.5, 2.0 * angle_error))  # Limitiamo la velocità angolare

        self.cmd_vel_pub.publish(cmd)

    def follow_path(self, waypoints):
        rate = rospy.Rate(10)  # Frequenza di aggiornamento

        for waypoint in waypoints:
            self.target_position = waypoint
            rospy.loginfo(f"Nuova destinazione: {waypoint}")

            while True:
                distance = self.compute_distance()
                if distance is None:
                    rospy.logwarn("Distanza non disponibile, in attesa di dati...")
                    rate.sleep()
                    continue  # Continua ad aspettare i dati
                
                if distance <= 0.2:
                    break  # Uscire dal loop se la distanza è sufficiente

                self.control_speed()
                rate.sleep()

        rospy.loginfo("Percorso completato, terminazione del nodo.")

if __name__ == '__main__':
    leader = Leader()
    
    waypoints = [ (26.0, 28.0), (30.0, 37.0), (21.0, 41.0), (17.0, 32.0), (26.0, 28.0)]
    leader.follow_path(waypoints)




# import rospy
# import actionlib
# from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal  # Usa il pacchetto corretto di MBF
# from geometry_msgs.msg import PoseStamped

# class LeaderMBF:
#     def __init__(self):
#         rospy.init_node('leader_move_base_flex_node', anonymous=True)

#         # Crea un Action Client per Move Base Flex
#         self.client = actionlib.SimpleActionClient('/mir_leader/move_base_flex/move_base', MoveBaseAction)
#         rospy.loginfo("Aspettando il server di Move Base Flex...")
#         self.client.wait_for_server()
#         rospy.loginfo("Server MBF trovato!")

#     def send_goal(self, x, y):
#         goal = MoveBaseGoal()
#         goal.target_pose = PoseStamped()  # Crea un PoseStamped per il target

#         goal.target_pose.header.stamp = rospy.Time.now()
#         goal.target_pose.header.frame_id = "map"
#         goal.target_pose.pose.position.x = x
#         goal.target_pose.pose.position.y = y
#         goal.target_pose.pose.orientation.w = 1.0  # Nessuna rotazione

#         rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}")
#         self.client.send_goal(goal)
#         self.client.wait_for_result()

#         result = self.client.get_result()
#         rospy.loginfo(f"Goal completato, risultato: {result}")

# if __name__ == '__main__':
#     leader = LeaderMBF()
#     leader.send_goal(28.0, 28.0)  # Cambia le coordinate se necessario




