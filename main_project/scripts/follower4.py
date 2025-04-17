#!/usr/bin/env python

import rospy
import math
from collections import deque
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

class Follower:
    def __init__(self):
        rospy.init_node("follower_node")

        self.pose_follower = None
        self.leader_poses = deque(maxlen=800)  # Memoria di circa x secondi a 10 Hz
        self.cmd_pub = rospy.Publisher("/mir_follower/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.leader_callback)
        rospy.Subscriber("/mir_follower/mir_pose_simple", Pose, self.follower_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.last_leader_position = None
        self.leader_stable_count = 0  # Conta per vedere se il leader è fermo
        self.LEADER_STABLE_THRESHOLD = 20  # Numero di cicli prima di considerare il leader fermo
        self.SAFE_DISTANCE = 1.0  # Distanza di sicurezza minima dal leader

    def leader_callback(self, msg):
        """Salva la posa del leader."""
        current_position = (msg.position.x, msg.position.y)

        # Controlla se il leader è fermo
        if self.last_leader_position is not None:
            movement = math.sqrt(
                (current_position[0] - self.last_leader_position[0])**2 +
                (current_position[1] - self.last_leader_position[1])**2
            )
            if movement < 0.02:  # Soglia per considerarlo fermo
                self.leader_stable_count += 1
            else:
                self.leader_stable_count = 0
        self.last_leader_position = current_position

        self.leader_poses.append(msg)

    def follower_callback(self, msg):
        """Aggiorna la posizione corrente del follower."""
        self.pose_follower = msg

    def follow_leader(self):
        while not rospy.is_shutdown():
            if len(self.leader_poses) < self.leader_poses.maxlen or self.pose_follower is None:
                self.rate.sleep()
                continue  # Aspetta di avere abbastanza pose memorizzate

            # 📍 Recupera la posa "passata" del leader
            target_pose = self.leader_poses.popleft()

            # 📍 Calcola la distanza tra il follower e la posa target
            dx = target_pose.position.x - self.pose_follower.position.x
            dy = target_pose.position.y - self.pose_follower.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # 📍 Calcola l'angolo verso la posa target
            angle_to_target = math.atan2(dy, dx)

            # 📍 Estrai l'orientazione attuale del follower
            orientation_q = self.pose_follower.orientation
            _, _, yaw_follower = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # 📌 Crea il comando
            cmd = Twist()

            # 🚀 **Controllo della velocità lineare**
            if distance > self.SAFE_DISTANCE:
                cmd.linear.x = min(0.5, 0.3 * distance)  # Aumenta la velocità se è lontano
            else:
                cmd.linear.x = 0.0  # Se è vicino, fermati

            # 📍 **Controllo della rotazione per allinearsi alla posa target**
            angle_diff = angle_to_target - yaw_follower
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizza tra -pi e pi

            cmd.angular.z = max(-0.3, min(0.3, 0.5 * angle_diff))  # Limita la velocità angolare

            # 🚦 **Condizione di STOP**: Se il leader è fermo da troppo tempo e il follower è vicino, si ferma completamente
            if self.leader_stable_count > self.LEADER_STABLE_THRESHOLD and distance < self.SAFE_DISTANCE:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # 📤 Pubblica il comando
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
