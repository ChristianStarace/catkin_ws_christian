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
        self.leader_poses = deque(maxlen=80)  # 8 secondi di ritardo (10 Hz * 8s)
        self.cmd_pub = rospy.Publisher("/mir_follower/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.leader_callback)
        rospy.Subscriber("/mir_follower/mir_pose_simple", Pose, self.follower_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.prev_linear_x = 0.0  
        self.prev_angular_z = 0.0  
        self.last_leader_orientation = None  

    def leader_callback(self, msg):
        self.leader_poses.append(msg)

    def follower_callback(self, msg):
        self.pose_follower = msg

    def follow_leader(self):
        while not rospy.is_shutdown():
            if self.pose_follower is None or len(self.leader_poses) < 80:
                rospy.loginfo("In attesa di dati sufficienti...")
                self.rate.sleep()
                continue

            # 📍 Prendi la posa più vecchia salvata del leader
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

            # 📍 Estrai l'orientazione della posa target
            orientation_q_target = target_pose.orientation
            _, _, yaw_target = euler_from_quaternion([orientation_q_target.x, orientation_q_target.y, orientation_q_target.z, orientation_q_target.w])

            # 🔍 Controllo movimento e rotazione del leader
            leader_rotation = 0
            if self.last_leader_orientation is not None:
                leader_rotation = abs(yaw_target - self.last_leader_orientation)  
            self.last_leader_orientation = yaw_target

            # 🛑 **Zona di sicurezza dinamica**
            safe_distance = 2.5  
            if leader_rotation > 0.02:  
                safe_distance = 3  # ➡️ Se il leader ruota, teniamo più distanza

            # 📌 Crea il comando
            cmd = Twist()
            
            # 🚨 **STOP anticipato in curva**
            if distance < safe_distance:  
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
            else:
                # 📍 **Controllo della rotazione per allinearsi alla posa target**
                angle_diff = angle_to_target - yaw_follower
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  

                # 🎯 **Curva più ampia per il follower**
                if leader_rotation > 0.02:
                    angle_diff *= 0.5  

                cmd.angular.z = max(-0.3, min(0.3, 0.5 * angle_diff))  

                # 🚀 **Velocità lineare più intelligente**
                if leader_rotation > 0.02:  
                    target_linear_x = min(0.2, 0.05 * distance)  
                else:
                    if distance > safe_distance:
                        target_linear_x = min(0.5, 0.3 * distance)
                    elif distance > 1.5:
                        target_linear_x = 0.2 * distance
                    else:
                        target_linear_x = 0.0  

                # 🔹 **Smooth Transition**
                alpha = 0.1  
                cmd.linear.x = (1 - alpha) * self.prev_linear_x + alpha * target_linear_x
                cmd.angular.z = (1 - alpha) * self.prev_angular_z + alpha * cmd.angular.z

                # Memorizza le velocità precedenti
                self.prev_linear_x = cmd.linear.x
                self.prev_angular_z = cmd.angular.z

            # 📤 Pubblica il comando
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
