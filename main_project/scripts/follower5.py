#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

class Follower:
    def __init__(self):
        rospy.init_node("follower_node")
        
        self.pose_leader = None
        self.pose_follower = None
        self.cmd_pub = rospy.Publisher("/mir_follower/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.leader_callback)
        rospy.Subscriber("/mir_follower/mir_pose_simple", Pose, self.follower_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.last_leader_position = None  
        self.last_leader_orientation = None  
        self.prev_linear_x = 0.0  
        self.prev_angular_z = 0.0  

    def leader_callback(self, msg):
        self.pose_leader = msg

    def follower_callback(self, msg):
        self.pose_follower = msg

    def follow_leader(self):
        while not rospy.is_shutdown():
            if self.pose_leader is None or self.pose_follower is None:
                self.rate.sleep()
                continue

            # 📍 Calcola la distanza tra leader e follower
            dx = self.pose_leader.position.x - self.pose_follower.position.x
            dy = self.pose_leader.position.y - self.pose_follower.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # 📍 Calcola l'angolo tra il follower e il leader
            angle_to_leader = math.atan2(dy, dx)

            # 📍 Estrai l'orientazione attuale del follower
            orientation_q = self.pose_follower.orientation
            _, _, yaw_follower = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # 📍 Estrai l'orientazione attuale del leader
            orientation_q_leader = self.pose_leader.orientation
            _, _, yaw_leader = euler_from_quaternion([orientation_q_leader.x, orientation_q_leader.y, orientation_q_leader.z, orientation_q_leader.w])

            # 🔍 Controlliamo se il leader è fermo
            leader_movement = 1  
            if self.last_leader_position is not None:
                leader_dx = abs(self.pose_leader.position.x - self.last_leader_position[0])
                leader_dy = abs(self.pose_leader.position.y - self.last_leader_position[1])
                leader_movement = leader_dx + leader_dy  
            
            self.last_leader_position = (self.pose_leader.position.x, self.pose_leader.position.y)

            # 🔍 Controlliamo se il leader sta ruotando
            leader_rotation = 0
            if self.last_leader_orientation is not None:
                leader_rotation = abs(yaw_leader - self.last_leader_orientation)  
            
            self.last_leader_orientation = yaw_leader

            # 🛑 **Zona di sicurezza dinamica**
            safe_distance = 4.5  
            if leader_rotation > 0.02:  
                safe_distance = 5  # ➡️ Se il leader ruota, teniamo più distanza

            # 📌 Crea il comando
            cmd = Twist()
                
            # 🚨 **STOP anticipato in curva** se il leader si ferma mentre gira
            if leader_movement < 0.02 and distance < safe_distance:  
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
            else:
                # 📍 **Controllo della rotazione per allinearsi al leader**
                angle_diff = angle_to_leader - yaw_follower
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  

                # 🎯 **Curva più ampia per il follower**
                if leader_rotation > 0.02:
                    angle_diff *= 0.5  # ➡️ Se il leader ruota, riduciamo l'angolo del follower

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
