#!/usr/bin/env python3
# PID manuale

import rospy
import math
from collections import deque
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

class Follower:
    def __init__(self):
        rospy.init_node("follower_node")
        
        self.pose_follower = None
        self.leader_path = deque(maxlen=80)  # Coda FIFO per seguire la traiettoria del leader
        self.cmd_pub = rospy.Publisher("/mir_follower/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.leader_callback)
        rospy.Subscriber("/mir_follower/mir_pose_simple", Pose, self.follower_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.last_leader_position = None  # Per rilevare se il leader è fermo

        # Parametri PID per il movimento lineare
        self.kp_linear = 0.5
        self.ki_linear = 0.0
        self.kd_linear = 0.1
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        # Parametri PID per la rotazione angolare
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1
        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

    def leader_callback(self, msg):
        """ Memorizza la posizione del leader nella coda """
        self.leader_path.append((msg.position.x, msg.position.y, msg.orientation))

    def follower_callback(self, msg):
        """ Aggiorna la posizione del follower """
        self.pose_follower = msg

    def pid(self, error, prev_error, integral, kp, ki, kd):
        """ Calcola l'uscita del PID data l'errore, l'errore precedente e l'integrale """
        integral += error
        derivative = error - prev_error
        output = kp * error + ki * integral + kd * derivative
        return output, prev_error, integral

    def follow_leader(self):
        while not rospy.is_shutdown():
            if not self.leader_path or self.pose_follower is None:
                self.rate.sleep()
                continue

            # Prendi la posizione che il leader ha attraversato un istante fa
            target_x, target_y, target_orientation = self.leader_path[0]

            # Calcola la distanza tra il follower e il punto target
            dx = target_x - self.pose_follower.position.x
            dy = target_y - self.pose_follower.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # Calcola l'angolo tra il follower e la posizione target
            angle_to_target = math.atan2(dy, dx)

            # Estrai l'orientazione attuale del follower
            orientation_q = self.pose_follower.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # Calcola la differenza angolare tra il follower e la posizione target
            angle_diff = angle_to_target - yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizza tra -pi e pi

            # --- Calcolo del comando lineare con PID ---
            linear_error = distance
            cmd_linear, self.prev_error_linear, self.integral_linear = self.pid(
                linear_error, self.prev_error_linear, self.integral_linear, self.kp_linear, self.ki_linear, self.kd_linear
            )

            # --- Calcolo del comando angolare con PID ---
            angular_error = angle_diff
            cmd_angular, self.prev_error_angular, self.integral_angular = self.pid(
                angular_error, self.prev_error_angular, self.integral_angular, self.kp_angular, self.ki_angular, self.kd_angular
            )

            # Controlliamo se il leader è fermo
            leader_movement = 1  # Assume che il leader si muova
            if self.last_leader_position is not None:
                leader_dx = abs(target_x - self.last_leader_position[0])
                leader_dy = abs(target_y - self.last_leader_position[1])
                leader_movement = leader_dx + leader_dy
            
            self.last_leader_position = (target_x, target_y)

            cmd = Twist()

            # Se il leader è fermo e il follower è vicino, fermiamo il follower
            #if leader_movement < 0.02 and distance < 2.0:
            if leader_movement < 0.15 and distance < 2.5:    
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
            else:
                # Manteniamo una distanza minima tra leader e follower
                if distance >= 3.0:
                    cmd.linear.x = 0.15 * distance
                elif distance < 3.0 and distance >= 2.5:
                    cmd.linear.x = 0.1 * distance                
                else:
                    cmd.linear.x = 0 #0.05 * distance

            # Controllo della rotazione per allinearsi al leader
            angle_diff = angle_to_target - yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizza tra -pi e pi

            if abs(angle_diff) > 0.1:
                cmd.angular.z = max(-0.5, min(0.5, 0.7 * angle_diff))

            # Pubblica il comando
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
