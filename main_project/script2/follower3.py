#!/usr/bin/env python3
# PID manuale

import rospy
import math
from collections import deque
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
import numpy as np  # Per calcolare la deviazione standard

class Follower:
    def __init__(self):
        rospy.init_node("follower_node")
        
        self.pose_follower = None
        self.leader_path = deque(maxlen=80)  # Coda FIFO per seguire la traiettoria del leader
        self.cmd_pub = rospy.Publisher("/mir_follower/mobile_base_controller/cmd_vel", Twist, queue_size=80)

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

        # Timer per verificare il tempo in cui i robot sono fermi
        self.frozen_time = 0.0  # Tempo in cui entrambi i robot sono fermi
        self.freeze_threshold = 60.0  # Tempo limite di 60 secondi

        # Variabili per calcolare velocità media e precisione
        self.total_distance = 0.0
        self.time_elapsed = 0.0
        self.prev_time = rospy.get_time()

        # Variabili per calcolare la deviazione standard
        self.positions = []

    def leader_callback(self, msg):
        """ Memorizza la posizione del leader nella coda """
        self.leader_path.append((msg.position.x, msg.position.y, msg.orientation))

    def follower_callback(self, msg):
        """ Aggiorna la posizione del follower """
        self.pose_follower = msg
        self.positions.append((msg.position.x, msg.position.y))

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

            # Calcola il tempo del ciclo
            cycle_time = rospy.get_time() - self.prev_time
            self.prev_time = rospy.get_time()

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
            if leader_movement < 0.02 and distance < 3.0:    
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
                self.frozen_time += cycle_time  # Incrementa il timer ogni ciclo
            else:
                self.frozen_time = 0.0  # Reset del timer se il leader si muove

                # Manteniamo una distanza minima tra leader e follower
                if distance >= 3.0:
                    cmd.linear.x = 0.3 * distance
                elif distance < 3.0 and distance >= 1.0:
                    cmd.linear.x = 0.2 * distance                
                else:
                    cmd.linear.x = 0.1 * distance

            # Controllo della rotazione per allinearsi al leader
            if abs(angle_diff) > 0.1:
                cmd.angular.z = max(-0.5, min(0.5, 0.7 * angle_diff))

            # Se il tempo fermo supera la soglia, fermiamo completamente il robot
            if self.frozen_time >= self.freeze_threshold:
                rospy.loginfo("Entrambi i robot sono fermi per più di 60 secondi. Fermando il sistema.")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # Calcola la velocità media
            self.time_elapsed += cycle_time
            velocity = cmd.linear.x
            self.total_distance += velocity * cycle_time
            avg_velocity = self.total_distance / self.time_elapsed if self.time_elapsed > 0 else 0.0

            # Calcola la precisione (deviazione standard)
            positions_array = np.array(self.positions)
            if len(self.positions) > 1:
                x_diff = positions_array[:, 0] - target_x
                y_diff = positions_array[:, 1] - target_y
                trajectory_precision = np.std(np.sqrt(x_diff**2 + y_diff**2))
            else:
                trajectory_precision = 0.0

            # Log delle informazioni
            rospy.loginfo(f"Tempo ciclo: {cycle_time:.2f} s")
            rospy.loginfo(f"Tempo fermo: {self.frozen_time:.2f} s")
            rospy.loginfo(f"Velocità media: {avg_velocity:.2f} m/s")
            rospy.loginfo(f"Precisione traiettoria (deviazione standard): {trajectory_precision:.2f} m")
            
            # Pubblica il comando
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
