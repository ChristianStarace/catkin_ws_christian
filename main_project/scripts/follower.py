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
        self.leader_path = deque(maxlen=80)  # Coda FIFO per seguire la traiettoria del leader
        self.cmd_pub = rospy.Publisher("/mir_follower/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.leader_callback)
        rospy.Subscriber("/mir_follower/mir_pose_simple", Pose, self.follower_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.last_leader_position = None  # Per rilevare se il leader è fermo

    def leader_callback(self, msg):
        """ Memorizza la posizione del leader nella coda """
        self.leader_path.append((msg.position.x, msg.position.y, msg.orientation))

    def follower_callback(self, msg):
        """ Aggiorna la posizione del follower """
        self.pose_follower = msg

    def follow_leader(self):
        while not rospy.is_shutdown():
            if not self.leader_path or self.pose_follower is None:
                self.rate.sleep()
                continue

            # Prendi la posizione che il leader ha attraversato un istante fa
            target_x, target_y, target_orientation = self.leader_path.popleft()

            # Calcola la distanza tra il follower e il punto target
            dx = target_x - self.pose_follower.position.x
            dy = target_y - self.pose_follower.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # Se il follower è vicino al punto target, rimuoviamo il punto dalla coda
            # if distance < 0.3:  
            #     self.leader_path.popleft()
            #     continue

            # Calcola l'angolo tra il follower e la posizione target
            angle_to_target = math.atan2(dy, dx)

            # Estrai l'orientazione attuale del follower
            orientation_q = self.pose_follower.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # Controlliamo se il leader è fermo
            leader_movement = 1  # Assume che il leader si muova
            if self.last_leader_position is not None:
                leader_dx = abs(target_x - self.last_leader_position[0])
                leader_dy = abs(target_y - self.last_leader_position[1])
                leader_movement = leader_dx + leader_dy
            
            self.last_leader_position = (target_x, target_y)

            # Crea il comando di movimento
            cmd = Twist()

            # Se il leader è fermo e il follower è vicino, fermiamo il follower
            if leader_movement < 0.02 and distance < 2.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
            else:
                # Manteniamo una distanza minima tra leader e follower
                if distance >= 2.0:
                    cmd.linear.x = 0.3 * distance
                elif distance < 2.0 and distance >= 1.0:
                    cmd.linear.x = 0.2 * distance
                else:
                    cmd.linear.x = 0.1 * distance  

            # Controllo della rotazione per allinearsi al leader
            angle_diff = angle_to_target - yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizza tra -pi e pi

            if abs(angle_diff) > 0.05:  # Soglia più bassa per ridurre oscillazioni
                cmd.angular.z = max(-0.3, min(0.3, 0.5 * angle_diff))


            # Pubblica il comando
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
