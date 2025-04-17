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
        self.last_leader_position = None  # Per rilevare se il leader è fermo

    def leader_callback(self, msg):
        self.pose_leader = msg

    def follower_callback(self, msg):
        self.pose_follower = msg

    def follow_leader(self):
        while not rospy.is_shutdown():
            if self.pose_leader is None or self.pose_follower is None:
                self.rate.sleep()
                continue

            # Calcola la distanza tra leader e follower
            dx = self.pose_leader.position.x - self.pose_follower.position.x
            dy = self.pose_leader.position.y - self.pose_follower.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # Calcola l'angolo tra il follower e il leader
            angle_to_leader = math.atan2(dy, dx)

            # Estrai l'orientazione attuale del follower
            orientation_q = self.pose_follower.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

# Controlliamo se il leader è fermo
            leader_movement = 1  # All'inizio assumiamo che il leader si stia muovendo
            if self.last_leader_position is not None:
                leader_dx = abs(self.pose_leader.position.x - self.last_leader_position[0])
                leader_dy = abs(self.pose_leader.position.y - self.last_leader_position[1])
                leader_movement = leader_dx + leader_dy
            
            self.last_leader_position = (self.pose_leader.position.x, self.pose_leader.position.y)
            
            # Crea il comando
            cmd = Twist()
                
            # Se il leader è fermo e il follower è vicino, fermiamo il follower
            if leader_movement < 0.02 and distance < 2.5:  # Leader fermo e follower vicino
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0  
            else:
                # Velocità lineare proporzionale alla distanza
                if distance > 2.0:
                    cmd.linear.x = min(0.5, 0.3 * distance)
                elif distance > 1:
                    cmd.linear.x = 0.2 * distance
                else:
                    cmd.linear.x = 0.0  

            # Controllo della rotazione per allinearsi al leader
            angle_diff = angle_to_leader - yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalizza tra -pi e pi

            if abs(angle_diff) > 0.1:
                cmd.angular.z = max(-0.5, min(0.5, 0.7 * angle_diff))  # Limita la velocità angolare

            # Pubblica il comando
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass

