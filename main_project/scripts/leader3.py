#!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Pose
# import math

# class Leader:
#     def __init__(self):
#         rospy.init_node('leader_node', anonymous=True)
#         self.cmd_vel_pub = rospy.Publisher('/mir_leader/mobile_base_controller/cmd_vel', Twist, queue_size=10)
#         self.current_position = None
#         self.current_orientation = None
#         self.target_position = (2.0, 0.0)  # Destinazione in (x, y)
#         self.target_orientation = 0.0  # Orientamento desiderato in radianti (ad esempio 0.0 per orientamento verso est)
#         rospy.Subscriber('/mir_leader/mir_pose_simple', Pose, self.pose_callback)
#         self.rate = rospy.Rate(10)

#     def pose_callback(self, msg):
#         # Estrai la posizione attuale (x, y) e l'orientamento (theta) del leader
#         self.current_position = msg.position
#         # Assumiamo che l'orientamento (theta) sia nel campo orientamento di msg.pose
#         # Se l'orientamento è già in radianti, usiamo direttamente msg.pose.orientation.z o un altro campo
#         self.current_orientation = msg.orientation.z  # Cambia se il campo dell'orientamento è diverso

#     def compute_distance(self):
#         # Calcola la distanza tra la posizione attuale e il target
#         dx = self.target_position[0] - self.current_position.x
#         dy = self.target_position[1] - self.current_position.y
#         return math.sqrt(dx**2 + dy**2)

#     def compute_angle_to_target(self):
#         # Calcola l'angolo tra la posizione attuale e il target
#         dx = self.target_position[0] - self.current_position.x
#         dy = self.target_position[1] - self.current_position.y
#         return math.atan2(dy, dx)

#     def compute_angle_error(self):
#         # Calcola l'errore angolare tra l'orientamento attuale e l'orientamento desiderato
#         angle_error = self.target_orientation - self.current_orientation
#         # Normalizza l'errore angolare tra -pi e pi
#         angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
#         return angle_error

#     def control_speed(self):
#         if self.current_position is None or self.current_orientation is None:
#             return  # Non possiamo calcolare la distanza senza la posizione attuale

#         distance = self.compute_distance()
#         angle_to_target = self.compute_angle_to_target()
#         angle_error = self.compute_angle_error()

#         # Velocità lineare e angolare dipendono dalla distanza, dall'angolo e dall'orientamento
#         linear_speed = min(0.5, distance * 0.5)  # Velocità lineare decresce man mano che si avvicina
#         angular_speed = min(1.0, abs(angle_error) * 1.0)  # Velocità angolare dipende dall'errore angolare

#         # Se la distanza è piccola, fermati
#         if distance < 0.1:
#             linear_speed = 0.0
#             angular_speed = 0.0

#         # Creiamo il comando Twist per muovere il robot
#         cmd = Twist()
#         cmd.linear.x = linear_speed
#         cmd.angular.z = angular_speed

#         self.cmd_vel_pub.publish(cmd)

#     def run(self):
#         while not rospy.is_shutdown():
#             self.control_speed()
#             self.rate.sleep()

# if __name__ == '__main__':
#     leader = Leader()
#     leader.run()



import rospy
import math
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion

class Leader:
    def __init__(self):
        rospy.init_node('leader_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/mir_leader/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        # Posizione e orientamento del leader
        self.pose_leader = None
        self.target_position = (5.0, 5.0)  # Destinazione in (x, y)
        self.tolerance = 0.5  # Tolleranza per considerare il leader arrivato al target

        # Subscrizione al topic /mir_leader/mir_pose_simple
        rospy.Subscriber('/mir_leader/mir_pose_simple', Pose, self.pose_callback)
        self.rate = rospy.Rate(10)

    def pose_callback(self, msg):
        # Estrai la posizione attuale (x, y) e l'orientamento del leader
        self.pose_leader = msg

    def compute_distance(self):
        # Calcola la distanza tra la posizione attuale e la destinazione
        dx = self.target_position[0] - self.pose_leader.position.x
        dy = self.target_position[1] - self.pose_leader.position.y
        return math.sqrt(dx**2 + dy**2)

    def compute_target_angle(self):
        # Calcola l'angolo verso la destinazione
        dx = self.target_position[0] - self.pose_leader.position.x
        dy = self.target_position[1] - self.pose_leader.position.y
        return math.atan2(dy, dx)

    def control_speed(self):
        if self.pose_leader is None:
            return  # Non possiamo calcolare senza la posizione attuale

        distance = self.compute_distance()
        target_angle = self.compute_target_angle()

        # Estrai l'orientamento attuale del leader
        orientation_q = self.pose_leader.orientation
        _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calcola l'errore angolare
        angle_error = target_angle - current_yaw
        # Normalizza l'errore angolare tra -pi e pi
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Log per monitorare i valori
        rospy.loginfo(f"Distanza al target: {distance}")
        rospy.loginfo(f"Errore angolare: {angle_error} rad")

        # Crea il comando Twist
        cmd = Twist()

        # Se la distanza al target è inferiore alla tolleranza, fermati
        if distance < self.tolerance:
            rospy.loginfo("Destinazione raggiunta, fermo il leader")
            cmd.linear.x = 0.0  # Fermati
            cmd.angular.z = 0.0  # Fermati
        else:
            # Se l'errore angolare è significativo, ruota
            if abs(angle_error) > 0.2:
                cmd.linear.x = 0.0  # Fermati per ruotare
                cmd.angular.z = 0.5 if angle_error > 0 else -0.5  # Velocità angolare per ruotare verso la destinazione
            else:
                # Se siamo allineati, muoviamoci verso la destinazione
                if distance > 0.2:
                    cmd.linear.x = min(0.5, 0.3 * distance)  # Velocità lineare proporzionale alla distanza
                    cmd.angular.z = 0.0  # Fermati per muoverti in avanti
                else:
                    cmd.linear.x = 0.0  # Fermati quando sei arrivato al target
                    cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.control_speed()
            self.rate.sleep()

if __name__ == '__main__':
    leader = Leader()
    leader.run()




