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
        self.current_yaw = None
        
        self.cmd_vel_pub = rospy.Publisher("/mir_leader/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        
        rospy.Subscriber("/mir_leader/mir_pose_simple", Pose, self.pose_callback)

        # Contatori
        self.goals_reached = 0
        self.goals_failed = 0
        self.goals_timeout = 0
        self.goals_aborted = 0
        
        # Metriche
        self.total_time = 0
        self.stop_time = 0
        self.avg_speed = 0
        self.avg_deviation = 0
        self.std_deviation = 0
        self.speed_sum = 0
        self.deviations = []

    def pose_callback(self, msg):
        self.current_x = msg.position.x
        self.current_y = msg.position.y

        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)

        rospy.loginfo(f"Aggiornata posizione: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.2f}°")

    def is_goal_reached(self, x, y, yaw, pos_tolerance=0.3333, yaw_tolerance=radians(200.33)):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return False

        distance = sqrt((self.current_x - x) ** 2 + (self.current_y - y) ** 2)
        yaw_diff = abs(self.current_yaw - yaw)
        yaw_diff = min(yaw_diff, 2 * pi - yaw_diff)
        
        rospy.loginfo(f"Distanza: {distance:.2f} (tolleranza: {pos_tolerance}) | Diff. yaw: {math.degrees(yaw_diff):.2f}° (tolleranza: {math.degrees(yaw_tolerance):.2f}°)")

        return distance <= pos_tolerance and yaw_diff <= yaw_tolerance

    def send_goal(self, x, y, yaw_degrees, pos_tolerance=0.3333, yaw_tolerance=radians(200.33)):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        yaw = radians(yaw_degrees)
        quat = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        rospy.loginfo(f"Inviando goal MBF: x={x}, y={y}, yaw={yaw_degrees}°")
        self.client.send_goal(goal)

        rate = rospy.Rate(10)
        start_time = time.time()
        timeout = 45
        goal_reached = False

        while not rospy.is_shutdown():
            state = self.client.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("MBF segnala che il goal è stato raggiunto!")
                self.goals_reached += 1
                self.stop_robot()
                goal_reached = True
                break

            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("MBF ha fallito il goal, lo annullo.")
                self.goals_aborted += 1
                self.client.cancel_goal()
                rospy.sleep(0.5)
                break

            if self.is_goal_reached(x, y, yaw, pos_tolerance, yaw_tolerance):
                rospy.loginfo(f"\033[92mGoal: x={x}, y={y}, yaw={degrees(yaw):.2f}° raggiunto entro le tolleranze!\033[0m")
                self.client.cancel_goal()
                self.stop_robot()
                self.goals_reached += 1
                goal_reached = True
                break

            if time.time() - start_time > timeout:
                rospy.logwarn("Timeout raggiunto, annullo il goal!")
                self.goals_timeout += 1
                self.client.cancel_goal()
                rospy.sleep(1)
                break

            rate.sleep()

        # Calcolare il tempo totale ed altre metriche
        end_time = time.time()
        self.total_time = end_time - start_time
        
        # Calcolare tempo fermo (quando la velocità lineare è zero)
        if goal_reached:
            self.stop_time += 1 if self.cmd_vel_pub.get_num_connections() == 0 else 0
        
        return goal_reached

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.sleep(0.5)

    def follow_path(self, path):
        for i, point in enumerate(path):
            x, y = point[:2]
            yaw = point[2] if len(point) > 2 else 0
            rospy.loginfo(f"\n[GOAL {i+1}/{len(path)}] Raggiungendo punto: x={x}, y={y}, yaw={degrees(yaw):.2f}°")
            self.send_goal(x, y, yaw_degrees=yaw)
            rospy.sleep(1)
        
        # Dopo aver completato tutti i goal
        self.stop_follower()

    def stop_follower(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("Follower fermato dopo aver completato il percorso.")

    def calculate_metrics(self, path):
        # Calcolare la velocità media e la deviazione standard delle distanze
        distance_sum = 0
        for point in path:
            x, y, yaw = point
            distance = sqrt((self.current_x - x) ** 2 + (self.current_y - y) ** 2)
            self.deviations.append(distance)
            distance_sum += distance
        
        self.avg_deviation = distance_sum / len(path)
        self.std_deviation = math.sqrt(sum([(d - self.avg_deviation) ** 2 for d in self.deviations]) / len(path))
        self.avg_speed = self.speed_sum / len(path)

if __name__ == '__main__':
    leader = LeaderMBF()
    
    square_path = [
        (38.5, 40.5, -180),
        (38.5, 47.5, 90),
        (45.5, 47.5, 0),
        (45.5, 40.5, -90)
    ]
    
    repetitions = 15

    for i in range(repetitions):
        rospy.loginfo(f"\n{('='*20)}\nEsecuzione quadrato numero {i+1}/{repetitions}\n{('='*20)}")
        leader.follow_path(square_path)
        rospy.sleep(1)

    # Dopo aver finito tutte le ripetizioni
    rospy.loginfo("\n=================================== RISULTATI FINALI ===================================")
    rospy.loginfo(f"Goals raggiunti correttamente: {leader.goals_reached}")
    rospy.loginfo(f"Goals falliti da MBF: {leader.goals_aborted}")
    rospy.loginfo(f"Goals falliti per timeout: {leader.goals_timeout}")
    rospy.loginfo("=========================================================")
    rospy.loginfo(f"Tempo totale: {leader.total_time:.2f} secondi")
    rospy.loginfo(f"Tempo fermo: {leader.stop_time * 0.1:.2f} secondi")
    rospy.loginfo(f"Velocità media: {leader.avg_speed:.2f} m/s")
    rospy.loginfo(f"Precisione traiettoria: {leader.avg_deviation:.2f} m")
    rospy.loginfo(f"Deviazione standard della traiettoria: {leader.std_deviation:.2f} m")
    rospy.loginfo("=========================================================")
