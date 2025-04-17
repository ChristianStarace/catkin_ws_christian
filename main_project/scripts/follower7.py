#!/usr/bin/env python
import rospy
import time
from collections import deque
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        rospy.init_node("follower_node")

        self.cmd_pub = rospy.Publisher("/mir_follower/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/mir_leader/mobile_base_controller/cmd_vel", Twist, self.leader_cmd_callback)

        self.cmd_queue = deque()  # Coda per memorizzare i comandi del leader con timestamp
        self.delay = 2.0  # Ritardo in secondi per seguire la scia

        self.rate = rospy.Rate(20)  # Frequenza di aggiornamento (20 Hz)

    def leader_cmd_callback(self, msg):
        """Memorizza i comandi di velocità del leader con timestamp"""
        self.cmd_queue.append((rospy.Time.now().to_sec(), msg))

    def follow_leader(self):
        """Applica i comandi salvati con il ritardo scelto"""
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            # Controlla se abbiamo comandi memorizzati più vecchi del ritardo scelto
            while self.cmd_queue and (current_time - self.cmd_queue[0][0] >= self.delay):
                _, cmd = self.cmd_queue.popleft()  # Prende il comando più vecchio

                self.cmd_pub.publish(cmd)  # Invia il comando al follower

            self.rate.sleep()

if __name__ == "__main__":
    try:
        follower = Follower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
