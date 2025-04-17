#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_leader():
    rospy.init_node('leader_node', anonymous=True)
    pub = rospy.Publisher('/mir_leader/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    cmd = Twist()
    cmd.linear.x = 0.4  # Velocità in avanti
    cmd.angular.z = 0.1 # Piccola rotazione

    while not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_leader()
    except rospy.ROSInterruptException:
        pass
