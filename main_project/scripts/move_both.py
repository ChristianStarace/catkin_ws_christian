#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_both():
    rospy.init_node('move_both_robots', anonymous=True)
    
    pub_leader = rospy.Publisher('/mir_leader/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    pub_follower = rospy.Publisher('/mir_follower/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    cmd = Twist()
    cmd.linear.x = 0.5  # Velocità lineare in avanti

    while not rospy.is_shutdown():
        pub_leader.publish(cmd)
        pub_follower.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_both()
    except rospy.ROSInterruptException:
        pass