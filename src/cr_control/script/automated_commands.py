#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("automatic_commands_publisher", )
    cmd_pub = rospy.Publisher("/cr_velocity_controller/cmd_vel", data_class=Twist, queue_size=100)
    rate = rospy.Rate(100)

    msg = Twist()
    msg.linear.x = 0.25
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    seconds = 5

    prev_time = None
    now = rospy.now()
    while seconds > 0 and not rospy.is_shutdown():
        prev_time = now
        now = rospy.now()

        cmd_pub.publish(msg)

        seconds -= (now - prev_time)
        rate.sleep()

if __name__ == "__main__":
    main()
