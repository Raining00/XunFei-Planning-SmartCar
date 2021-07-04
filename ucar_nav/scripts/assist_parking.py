#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from roborts_msgs.msg import TwistAccel
import time

if __name__ == '__main__':
    rospy.init_node("assist_parking")
    cmd_vel_acc = rospy.Publisher("/cmd_vel_acc",TwistAccel,queue_size=1)
    cmd = TwistAccel()
    time.sleep(0.5)
    cmd.twist.linear.x = -0.3
    cmd.twist.linear.y = -0.3
    cmd.twist.angular.z = -0.6
    cmd_vel_acc.publish(cmd)
    time.sleep(0.6)
    cmd.twist.linear.x = 0
    cmd.twist.linear.y = 0
    cmd.twist.angular.z = 0
    cmd_vel_acc.publish(cmd)
    print("done\n")