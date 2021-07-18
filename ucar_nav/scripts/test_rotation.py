#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from roborts_msgs.msg import TwistAccel
from playsound import playsound
import csv
import  random
import  math
import time
import  os

if __name__ == '__main__':
    rospy.init_node("test_rotation")
    cmd_vel_acc = rospy.Publisher('cmd_val_acc',TwistAccel,queue_size = 1)
    time.sleep(0.5)
    cmd = TwistAccel()
    cmd.twist.angular.z = 3.14
    cmd_vel_acc.publish(cmd)
    time.sleep(1)
    cmd.twist.linear.x = 0
    cmd.twist.linear.y = 0
    cmd.twist.angular.z = 0
    cmd_vel_acc.publish(cmd)
    print("done\n")