import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from roborts_msgs.msg import TwistAccel
from playsound import playsound
import csv
import random
import math
import threading
import time
import os
import sys
from codedetect.srv import *

#amcl_sub = rospy.Subscriber(
            #"/amcl_pose", PoseWithCovarianceStamped, amclCallback, queue_size=1)
assist_parking_pub = rospy.Publisher(
            "/cmd_vel_acc", TwistAccel, queue_size=1)

def amclCallback(pose):
    current_map_pose=pose
    current_map_yaw = math.atan2(2 * (current_map_pose.pose.pose.orientation.w * current_map_pose.pose.pose.orientation.z), 1 - 2 * current_map_pose.pose.pose.orientation.z * current_map_pose.pose.pose.orientation.z)

def marker_client(parking_id):
    rospy.loginfo("Waiting for mark service")
    rospy.wait_for_service("codedetect")
    b = 20
    while b > 0:
        try:
            cuda = rospy.ServiceProxy("codedetect", code)
            req = cuda(int(parking_id))
            assist_parking(
                req.twist.linear.x, req.twist.linear.y, req.twist.linear.z)
            b = 0
        except rospy.ServiceException as e:
            print("no parking code %s", e)
            b -= 1
            time.sleep(0.01)

def assist_parking(d_x, d_y, d_yaw):
    if d_x < 0.15:
        print("No need to assist")
        return
    start_yaw = 1.57 - d_yaw
    dt = 1.0
    V_x = d_x/dt
    V_y = d_y/dt
    V_yaw = d_yaw/dt
    print("Assisting", "d_x", d_x, "d_y", d_y, "d_yaw", d_yaw, "start_yaw", start_yaw )
    start_time=time.time() * 1000.0
    cmd_vel_acc = TwistAccel()
    while True:
        current_time = time.time() * 1000.0
        diff_time = current_time - start_time
        if diff_time > dt * 1000.0:
            break
        current_yaw = start_yaw + V_yaw * diff_time /1000
        cmd_vel_acc.twist.linear.x = V_x*math.sin(current_yaw) + V_y*math.cos(current_yaw)
        cmd_vel_acc.twist.linear.y = V_x*math.cos(current_yaw) - V_y*math.sin(current_yaw)
        cmd_vel_acc.twist.angular.z = -V_yaw
        assist_parking_pub.publish(cmd_vel_acc)
    cmd_vel_acc.twist.linear.x = 0
    cmd_vel_acc.twist.linear.y = 0
    cmd_vel_acc.twist.angular.z = 0
    assist_parking_pub.publish(cmd_vel_acc)

if __name__=="__main__":
    rospy.init_node('assist_parking')
    marker_client(int(sys.argv[1]))
    rospy.spin()
