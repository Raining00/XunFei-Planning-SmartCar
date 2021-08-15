#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from roborts_msgs.msg import TwistAccel
from roborts_msgs.srv import Obstacles
from playsound import playsound
import tf
import csv
import random
import math
import threading
import time
import os
import sys
from codedetect.srv import *
from tkdnn_ros_msgs.srv import *

marker_to_food = {0: "Vegetables",
                  1: "Fruits",
                  2: "Meat"}


def list2PoseStemped(Point):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = Point[0]
    pose.pose.position.y = Point[1]
    pose.pose.position.z = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = Point[2]
    pose.pose.orientation.w = Point[3]
    return pose


# we will define a param named type to describe a navigation goal
# 1 is meaned use distance
# 2
# 3 mission point : QR code
# 4 last point

class mission_task:
    def __init__(self, goal_path, parking_path, parking_id):
        # nav point
        self.nav_list = []
        self.parking_list = []
        self.goal_type_list = []
        self.goal_type = 0
        self.last_state = 0
        with open(goal_path, 'rb') as f:
            reader = csv.reader(f)
            for cols in reader:
                self.nav_list.append([float(values) for values in cols])
                self.goal_type_list.append(int(cols[-1]))
        with open(parking_path, 'rb') as f:
            reader = csv.reader(f)
            for cols in reader:
                self.parking_list.append([float(values) for values in cols])

        # vision about
        self.parking_id = parking_id
        self.food_id = 0
        self.mark_received = False
        self.food_voice = False
        self.vision_count = 0
        self.glasses_num = random.randint(0, 2)
        self.longhair_num = random.randint(0, 2)
        self.swing_head = False
        # navigation about
        self.goal_id = 0
        self.current_goal = PoseStamped()
        self.last_goal = PoseStamped()
        self.current_goal.header.frame_id = "map"
        self.count = 0
        self.state = 0
        self.last_state = 0
        self.current_map_pose = PoseWithCovarianceStamped()
        self.last_map_pose = PoseWithCovarianceStamped()

        self.amcl_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amclCallback, queue_size=1)
        self.voice_sub = rospy.Subscriber(
            "nav_start", Twist, self.voiceCallback, queue_size=1)
        self.change_pub = rospy.Publisher("/change_mode", Twist, queue_size=1)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
        self.assist_parking_pub = rospy.Publisher(
            "/cmd_vel_acc", TwistAccel, queue_size=1)
        self.obsService = rospy.Service("obstacles", Obstacles, self.obsHandle)
        self.coding = False

    def Run(self):
        print("current goal", self.current_goal.pose.position.x, self.current_goal.pose.position.y)
        dx = self.current_goal.pose.position.x - \
             self.current_map_pose.pose.pose.position.x
        dy = self.current_goal.pose.position.y - \
             self.current_map_pose.pose.pose.position.y
        yaw = math.atan2(
            2 * (self.current_map_pose.pose.pose.orientation.w * self.current_map_pose.pose.pose.orientation.z),
            1 - 2 * self.current_map_pose.pose.pose.orientation.z * self.current_map_pose.pose.pose.orientation.z)
        yaw_goal = math.atan2(2 * self.current_goal.pose.orientation.w * self.current_goal.pose.orientation.z,
                              1 - 2 * self.current_goal.pose.orientation.z * self.current_goal.pose.orientation.z)
        distance = math.sqrt(dx * dx + dy * dy)
        dyaw = abs(yaw_goal - yaw)
        print(self.state, "dis", round(distance, 2), "dyaw", round(dyaw, 2), "goal_id",
              self.goal_id, 'goal_type', self.goal_type)
        if self.state == 0:
            return
        elif self.state == 1:
            if self.goal_type == 1:  # 区域切换导航点
                if distance < 0.15 and dyaw < 0.3:
                    if self.goal_id == 1:
                        time.sleep(0.5)
                    if self.goal_id == len(self.nav_list):
                        self.pub_next_goal(is_parking=True)
                        self.state = 2
                        temp_msg = Twist()
                        temp_msg.linear.x = 1
                        self.change_pub.publish(temp_msg)
                    else:
                        self.pub_next_goal()
            elif self.goal_type == 3:  # 任务领取区导航点
                if distance < 0.2  and not self.coding:
                    marker_time = threading.Timer(0.1, self.marker_client)
                    marker_time.daemon = True
                    marker_time.start()
                    self.coding = True
                elif self.mark_received and self.coding:
                    self.pub_next_goal()
        elif self.state == 2:
            if distance < 0.25:
                timer = threading.Timer(4, self.broadcast)
                timer.start()
                self.state = 4
            elif self.count % 2 == 0:
                self.goal_pub.publish(self.current_goal)
            self.count += 1
        elif self.state == 3:
            if dyaw < 0.1:
                self.current_goal.header.frame_id = 'map'
                self.current_goal.header.stamp = rospy.Time.now()
                self.current_goal.pose.position.x = self.last_goal.pose.position.x
                self.current_goal.pose.position.y = self.last_goal.pose.position.y
                self.current_goal.pose.orientation.z = self.last_goal.pose.orientation.z
                self.current_goal.pose.orientation.w = self.last_goal.pose.orientation.w
                print("last goal", self.last_goal.pose.position.x, self.last_goal.pose.position.y)
                print("restore last goal", self.current_goal.pose.position.x, self.current_goal.pose.position.y)
                self.goal_pub.publish(self.current_goal)
                self.state = self.last_state
                print("dead finished")

    def pub_next_goal(self, is_parking=False):
        if not is_parking:
            self.current_goal = list2PoseStemped(self.nav_list[self.goal_id])
            self.goal_type = self.goal_type_list[self.goal_id]
            time.sleep(0.2)
        else:
            self.current_goal = list2PoseStemped(self.parking_list[self.parking_id])
        self.goal_id += 1
        self.goal_pub.publish(self.current_goal)
        self.last_goal.pose.position.x = self.current_goal.pose.position.x
        self.last_goal.pose.position.y = self.current_goal.pose.position.y
        self.last_goal.pose.orientation.z = self.current_goal.pose.orientation.z
        self.last_goal.pose.orientation.w = self.current_goal.pose.orientation.w
        print("pub next goal", self.current_goal.pose.position.x, self.current_goal.pose.position.y)

    def obsHandle(self, req):
        print("dead detected")
        time.sleep(0.15)
        #if self.mark_received:
        #    if 1.744 < req.x < 2.374 and -3.679< req.y < -3.009:
        #        return True 
        yaw = math.atan2(req.y - self.current_map_pose.pose.pose.position.y,
                         req.x - self.current_map_pose.pose.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #self.last_goal.pose.position.x = self.current_goal.pose.position.x
        #self.last_goal.pose.position.y = self.current_goal.pose.position.y
        #self.last_goal.pose.orientation.z = self.current_goal.pose.orientation.z
        #self.last_goal.pose.orientation.w = self.current_goal.pose.orientation.w
        self.current_goal.header.frame_id = 'map'
        self.current_goal.header.stamp = rospy.Time.now()
        self.current_goal.pose.position.x = self.current_map_pose.pose.pose.position.x
        self.current_goal.pose.position.y = self.current_map_pose.pose.pose.position.y
        self.current_goal.pose.orientation.z = q[2]
        self.current_goal.pose.orientation.w = q[3]
        print("change current goal", self.current_goal.pose.position.x, self.current_goal.pose.position.y)
        self.last_state = self.state
        self.state = 3
        self.goal_pub.publish(self.current_goal)
	return True

    def person_client(self):
        print("waiting for person service")
        rospy.wait_for_service('person_num')
        try:
            person = rospy.ServiceProxy("person_num", personNum)
            res = person()
            self.personNum = res.twist
            print("glass", self.personNum.linear.x, "long", self.personNum.linear.y)
            if self.personNum.linear.x > 15:
                self.glasses_num = 15
            else:
                self.glasses_num = self.personNum.linear.x
            if self.personNum.linear.y > 15:
                self.longhair_num = 15
            else:
                self.longhair_num = self.personNum.linear.y
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)

    def broadcast(self):
        print('person client!!')
        if self.glasses_num < 20 and self.longhair_num < 20:
            if self.state != 7:
                self.state = 7
                self.person_client()
                playsound('/home/ucar/voice_ws/src/voice_library/D' +
                          str(int(self.food_id)) + '.wav')
                music = '/home/ucar/voice_ws/src/voice_library/戴眼镜' + \
                        str(int(self.glasses_num)) + '.wav'
                playsound(music)
                music = '/home/ucar/voice_ws/src/voice_library/长头发' + \
                        str(int(self.longhair_num)) + '.wav'
                playsound(music)

    def amclCallback(self, pose):
        self.current_map_pose = pose
        self.Run()

    def voiceCallback(self, msg):
        if msg.linear.x==1:
            self.current_goal = list2PoseStemped(self.nav_list[self.goal_id])
            self.current_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.current_goal)
            self.last_goal.pose.position.x = self.current_goal.pose.position.x
            self.last_goal.pose.position.y = self.current_goal.pose.position.y
            self.last_goal.pose.orientation.z = self.current_goal.pose.orientation.z
            self.last_goal.pose.orientation.w = self.current_goal.pose.orientation.w
            self.goal_type = self.goal_type_list[self.goal_id]
            self.goal_id = 1
            self.state = 1

    def marker_client(self):
        rospy.wait_for_service("codedetect")
        if True:
            a = 0
            b = 15
            while a != 1 and b > 0:
                try:
                    cuda = rospy.ServiceProxy("codedetect", code)
                    req = cuda(3)
                    self.food_id = req.twist.angular.x
                    print("food:", marker_to_food[self.food_id])
                    a, b = 1, 0
                    playsound('/home/ucar/voice_ws/src/voice_library/B' +
                              str(int(self.food_id)) + 'A.wav')
                    self.pub_next_goal()
                    self.mark_received = True
                    self.Run()
                except rospy.ServiceException as e:
                    print("service call failed %s", e)
                    a, b = 0, -1
                    marker_timer = threading.Timer(0.1,self.marker_client)
                    marker_timer.daemon = True
                    marker_timer.start()
        elif False:
            b = 20
            while b > 0:
                try:
                    cuda = rospy.ServiceProxy("codedetect", code)
                    req = cuda(int(self.parking_id))
                    self.assist_parking(
                        req.twist.linear.x - 0.1, req.twist.linear.y, req.twist.linear.z, 2)
                    b = 0
                    # fixme
                except rospy.ServiceException as e:
                    print("no parking code %s", e)
                    b -= 1
                    time.sleep(0.01)


if __name__ == '__main__':
    try:
        rospy.init_node("task_controller")
        # r = rospkg.RosPack()
        # if used launch file to start node, take such way to open the csv files
        # goal_path = os.path.join(r.get_path("ucar_nav"),"scripts/test.csv")
        # parking_path = os.path.join(r.get_path("ucar_nav"),"scripts/parking.csv")
        # Directly open python file in terminal, take such way to open the csv file
        goal_path = os.getcwd() + "/laser.csv"
        parking_path = os.getcwd() + '/parking_0607.csv'
        mission_task(goal_path, parking_path, int(sys.argv[1]))
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shuting down")
        exit()
