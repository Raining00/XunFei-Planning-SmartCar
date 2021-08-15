#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import os
import random
import threading

import math
import rospy
import sys
import tf
import time
from codedetect.srv import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from playsound import playsound
from roborts_msgs.msg import TwistAccel
from roborts_msgs.srv import Obstacles, ObstaclesResponse
from tkdnn_ros_msgs.srv import *

marker_to_food = {0: "Vegetables",
                  1: "Fruits",
                  2: "Meat"}


parking_reverse_dict = {
    0: (-0.819, 0.574),
    1: (-0.707, 0.707),
    2: (-0.559, 0.829)
}


def list2PoseStemped(Point):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
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
        self.way_point = [0.04,-2.50,0.702,0.712]
        self.finishing_parking = False
        self.goal_type = 0
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
        self.is_parking = False
        self.food_id = 0
        self.mark_received = False
        self.food_voice = False
        self.vision_count = 0
        self.glasses_num = random.randint(0, 6)
        self.longhair_num = random.randint(0, 6)
        self.person_num = 0
        # navigation about
        self.goal_id = 0
        self.current_goal = PoseStamped()
        self.last_goal = PoseStamped()
        self.current_goal.header.frame_id = "map"
        self.state = 0
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
        self.enable_service = True
        self.exceptionCheck = threading.Timer(1, self.CheckException)
        self.exceptionCheck.daemon = True
        self.has_five = True

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
        dyaw = yaw_goal - yaw
        while (dyaw >= math.pi or dyaw < -math.pi):
            if dyaw >= math.pi:
                dyaw -= math.pi * 2
            elif dyaw < -math.pi:
                dyaw += math.pi * 2
        dyaw = abs(dyaw)
        print(self.state, "dis", round(distance, 3), "dyaw", round(dyaw, 3), "goal_id",
              self.goal_id, 'goal_type', self.goal_type)
        if self.state == 0:
            return
        elif self.state == 1:
            if self.goal_type == 1:  # 区域切换导航点
                # if distance < 0.5:
                # if self.goal_id == len(self.nav_list):
                # self.is_parking = True
                if distance < 0.15 and dyaw < 0.15:
                    if self.goal_id == len(self.nav_list) - 1:
                        self.is_parking = True
                        temp_msg = Twist()
                        if self.parking_id == 2:
                            temp_msg.linear.x = 1
                        else:
                            temp_msg.linear.x = 3
                        temp_msg.linear.y = 1
                        temp_msg.linear.z = 20
                        time.sleep(0.5)
                        self.change_pub.publish(temp_msg)
                        # self.pub_next_goal()
                        self.has_five = self.person_client()
                        if self.has_five:
                            if self.person_num == 2:
                                if self.parking_id == 2:
                                    way_point = [0.42, -1.5,0.702,0.712]
                                    self.current_goal = list2PoseStemped(way_point)
                                    self.goal_pub.publish(self.current_goal)
                                    self.state = 11
                                    return 
                                else:
                                    self.pub_next_goal(is_parking = True)
                            else:
                                self.pub_next_goal()
                            #self.current_goal = list2PoseStemped(self.way_point)
                            #self.goal_pub.publish(self.current_goal)
                        else:
                            #self.pub_next_goal(is_parking =True)
                            self.pub_next_goal(is_parking=True if self.person_num >= 2 else False)
                        # for having detected 2 models
                    else:
                        self.pub_next_goal()
                else:
                    self.goal_pub.publish(self.current_goal)
            elif self.goal_type == 2:
                if dyaw < 0.2:
                    temp_msg = Twist()
                    temp_msg.linear.y = 2
                    self.change_pub.publish(temp_msg)
                    time.sleep(0.5)
                    if self.has_five:
                        self.current_goal = list2PoseStemped(self.way_point)
                        self.goal_pub.publish(self.current_goal)
                        self.state =10
                    else:
                        self.pub_next_goal(is_parking=True)
                        self.state = 2
            elif self.goal_type == 3:  # 任务领取区导航点
                if distance < 0.2 and not self.coding: 
                    self.coding = True
                    self.enable_service = False
                    self.marker_client()
        elif self.state == 2:
            if distance <= 0.35:
                self.person_client()
                if self.person_num <2:
                    way_point = [self.current_goal.pose.position.x,self.current_goal.pose.position.y]
                    way_point.append(parking_reverse_dict[self.parking_id][0])
                    way_point.append(parking_reverse_dict[self.parking_id][1])
                    self.current_goal = list2PoseStemped(way_point)
                    self.goal_pub.publish(self.current_goal)
                    self.state = 12
                    return
                time.sleep(0.5)
                timer = threading.Timer(4, self.broadcast)
                timer.start()
                self.finishing_parking = True
                self.state = 4
        elif self.state == 3:
            if dyaw <= 0.15:
                temp_msg = Twist()
                temp_msg.linear.y = 1
                self.change_pub.publish(temp_msg)
                time.sleep(0.5)
                self.continueNav()
                print('continue nav')
        elif self.state == 10:
            if dyaw <= 0.20:
                temp_msg = Twist()
                temp_msg.linear.y = 2
                self.change_pub.publish(temp_msg)
                time.sleep(0.1)
                if self.parking_id == 2:
                    way_point = [0.42, -1.5,0.702,0.712]
                    self.current_goal = list2PoseStemped(way_point)
                    self.goal_pub.publish(self.current_goal)
                    self.state = 11
                else:
                    self.pub_next_goal(is_parking = True)
        elif self.state == 11:
            if distance<= 0.30:
                self.pub_next_goal(is_parking = True)
        elif self.state == 12:
            if dyaw < 0.15:
                temp_msg = Twist()
                temp_msg.linear.y = 2
                self.change_pub.publish(temp_msg)
                timer = threading.Timer(2, self.broadcast)
                timer.start()
                self.state = 4
    
    def pub_next_goal(self, is_parking=False):
        if is_parking == False:
            if self.goal_id == len(self.nav_list):
                return
            self.current_goal = list2PoseStemped(self.nav_list[self.goal_id])
            self.goal_type = self.goal_type_list[self.goal_id]
        elif is_parking == True:
            self.current_goal = list2PoseStemped(self.parking_list[self.parking_id])
            self.state = 2
            self.is_parking = True
        self.goal_id += 1
        self.goal_pub.publish(self.current_goal)
        print("pub next goal", self.current_goal.pose.position.x, self.current_goal.pose.position.y)

    def obsHandle(self, req):
        if self.state == 3:  # or self.is_parking == True:
            print('already in search, refused new request')
            return ObstaclesResponse(False)
        if self.enable_service == False:
            return ObstaclesResponse(False)
        if self.coding == True:
            print('coding! refused new request')
            return ObstaclesResponse(False)
        if req.type == 1:
            time.sleep(0.2)
        yaw = math.atan2(req.y - self.current_map_pose.pose.pose.position.y,
                         req.x - self.current_map_pose.pose.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.last_goal.pose.position.x = self.current_goal.pose.position.x
        self.last_goal.pose.position.y = self.current_goal.pose.position.y
        self.last_goal.pose.orientation.z = self.current_goal.pose.orientation.z
        self.last_goal.pose.orientation.w = self.current_goal.pose.orientation.w
        self.current_goal.header.frame_id = 'map'
        self.current_goal.header.stamp = rospy.Time.now()
        self.current_goal.pose.position.x = self.current_map_pose.pose.pose.position.x
        self.current_goal.pose.position.y = self.current_map_pose.pose.pose.position.y
        self.current_goal.pose.orientation.z = q[2]
        self.current_goal.pose.orientation.w = q[3]
        self.state = 3
        self.goal_pub.publish(self.current_goal)
        print('searching for target! ')
        return ObstaclesResponse(True)

    def continueNav(self):
        self.current_goal.header.frame_id = 'map'
        self.current_goal.header.stamp = rospy.Time.now()
        self.current_goal.pose.position.x = self.last_goal.pose.position.x
        self.current_goal.pose.position.y = self.last_goal.pose.position.y
        self.current_goal.pose.orientation.z = self.last_goal.pose.orientation.z
        self.current_goal.pose.orientation.w = self.last_goal.pose.orientation.w
        if not self.is_parking:
            self.state = 1
        else:
            self.state = 2
        self.goal_pub.publish(self.current_goal)
        time.sleep(0.2)

    def person_client(self):
        print("waiting for person service")
        rospy.wait_for_service('person_num')
        try:
            person = rospy.ServiceProxy("person_num", personNum)
            res = person()
            self.personNum = res.twist
            print("glass", self.personNum.linear.x, "long", self.personNum.linear.y, "person", self.personNum.linear.z,'has not 5?',int(self.personNum.angular.x))
            if self.personNum.linear.x > 2:
                self.glasses_num = 2
            else:
                self.glasses_num = self.personNum.linear.x
            if self.personNum.linear.y > 2:
                self.longhair_num = 2
            else:
                self.longhair_num = self.personNum.linear.y
            if self.personNum.linear.z > 2:
                self.person_num = 2
            else:
                self.person_num = self.personNum.linear.z
            if int(self.personNum.angular.x) == 1:
                return False
            else:
                return True
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)

    def broadcast(self):
        print('person client!!')
        if self.glasses_num < 20 and self.longhair_num < 20:
            if self.state != 7:
                self.state = 7
                self.person_client()

                playsound('/home/ucar/voice_ws/src/voice_library/' +
                          str(int(self.person_num)) + str(int(self.longhair_num)) + str(int(self.glasses_num)) + '.wav')

                # playsound('/home/ucar/voice_ws/src/voice_library/D' +
                #           str(int(self.food_id)) + '.wav')
                # music = '/home/ucar/voice_ws/src/voice_library/戴眼镜' + \
                #         str(int(self.glasses_num)) + '.wav'
                # playsound(music)
                # music = '/home/ucar/voice_ws/src/voice_library/长头发' + \
                #         str(int(self.longhair_num)) + '.wav'
                # playsound(music)

    def amclCallback(self, pose):
        self.current_map_pose = pose
        self.Run()

    def voiceCallback(self, msg):
        if msg.linear.x == 1:
            time.sleep(0.1)
            self.current_goal = list2PoseStemped(self.nav_list[self.goal_id])
            self.current_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.current_goal)
            self.goal_type = self.goal_type_list[self.goal_id]
            self.goal_id = 1
            self.state = 1
            self.exceptionCheck.start()

    def marker_client(self):
        rospy.wait_for_service("codedetect")
        a = 0
        b = 15
        while self.coding == True:
            try:
                cuda = rospy.ServiceProxy("codedetect", code)
                req = cuda(3)
                self.food_id = req.twist.angular.x
                playsound('/home/ucar/voice_ws/src/voice_library/B' +
                          str(int(self.food_id)) + 'A.wav')
                self.pub_next_goal()
                self.mark_received = True
                self.coding = False
                self.Run()
            except rospy.ServiceException as e:
                print("service call failed %s", e)

    def CheckException(self):
        if self.is_parking == True:
            self.goal_pub.publish(self.current_goal)
            self.exceptionCheck = threading.Timer(4, self.CheckException)
            self.exceptionCheck.daemon = True
            self.exceptionCheck.start()


if __name__ == '__main__':
    try:
        rospy.init_node("task_controller")
        goal_path = os.getcwd() + "/laser.csv"
        parking_path = os.getcwd() + '/parking.csv'
        if int(sys.argv[1]) not in [1, 2, 3]:
            raise "Parking goal error. Please input 1 or 2 or 3."
        mission_task(goal_path, parking_path, int(sys.argv[1]) - 1)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shuting down")
        exit()
