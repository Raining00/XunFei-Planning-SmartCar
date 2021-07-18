#!/usr/bin/env python
# -*- coding: utf-8 -*-


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


# we will define a param named state to control some special place
# state:
# 0:  wait for start msg
# 1:  start navigation
# 2: wait for code detect msg...
# 3:  an normal state between code detect and C area
# 4: wait for parking.  Specially, we will give a message to tell vision node to start taking photoes when the car reach the "C" area
# 5: successfully parking, start to broadcast, specially, we will wait for five second to make sure detecting result can be recived.
 # 6: finishing
class mission_task:
    def __init__(self, goal_path, parking_path, parking_id):
        # nav point
        self.nav_list = []
        self.parking_list = []
        with open(goal_path, 'rb') as f:
            reader = csv.reader(f)
            for cols in reader:
                self.nav_list.append([float(values) for values in cols])
        with open(parking_path, 'rb') as f:
            reader = csv.reader(f)
            for cols in reader:
                self.parking_list.append([float(values) for values in cols])

        # vision about
        self.parking_id = parking_id
        self.food_id = 0
        self.mark_recived = False
        self.food_voice = False
        self.vision_count = 0
        self.glasses_num = random.randint(0, 2)
        self.longhair_num = random.randint(0, 2)
        self.swing_head = False
        # navigation about
        self.goal_id = 0
        self.current_goal = PoseStamped()
        self.last_goal = self.current_goal
        self.current_goal.header.frame_id = "map"
        self.count = 0
        self.state = 0
        self.current_map_pose = PoseWithCovarianceStamped()
        self.last_map_pose = PoseWithCovarianceStamped()
        self.min_distance = [0.80, 0.15, 0.12, 0.1]

        self.exceptionCheck = threading.Timer(27, self.CheckException)
        self.exceptionCheck.daemon = True

        # Exception handling
        #self.time_start = time.time()
        #self.wait_time = [10, 10, 10, 10,10]

        self.amcl_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amclCallback, queue_size=1)
        self.voice_sub = rospy.Subscriber(
            "nav_start", Twist, self.voiceCallback, queue_size=1)
        # self.person_sub = rospy.Subscriber("/person_num",Twist,self.personNumCallback,queue_size=1)
        # self.parking_sub = rospy.Subscriber("/parking_id",Twist,self.markerCallback,queue_size=1)
        self.change_pub = rospy.Publisher("/change_mode", Twist, queue_size=1)
        # self.reinit_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size = 1)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
        self.assist_parking_pub = rospy.Publisher(
            "/cmd_vel_acc", TwistAccel, queue_size=1)
        self.stop_nav = rospy.Publisher("/stop_nav", Twist, queue_size=1)

    def Run(self):
        dx = self.current_goal.pose.position.x - \
            self.current_map_pose.pose.pose.position.x
        dy = self.current_goal.pose.position.y - \
            self.current_map_pose.pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        print(self.state, "dis", distance, "goal_id", self.goal_id)
        if self.state == 0:
            return
        elif self.state == 1:
            if distance < self.min_distance[self.goal_id-1]:
                #if self.goal_id == 1:
                    #temp_msg = Twist()
                    #temp_msg.linear.x = 2
                    #self.change_pub.publish(temp_msg)
                if self.goal_id == len(self.nav_list)-2:
                    self.marker_client(1)
                    playsound('/home/ucar/voice_ws/src/voice_library/B' +
                              str(int(self.food_id))+'A.wav')
                    if self.mark_recived == True:
                        self.state = 3
                    else:
                        return
                self.current_goal = list2PoseStemped(
                    self.nav_list[self.goal_id])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.exceptionCheck = threading.Timer(8, self.CheckException)
                # self.exceptionCheck.daemon=True
                self.exceptionCheck.start()
                self.goal_id += 1
            else:
                self.goal_pub.publish(self.current_goal)
        elif self.state == 3:
            if self.swing_head == True:
                yaw = math.atan2(2*(self.current_map_pose.pose.pose.orientation.w*self.current_map_pose.pose.pose.orientation.z),
                                 1-2*self.current_map_pose.pose.pose.orientation.z*self.current_map_pose.pose.pose.orientation.z)
                yaw_goal = math.atan2(2*self.current_goal.pose.orientation.w*self.current_goal.pose.orientation.z,
                                      1 - 2 * self.current_goal.pose.orientation.z*self.current_goal.pose.orientation.z)
                if abs(yaw_goal - yaw) < 0.1:
                    temp_msg = Twist()
                    temp_msg.linear.x = 1
                    temp_msg.linear.y = 1
                    temp_msg.linear.z = 20
                    self.change_pub.publish(temp_msg)
                    vel_msg = TwistAccel()
                    self.assist_parking_pub.publish(vel_msg)
                    os.system("sleep 0.2")
                    vel_msg = TwistAccel()
                    self.assist_parking_pub.publish(vel_msg)
                    self.current_goal = list2PoseStemped(
                        self.nav_list[self.goal_id])
                    self.current_goal.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(self.current_goal)
                    self.state = 4
                    self.exceptionCheck = threading.Timer(
                        4, self.CheckException)
                    # self.exceptionCheck.daemon=True
                    self.exceptionCheck.start()
            elif distance < 0.1:
                print('swing head  = False .......')
                temp_msg = Twist()
                temp_msg.linear.x = 1
                temp_msg.linear.y = 1
                temp_msg.linear.z = 20
                self.change_pub.publish(temp_msg)
                vel_msg = TwistAccel()
                self.assist_parking_pub.publish(vel_msg)
                os.system("sleep 0.3")
                self.current_goal = list2PoseStemped(
                    self.parking_list[self.parking_id])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.state = 5
                self.exceptionCheck = threading.Timer(4, self.CheckException)
                # self.exceptionCheck.daemon=True
                self.exceptionCheck.start()
            else:
                self.goal_pub.publish(self.current_goal)
        elif self.state == 4:
            yaw = math.atan2(2*(self.current_map_pose.pose.pose.orientation.w*self.current_map_pose.pose.pose.orientation.z),
                             1-2*self.current_map_pose.pose.pose.orientation.z*self.current_map_pose.pose.pose.orientation.z)
            yaw_goal = math.atan2(2*self.current_goal.pose.orientation.w*self.current_goal.pose.orientation.z,
                                  1 - 2 * self.current_goal.pose.orientation.z*self.current_goal.pose.orientation.z)
            if abs(yaw_goal - yaw) < 0.1:
                vel_msg = TwistAccel()
                self.assist_parking_pub.publish(vel_msg)
                temp_msg = Twist()
                temp_msg.linear.y = 1
                self.change_pub.publish(temp_msg)
                os.system("sleep 0.2")
                self.current_goal = list2PoseStemped(
                    self.parking_list[self.parking_id])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.state = 5
                self.exceptionCheck = threading.Timer(4, self.CheckException)
                # self.exceptionCheck.daemon=True
                self.exceptionCheck.start()
        elif self.state == 5:
            if distance < 0.35:
                #stop_msg = Twist()
                #stop_msg.linear.x = 1
                #self.stop_nav.publish(stop_msg)
                #vel_msg = TwistAccel()
                #self.assist_parking_pub.publish(vel_msg)
                self.marker_client(2)
                timer = threading.Timer(4, self.broadcast)
                timer.daemon = True
                timer.start()
                self.state = 6
            self.count += 1
        elif self.state == 6:
            vel_msg = TwistAccel()
            self.assist_parking_pub.publish(vel_msg)
            self.person_client()
            rospy.loginfo("戴眼镜 :%d 长头发: %d",
                          self.glasses_num, self.longhair_num)
            print("time total", time.time()-self.time_start)
            print("finished!")
        elif self.state == 8:
            zero_cmd = TwistAccel()
            self.assist_parking_pub.publish(zero_cmd)

    def person_client(self):
        print("Waiting for person service")
        rospy.wait_for_service('person_num')
        try:
            person = rospy.ServiceProxy("person_num", personNum)
            res = person()
            self.personNum = res.twist
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
        if self.glasses_num < 20 and self.longhair_num < 20:
            if self.state != 7:
                self.state = 8
                self.person_client()
                playsound('/home/ucar/voice_ws/src/voice_library/D' +
                          str(int(self.food_id))+'.wav')
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
        if msg.linear.x == 1:
            self.swing_head = False
        elif msg.linear.x == 2:
            self.swing_head = True
            self.nav_list = []
            with open('1_head.csv', 'rb') as f:
                reader = csv.reader(f)
                for cols in reader:
                    self.nav_list.append([float(values) for values in cols])
        self.current_goal = list2PoseStemped(self.nav_list[0])
        self.current_goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.current_goal)
        self.goal_id = 1
        self.state = 1
        self.time_start = time.time()
        self.exceptionCheck.start()
        '''
        if msg.linear.x == 3:
            change = Twist()
            change.linear.x = 3
            change.linear.z = 20
            self.change_pub.publish(change)
        elif msg.linear.x == 4:
            change = Twist()
            change.linear.x = 2
            change.linear.z = 20
            self.change_pub.publish(change)
            '''

    def marker_client(self, mode):
        rospy.loginfo("Waiting for mark service")
        rospy.wait_for_service("codedetect")
        if mode == 1:
            a = 0
            while a != 1:
                try:
                    cuda = rospy.ServiceProxy("codedetect", code)
                    req = cuda(3)
                    self.food_id = req.twist.angular.x
                    print("food:", marker_to_food[self.food_id])
                    self.mark_recived = True
                    a = 1
                except rospy.ServiceException as e:
                    print("service call failed %s", e)
                    a = 0
        elif mode == 2:
            b = 20
            while b > 0:
                try:
                    cuda = rospy.ServiceProxy("codedetect", code)
                    req = cuda(int(self.parking_id))
                    self.assist_parking(
                        req.twist.linear.x-0.1, req.twist.linear.y, req.twist.linear.z, 2)
                    b = 0
                    # fixme
                except rospy.ServiceException as e:
                    print("no parking code %s", e)
                    b -= 1
                    time.sleep(0.01)

#    def markerCallback(self,msg):
#        self.parking_id = msg.linear.x
#        self.mark_recived = True
    def assist_parking(self, dx, dy, dyaw, mode):
        dt = 0.5
        print("dx: ", dx, ", dy: ", dy, ", dyaw: ", dyaw)
        if 0:
            print("no need to assist")
            return
        else:
            print("need assisting!")
            Vx = dx/dt
            Vy = dy/dt
            cmd_vel_acc = TwistAccel()
        if mode == 1:
            pass
        if mode == 2:
            temp_vx=Vx*math.cos(dyaw) - Vy*math.sin(dyaw)
            if temp_vx<=0:
                return
            cmd_vel_acc.twist.linear.x = temp_vx
            cmd_vel_acc.twist.linear.y = (
                Vx*math.sin(dyaw) + Vy*math.cos(dyaw))
            # print(cmd_vel_acc.twist.linear.y)
            cmd_vel_acc.twist.angular.z = dyaw/dt
            self.assist_parking_pub.publish(cmd_vel_acc)
            time.sleep(dt)
            cmd_vel_acc.twist.linear.x = 0
            cmd_vel_acc.twist.linear.y = 0
            cmd_vel_acc.twist.angular.z = 0
            self.assist_parking_pub.publish(cmd_vel_acc)
            time.sleep(dt)

    def CheckException(self):
        if self.state == 1:
            if self.goal_id != len(self.nav_list)-2:
                self.marker_client(1)
                playsound('/home/ucar/voice_ws/src/voice_library/B' +
                          str(int(self.food_id))+'.wav')
                if self.mark_recived == True:
                    self.state = 3
            else:
                self.goal_pub.publish(self.current_goal)
            self.exceptionCheck = threading.Timer(8, self.CheckException)
            # self.exceptionCheck.daemon=True
            self.exceptionCheck.start()
        elif self.state == 3 or self.state == 4 or self.state == 5:
            #self.goal_pub.publish(self.current_goal)
            self.exceptionCheck = threading.Timer(4, self.CheckException)
            # self.exceptionCheck.daemon=True
            self.exceptionCheck.start()


if __name__ == '__main__':
    try:
        rospy.init_node("task_controller")
        # r = rospkg.RosPack()
        # if used launch file to start node, take such way to open the csv files
        # goal_path = os.path.join(r.get_path("ucar_nav"),"scripts/test.csv")
        # parking_path = os.path.join(r.get_path("ucar_nav"),"scripts/parking.csv")
        # Directly open python file in terminal, take such way to open the csv file
        goal_path = os.getcwd() + "/1.csv"
        parking_path = os.getcwd()+'/parking.csv'
        mission_task(goal_path, parking_path, int(sys.argv[1]))
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shuting down")
        exit()
