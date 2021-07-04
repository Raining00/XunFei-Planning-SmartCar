#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from playsound import playsound
import csv
import  random
import  math
import time
import  os

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

#we will define a param named state to control some special place
# state:
# 0:  wait for start msg
# 1:  start navigation
# 2: wait for code detect msg...
# 3:  an normal state between code detect and C area
# 4: wait for parking.  Specially, we will give a message to tell vision node to start taking photoes when the car reach the "C" area
# 5: successfully parking, start to broadcast, specially, we will wait for five second to make sure detecting result can be recived.
 #6: finishing 
class mission_task:
    def __init__(self,goal_path,parking_path):
        self.amcl_sub = rospy.Subscriber("/amcl_pose",PoseStamped,self.amclCallback,queue_size=2)
        self.voice_sub = rospy.Subscriber("nav_start",Twist,self.voiceCallback,queue_size=1)
        self.person_sub = rospy.Subscriber("/person_num",Twist,self.personNumCallback,queue_size=1)
        self.parking_sub = rospy.Subscriber("/marker_id",Twist,self.markerCallback,queue_size=1)
        self.change_pub = rospy.Publisher("/change_mode",Twist,queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)

        #nav point
        self.nav_list = []
        self.parking_list = []
        with open(goal_path,'rb') as f:
            reader = csv.reader(f)
            for cols in reader:
                self.nav_list.append([float(values) for values in cols])
        with open(parking_path,'rb') as f:
            reader = csv.reader(f)
            for cols in reader:
                self.parking_list.append([float(values) for values in cols])

        #vision about
        self.mark_id = 0
        self.mark_recived = False
        self.food_voice = False
        self.vision_count = 0
        self.glasses_num = random.randint(0,2)
        self.longhair_num = random.randint(0,2)

        #navigation about
        self.goal_id = 0
        self.current_goal = PoseStamped()
        self.current_goal.header.frame_id = "map"
        
        self.state = 0
        self.current_map_pose = PoseStamped()
        self.min_distance = 0.25
        self.time_wait = time.time()

    def Run(self):
        dx = self.current_goal.pose.position.x - self.current_map_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.current_map_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        if self.state == 0:
            return
        elif self.state == 1:
            if distance < self.min_distance:
                if self.goal_id  == len(self.nav_list)-1:
                    if self.mark_recived == False:
                        self.state  = 2
                self.current_goal = list2PoseStemped(self.nav_list[self.goal_id])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.goal_id+=1
        elif self.state == 3:
            if distance < self.min_distance:
                temp_msg = Twist()
                temp_msg.linear.x = 1
                temp_msg.linear.y = 1
		temp_msg.linear.z = 20
                self.change_pub.publish(temp_msg)
                self.current_goal = list2PoseStemped(self.parking_list[int(self.mark_id)])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.state = 4
                self.time_wait = time.time()
        elif self.state == 4:
            if distance<0.2:
                if self.food_voice == False:
                    playsound('/home/ucar/voice_ws/src/voice_library/'+str(int(self.mark_id))+'.wav')
                    playsound('/home/ucar/voice_ws/src/voice_library/waitng_personnum.wav')
                    self.food_voice = True
                if time.time() - self.time_wait >= 3:
                    self.state = 5
        elif self.state == 5:
            print("finished!")
            self.broadcast()


    def broadcast(self):
        if self.glasses_num < 3 and self.longhair_num <3:
            if self.state != 6:
                self.state = 6
                music = '/home/ucar/voice_ws/src/voice_library/戴眼镜' + str(int(self.glasses_num)) + '.wav'
                playsound(music)
                music = '/home/ucar/voice_ws/src/voice_library/长头发' + str(int(self.longhair_num)) + '.wav'
                playsound(music)
               


    def amclCallback(self,pose):
        self.current_map_pose = pose
        self.Run()

    def voiceCallback(self,msg):
        if msg.linear.x == 1:
            self.current_goal = list2PoseStemped(self.nav_list[0])
            self.current_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.current_goal)
            self.goal_id+=1
            self.state = 1
    
    def markerCallback(self,msg):
        self.mark_id = msg.linear.x
        self.current_goal = list2PoseStemped(self.nav_list[-1])
        self.current_goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.current_goal)
        self.mark_recived = True
        self.state = 3

    def personNumCallback(self,personNum):
        if personNum.linear.x <3 and personNum.linear.y<3:
            self.glasses_num = personNum.linear.x
            self.longhair_num = personNum.linear.y
            #self.broadcast()

if __name__=='__main__':
    try:
        rospy.init_node("task_controller")
        goal_path = os.getcwd() + "/test2.csv"
        parking_path = os.getcwd()+'/parking.csv'
        mission_task(goal_path,parking_path)
        rospy.spin()
    except  KeyboardInterrupt:
        rospy.loginfo("shuting down")
        exit()

