#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from playsound import playsound
import csv
import  random
import  math
import time
import  os
from darknet_ros_msgs.srv import *

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
        #self.person_sub = rospy.Subscriber("/person_num",Twist,self.personNumCallback,queue_size=1)
        self.parking_sub = rospy.Subscriber("/marker_id",Twist,self.markerCallback,queue_size=1)
        self.change_pub = rospy.Publisher("/change_mode",Twist,queue_size=1)
        #self.reinit_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size = 1)
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
        self.last_goal = self.current_goal
        self.current_goal.header.frame_id = "map"
        
        self.state = 0
        self.current_map_pose = PoseStamped()
        self.last_map_pose = PoseStamped()
        self.min_distance = [0.2,0.15,0.2]
        self.time_wait = time.time()

    def Run(self):
        dx = self.current_goal.pose.position.x - self.current_map_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.current_map_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        if self.state == 0:
            return
        elif self.state == 1:
            if distance < self.min_distance[self.goal_id-1]:
                if self.goal_id  == len(self.nav_list):
                    if self.mark_recived == True:
                        self.state  = 3
                        return
                    else:
                        return
                self.current_goal = list2PoseStemped(self.nav_list[self.goal_id])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.goal_id+=1
        elif self.state == 3:
            if distance < self.min_distance[self.goal_id-1]:
                temp_msg = Twist()
                temp_msg.linear.y = 1
                self.change_pub.publish(temp_msg)
                self.current_goal = list2PoseStemped(self.parking_list[int(self.mark_id)])
                self.current_goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.current_goal)
                self.state = 4
                self.time_wait = time.time()
        elif self.state == 4:
            if distance<0.2:
                if self.food_voice == False:
                    #playsound('/home/ucar/voice_ws/src/voice_library/delicious_full.wav')
                    playsound('/home/ucar/voice_ws/src/voice_library/'+str(int(self.mark_id))+'.wav')
                    playsound('/home/ucar/voice_ws/src/voice_library/waitng_personnum.wav')
                    self.food_voice = True
                if time.time() - self.time_wait >= 5:
                    self.state = 5
        elif self.state == 5:
            self.person_client()
            self.broadcast()
	    rospy.loginfo("戴眼镜 :%d 长头发: %d",self.glasses_num,self.longhair_num)
            print("finished!")
	    #self.state = 6
            

    def person_client(self):
        rospy.wait_for_service('person_num')
        try:
            person = rospy.ServiceProxy("person_num",person_num)
            req = person(True)
            self.glasses_num = req.glass
            self.longhair_num = req.longhair
        except rospy.ServiceException as e:
            print("service call failed: %s"%e)



    def broadcast(self):
        if self.glasses_num < 15 and self.longhair_num <15:
            if self.state != 6:
                self.state = 6
                music = '/home/ucar/voice_ws/src/voice_library/戴眼镜' + str(int(self.glasses_num)) + '.wav'
                playsound(music)
                music = '/home/ucar/voice_ws/src/voice_library/长头发' + str(int(self.longhair_num)) + '.wav'
                playsound(music)
               


    def amclCallback(self,pose):
        self.current_map_pose = pose
        dx = self.current_map_pose.pose.position.x - self.last_map_pose.pose.position.x
        dy = self.current_map_pose.pose.position.y - self.last_map_pose.pose.position.y
        if dx*dx + dy*dy > 9:
	    #initial_pose = PoseWithCovarianceStamped()
	    #initial_pose.header.stamp = rospy.Time.now()
	    #initial_pose.pose.pose = self.last_map_pose.pose
            #self.reinit_pub.publish(initial_pose)
	    rospy.loginfo("relocalization!!")
        else:
            self.last_map_pose = self.current_map_pose
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
        #self.current_goal = list2PoseStemped(self.nav_list[-1])
        #self.current_goal.header.stamp = rospy.Time.now()
        #self.goal_pub.publish(self.current_goal)
        self.mark_recived = True
        #self.state = 3

    #def personNumCallback(self,personNum):
        #if personNum.linear.x <4 and personNum.linear.y<4:
            #self.glasses_num = personNum.linear.x
            #self.longhair_num = personNum.linear.y
            #self.broadcast()

if __name__=='__main__':
    try:
        rospy.init_node("task_controller")
        #r = rospkg.RosPack()
        #if used launch file to start node, take such way to open the csv files
        #goal_path = os.path.join(r.get_path("ucar_nav"),"scripts/test.csv")
        #parking_path = os.path.join(r.get_path("ucar_nav"),"scripts/parking.csv")
        #Directly open python file in terminal, take such way to open the csv file
        goal_path = os.getcwd() + "/test.csv"
        parking_path = os.getcwd()+'/parking.csv'
        mission_task(goal_path,parking_path)
        rospy.spin()
    except  KeyboardInterrupt:
        rospy.loginfo("shuting down")
        exit()
