#!/usr/bin/env python
# -*- coding: utf-8 -*-
import  rospy   
from geometry_msgs.msg  import PoseStamped
import  csv
import  os
import  sys
import  time

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


def main(argv):
    print('parking goal id = '+argv[1])
    rospy.init_node("test_parking")
    parking_path = os.getcwd()+'/parking_0607.csv'
    goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
    parking_list = []
    with open(parking_path, 'rb') as f:
        reader = csv.reader(f)
        for cols in reader:
            parking_list.append([float(values) for values in cols])
    time.sleep(0.5)
    goal_ = list2PoseStemped(parking_list[int(argv[1])])
    goal_.header.stamp = rospy.Time.now()
    goal_pub.publish(goal_)
    time.sleep(0.5)



if __name__ == '__main__':
    main(sys.argv)
