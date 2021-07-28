 
import rospy
from roborts_msgs.srv import *

def test_server(req):
    print(req.x,req.y)
    return True

if __name__=="__main__":
    rospy.init_node("obstacle_test")
    s=rospy.Service("obstacles",Obstacles,test_server)
    print("ready")
    rospy.spin()
