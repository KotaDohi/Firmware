#!/usr/bin/env python
# ROS python API
import rospy
import time
import numpy as np

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import LinkStates
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import euler_from_quaternion

class Controller:
    def __init__(self,counter):
        self.uav_pos = Point(0.0, 0.0, 3.0)
        self.load = Point(0.0, 0.0, 0.0)
        self.data = []
        self.count0 = counter
        self.count = counter

    def get_position(self,msg):
        print(msg.pose[37].position.x)
        """
        self.load.x = msg.pose[37].position.x
        self.load.y = msg.pose[37].position.y
        self.load.z = msg.pose[37].position.z
        print("ore")
        if self.count>0:
            print(self.count)
            x = msg.pose[37].position.x
            y = msg.pose[37].position.y
            z = msg.pose[37].position.z
            self.data.append([self.count0-self.count,x,y,z])
            self.count -= 1
        if self.count ==0:
            print("done")
            np.savetxt('test.csv',self.data,delimiter='')
        """

def main():
    # initiate node
    count = 1000
    rospy.init_node('bag', anonymous=True)
    cnt = Controller(count)
    rate = rospy.Rate(100.0)
    rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.get_position)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
