#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import LinkStates
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class Controller:
    def __init__(self):
        self.uav_pos = Point(0.0, 0.0, 3.0)
        self.load_pos = Point(0.0, 0.0, 0.0)
        self.rel_pos = Point(0.0, 0.0, 3.0)
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        print(self.sp)

    def posLoad(self,msg):
        self.uav_pos.x = msg.pose[21].position.x
        self.uav_pos.y = msg.pose[21].position.y
        self.uav_pos.z = msg.pose[21].position.z

        #print(msg.pose[21].position.x) #pose of base_link
        #print(msg.pose[22])#pose of payload
        #print(msg.name)

    def posUAV(self,msg):
        self.load_pos.x = msg.pose.position.x
        self.load_pos.y = msg.pose.position.y
        self.load_pos.z = msg.pose.position.z
        #print(self.load_pos.z)

    def commander(self):
        self.rel_pos.x = self.uav_pos.x - self.load_pos.x
        self.rel_pos.y = self.uav_pos.y - self.load_pos.y
        self.rel_pos.z = self.uav_pos.z - self.load_pos.z

    
def test():
    # initiate node
    rospy.init_node('odometry_node', anonymous=True)
    rate = rospy.Rate(20.0)

    cnt = Controller()

    # Subscribe to UAV Position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posUAV)

    # Subscribe to payload Position
    rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.posLoad)
    rospy.spin()

if __name__ == '__main__':
    test()
