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

import time

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # initial values for setpoints
        self.sp.position.x = 0.90
        self.sp.position.y = 2.205
        self.sp.position.z = 3.2

        self.cenx = 0.761
        self.ceny = 0.893
        self.cenx = 0.90
        self.ceny = 2.205

        self.radi = 0.2

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self,t):
        if t< 10.0:
	    self.sp.position.x = 0.90
	    self.sp.position.y = 2.205
        self.sp.position.z = 1.6
        if t < 25.0:
            self.sp.position.x = self.cenx + self.radi;
            self.sp.position.y = self.ceny;
        else:
            self.sp.position.x = self.cenx + self.radi*np.cos((t-25.0)/2.0*2*np.pi);
            self.sp.position.y = self.ceny + self.radi*np.sin((t-25.0)/2.0*2*np.pi);

# Main function
def main():

    start = time.time()
    #setpoints for the trajectory
    #radius = 4.0
    freq = 100.0
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(freq)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    print("start")
    while not rospy.is_shutdown():
        fin = time.time()
        t = fin-start
    	cnt.updateSp(t)
    	sp_pub.publish(cnt.sp)
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
