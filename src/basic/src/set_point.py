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

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self,count,count0):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        #check http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 1.0
        # update the setpoint message with the required altitude
        self.sp.position.z = 0
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        self.count = count
        self.count0 = count0
        self.data = np.zeros((self.count0,10))

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks
    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self,t,height,radius,steps):
        self.sp.position.x = radius*np.cos(2*np.pi*t/steps)
        self.sp.position.y = radius*np.sin(2*np.pi*t/steps)
        z_points = 1500

        self.sp.position.z = height
        #if t<1500:
        #    self.sp.position.z = height/1500*t
        #    print(self.sp.position.z)
        #else:
        #    self.sp.position.z = height

    def get_position(self,msg):
        if self.count==self.count0:
            print("logging started")
            self.time = time.time()

        if self.count<=self.count0 and self.count>0:
            print(self.count)
            row = self.count0-self.count
            current_time = time.time()-self.time
            self.data[row][0] = current_time
            self.data[row][1] = msg.pose[87].position.x
            self.data[row][2] = msg.pose[87].position.y
            self.data[row][3] = msg.pose[87].position.z
            self.data[row][4] = msg.pose[1].position.x
            self.data[row][5] = msg.pose[1].position.y
            self.data[row][6] = msg.pose[1].position.z
            self.data[row][7] = msg.pose[207].position.x
            self.data[row][8] = msg.pose[207].position.y
            self.data[row][9] = msg.pose[207].position.z
        if self.count ==1:
            print("done")
            np.savetxt('test.csv',self.data,delimiter=',')
        self.count -= 1

# Main function
def main():
    #setpoints for the trajectory
    radius = 4.0
    freq = 100.0

    times = 5.0#time to make one circle

    count0 = 10000 #this is the time

    count = count0+20000

    waits = 300
    height = 23.0

    steps = freq*times

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller(count,count0)

    # ROS loop rate
    rate = rospy.Rate(freq)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to gazebo link_states
    rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.get_position)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Attitude Publisher
    #sp_att = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    modes.setTakeoff()
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
    t = 0
    print("start")
    while not rospy.is_shutdown():
    	cnt.updateSp(t,height,radius,steps)
    	sp_pub.publish(cnt.sp)
        t+=1
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
