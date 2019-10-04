#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
import set_point

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import LinkStates
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class Controller:
    def __init__(self):
        #Drone state
        self.state = State()
        self.uav_pos = Point(0.0, 0.0, 3.0)
        self.load_pos = Point(0.0, 0.0, 0.0)
        # Instantiate a attitude setpoints message
        self.sp = AttitudeTarget()
        # set the flag to use position setpoints and yaw angle
        #http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        self.sp.type_mask = int('10000000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        self.sp.body_rate.x = 0
        self.sp.body_rate.y = 0
        self.sp.body_rate.z = 0
        self.sp.thrust = 0

        # Instantiate a position setpoints message
        self.pos_sp = PositionTarget()
        self.pos_sp.type_mask = int('010111111000', 2)
        self.pos_sp.coordinate_frame = 1
        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = 3

        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0

    def position(self,msg):
        self.load_pos.x = msg.pose[28].position.x
        self.load_pos.y = msg.pose[28].position.y
        self.load_pos.z = msg.pose[28].position.z

        self.uav_pos.x = msg.pose[1].position.x
        self.uav_pos.y = msg.pose[1].position.y
        self.uav_pos.z = msg.pose[1].position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = 5.0

    def commander(self):

def main():
    # initiate node
    rospy.init_node('odometry_node', anonymous=True)

    # flight mode object
    modes = set_point.fcuModes()

    # flight Controller
    cnt = Controller()
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to UAV and Payload Position
    rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.posLoad)

    # Attitude Setpoint publisher
    at_pub = rospy.Publisher('mavros/setpoint_raw/attitude', PositionTarget, queue_size=1)

    # Position Setpoint Publisher
    pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    print('armed')

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    while cnt.uav_pos.z<5:
        print("uav_pos_z:",cnt.uav_pos.z)
        print("load_pos_z:",cnt.load_pos.z)
        cnt.updateSp()
    	pos_pub.publish(cnt.pos_sp)
    print("reached")
    
    # ROS main loop
    while not rospy.is_shutdown():
        sp_pub.publish(cnt.pos_sp)
        if cnt.uav_pos.z>1:
            print("uav_x",cnt.uav_pos.x)
            print("uav_y",cnt.uav_pos.y)
            print("load_x",cnt.load_pos.x)
            print("load_y",cnt.load_pos.y)
        print("----------------")
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
